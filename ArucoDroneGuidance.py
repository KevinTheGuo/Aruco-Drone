import cv2
import cv2.aruco
import numpy as np
import math

# CONSTANTS TO CHANGE:
camera_matrix = np.array( [[ 612.28318156,    0.        ,  377.95149893],
                           [   0.        ,  603.72201353,  273.9850609 ],
                           [   0.        ,    0.        ,    1.        ]])

dist_coeffs = np.array(   [[ 0.45710893, -0.68904653,  0.05444916, -0.01899903, -1.79491808]])
MARKER_SIDE_INCHES = 7.5
MARKER_SIDE_FEET = MARKER_SIDE_INCHES/12


# Generate our specific aruco dict
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

# Helper function to prune through all detected IDs and return executable and ignored command IDs
# INPUT: list of ids     (list)
# OUTPUT: move_dir       (int or None)
#         match_orient   (int or None)
#         match_pos      (int or None)
#         change_alt     (int or None)
#         ignored_cmds   (list)
#         num_duplicates (int)
def parseImageCmds(ids):
    # Initialize our return variables
    move_dir_cmd, match_orient_cmd, match_pos_cmd, change_alt_cmd, ignored_cmds, num_duplicates = None, None, None, None, [], 0

    # Flatten and sort ids to make getting the commands easier.
    ids = [item for items in ids for item in items]
    ids.sort()

    # Iterate through all the ids, putting the lowest id from each category in our return variable, and discarding others
    for id_num in ids:
        if (id_num >= 120) and (id_num <= 129) and (move_dir_cmd is None): # Check to make sure in range, and unset
            move_dir_cmd = id_num
        elif (id_num >= 220) and (id_num <= 229) and (match_orient_cmd is None):
            match_orient_cmd = id_num
        elif (id_num >= 320) and (id_num <= 329) and (match_pos_cmd is None):
            match_pos_cmd = id_num
        elif (id_num >= 420) and (id_num <= 429) and (change_alt_cmd is None):
            change_alt_cmd = id_num
        else:
            ignored_cmds.append(id_num)     # Any non-defined or already-set command ID is ignored

    # Check for duplicates. This means an id which is in both a cmd return variable and in the ignored_cmds list
    if move_dir_cmd in ignored_cmds:
        move_dir_cmd = None    # Indicate that this id is duplicated!
        num_duplicates+= 1
    if match_orient_cmd in ignored_cmds:
        match_orient_cmd = None
        num_duplicates+= 1
    if match_pos_cmd in ignored_cmds:
        match_pos_cmd = None
        num_duplicates+= 1
    if change_alt_cmd in ignored_cmds:
        change_alt_cmd = None
        num_duplicates+= 1

    # Check if any commands conflict with each other, and discard lower-priority commands
    if change_alt_cmd == 420:         # Landing overrides all other commands
        if move_dir_cmd is not None:  # Make sure each command is valid before adding it to ignored
            ignored_cmds.append(move_dir_cmd)
            move_dir_cmd = None
        if match_orient_cmd is not None:
            ignored_cmds.append(match_orient_cmd)
            match_orient_cmd = None
        if match_pos_cmd is not None:
            ignored_cmds.append(match_pos_cmd)
            match_pos_cmd = None
    elif (move_dir_cmd is not None) and (match_pos_cmd is not None):  # Match_pos takes priority over move_dir_cmd
        ignored_cmds.append(move_dir_cmd)
        move_dir_cmd = None

    # Return all our values
    return move_dir_cmd, match_orient_cmd, match_pos_cmd, change_alt_cmd, ignored_cmds, num_duplicates


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])



# Drone Marker System and Commands:

# Move in given direction:     ID 120-129
# Match marker orientation:    ID 220-229
# Match marker position:       ID 320-329
# Change altitude:             ID 420-429   (desired altitude ranges from 0-9 based on marker ID)
# Marker 420 (0 altitude) will override all other marker commands and trigger automated landing+shutoff sequence

# COMMAND PRIORITY:
# All lower-valued ID commands take priority over higher valued IDs of the same command, when detected at the same time
# If two identical IDs appear, the application will ignore them

# If two commands are compatible, they can be executed at the same time
# Two incompatible commands will have one take priority over the other. Command ID 420 is incompatible with all others
# Movement in a given direction is incompatible with matching marker position, and will be ignored if both are seen
# Changing altitude is compatible with all other commands
# Matching marker orientation is compatible with all other commands
# Matching marker position is incompatible with movement in a given direction, and takes priority over it

# Constants used to tune program response
ALT_DIVISOR = 1 # The higher this number is, the higher the drone will want to be.

# Initialize commands that will be "given" to the drone
x_heading = 0  # positive means go right, negative means go left
y_heading = 0  # positive means go up, negative means go down
rotation = 0   # rotation is clockwise
altitude_change = 0   # positive means raise altitude, negative means lower altitude

# Try to grab video input
video = cv2.VideoCapture(0)
if not video.isOpened():
    print('Could not open video!')
    exit()
# Tentative size of video: 640x480?

try:
    while(True):
        # Check for user input
        key = cv2.waitKey(1) & 0xff
        if key == 27:
            break

        # Grab another frame
        read_success, frame = video.read()
        if not read_success:
            print('Cannot read video file!')
            break

        # Detect and display all detected markers
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(frame, aruco_dict)
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Parse the id list for valid commands, keeping track of ignored commands and duplicates
        if (len(corners) > 0):
            move_dir_cmd, match_orient_cmd, match_pos_cmd, change_alt_cmd, ignored_cmds, num_duplicates = parseImageCmds(ids)

            # Execute each command, if valid
            # Move the way that the marker is pointing
            if move_dir_cmd is not None:
                move_dir_corners = corners[np.nonzero(ids == move_dir_cmd)[0][0]]    # Get the corners corresponding to this ID, and draw pose
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(move_dir_corners, MARKER_SIDE_FEET, camera_matrix, dist_coeffs)
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 1)
                rotation_matrix, jacobian = cv2.Rodrigues(rvec)
                euler = rotationMatrixToEulerAngles(rotation_matrix)
                x_heading = math.sin(euler[2]) # sin/cos flipped around because of the way markers are.
                y_heading = math.cos(euler[2])

            # Rotate until we're match the marker's rotation
            if match_orient_cmd is not None:
                match_orient_corners = corners[np.nonzero(ids == match_orient_cmd)[0][0]]
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(match_orient_corners, MARKER_SIDE_FEET, camera_matrix, dist_coeffs)
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 1)
                rotation_matrix, jacobian = cv2.Rodrigues(rvec)
                euler = rotationMatrixToEulerAngles(rotation_matrix)
                rotation = -euler[2]

            # Move so that the marker is in the middle of the screen
            if match_pos_cmd is not None:
                match_pos_corners = corners[np.nonzero(ids == match_pos_cmd)[0][0]]
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(match_pos_corners, MARKER_SIDE_FEET, camera_matrix, dist_coeffs)
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 1)
                x_heading = tvec[0]
                y_heading = -tvec[1]

            # Change distance from the marker to a specified amount
            if change_alt_cmd is not None:
                change_alt_corners = corners[np.nonzero(ids == change_alt_cmd)[0][0]]
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(change_alt_corners, MARKER_SIDE_FEET, camera_matrix, dist_coeffs)
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 1)
                desired_altitude = change_alt_cmd - 420 # Calculate desired altitude from ID of marker
                altitude_change = -(tvec[2]/ALT_DIVISOR - desired_altitude)

            # Print out ignored commands and duplicates
            if len(ignored_cmds) > 0:
                cv2.putText(frame, "Ignored: {}".format(ignored_cmds), (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if num_duplicates > 0:
                cv2.putText(frame, "# of Duplicates: {}".format(num_duplicates), (10, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


            # OBSOLETE:USE FOR REFERENCE --------------------------------
            obs_rvecs, obs_tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], MARKER_SIDE_FEET, camera_matrix, dist_coeffs)

            for i in range(0,len(rvecs)):
                obs_rvec = obs_rvecs[i][0]
                obs_tvec = obs_tvecs[i][0]     # 0 is a placeholder? try multiple markers

                # Get pitch/roll/yaw from rvecs
                obs_rotation_matrix, jacobian = cv2.Rodrigues(obs_rvec)
                obs_euler = rotationMatrixToEulerAngles(obs_rotation_matrix)

                cv2.putText(frame, "CurrPosition   X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(obs_tvec[0],obs_tvec[1],obs_tvec[2]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(frame, "CurrRotation   X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(obs_euler[0], obs_euler[1], obs_euler[2]), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Visualize X-Y translation
        cv2.arrowedLine(frame, (320,240), (int(320+20*x_heading),int(240-20*y_heading)), (0,0,255), 4)

        # Visualize rotation around Z-axis
        cv2.arrowedLine(frame, (320,200), (int(320+15*rotation),int(200)), (255,50,50), 4)
        cv2.arrowedLine(frame, (360,240), (int(360),int(240+15*rotation)), (255,50,50), 4)
        cv2.arrowedLine(frame, (320,280), (int(320-15*rotation),int(280)), (255,50,50), 4)
        cv2.arrowedLine(frame, (280,240), (int(280),int(240-15*rotation)), (255,50,50), 4)

        # Visualize altitude change
        cv2.arrowedLine(frame, (60,60), (int(60-10*altitude_change),int(60-10*altitude_change)), (0,255,0), 4)
        cv2.arrowedLine(frame, (60,420), (int(60-10*altitude_change),int(420+10*altitude_change)), (0,255,0), 4)
        cv2.arrowedLine(frame, (580,60), (int(580+10*altitude_change),int(60-10*altitude_change)), (0,255,0), 4)
        cv2.arrowedLine(frame, (580,420), (int(580+10*altitude_change),int(420+10*altitude_change)), (0,255,0), 4)

        # Display commands that have been visualized
        cv2.putText(frame, "Translate      X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(x_heading, y_heading, altitude_change), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        cv2.putText(frame, "Rotate            {:.2f}".format(rotation), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Reset command variables every time
        x_heading = 0
        y_heading = 0
        rotation = 0
        altitude_change = 0

        cv2.imshow("Aruco-Time!", frame)

except (Exception, KeyboardInterrupt) as e:
    video.release()
    cv2.destroyAllWindows()
    raise e

video.release()
cv2.destroyAllWindows()
