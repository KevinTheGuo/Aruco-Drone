import cv2
import cv2.aruco
import numpy as np

# Add our calculated camera calibration variables
camera_matrix = np.array( [[ 612.28318156,    0.        ,  377.95149893],
                           [   0.        ,  603.72201353,  273.9850609 ],
                           [   0.        ,    0.        ,    1.        ]])

dist_coeffs = np.array(   [[ 0.45710893, -0.68904653,  0.05444916, -0.01899903, -1.79491808]])

# Generate our specific aruco dict
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
# Draw a marker from this dictionary
drawn_marker = cv2.aruco.drawMarker(aruco_dict, 50, 500, None)

# Generate an Aruco Board from this dictionary
aruco_board = cv2.aruco.GridBoard_create(4, 5, 0.04, 0.01, aruco_dict, 0)
# Draw this board
drawn_board = cv2.aruco.drawPlanarBoard(aruco_board, (400,500), 1, 1)

# Try to grab video input
video = cv2.VideoCapture(0)
# Exit if video not opened.
if not video.isOpened():
    print('Could not open video!')
    exit()

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

        # Detect and display markers and their poses in our video frame
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(frame, aruco_dict)
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        #frame = cv2.aruco.drawDetectedMarkers(frame, rejected_corners)

        if (len(corners) > 0):
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], 1, camera_matrix, dist_coeffs)
            frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 1)

            for i in range(0,len(rvecs)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]     # 0 is a placeholder? try multiple markers

                # Get pitch/roll/yaw from rvecs
                rotation_matrix, jacobian = cv2.Rodrigues(rvec)
                euler = rotationMatrixToEulerAngles(rotation_matrix)

                cv2.putText(frame, "Translate X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(tvec[0],tvec[1],tvec[2]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(frame, "   Rotate X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(euler[0], euler[1], euler[2]), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow("Aruco-Time!", frame)

except (Exception, KeyboardInterrupt) as e:
    video.release()
    cv2.destroyAllWindows()
    raise e

video.release()
cv2.destroyAllWindows()
