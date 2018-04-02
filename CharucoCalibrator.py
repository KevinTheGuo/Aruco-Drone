import cv2
import cv2.aruco

# Try to grab video input
video = cv2.VideoCapture(0)
if not video.isOpened():
    print('Could not open video!')
    exit()

# Generate our specific aruco dict and draw a marker with it
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
drawn_marker = cv2.aruco.drawMarker(aruco_dict, 22, 500, None)

# Generate and draw a Charuco Board from this dictionary
charuco_board = cv2.aruco.CharucoBoard_create(4, 5, 0.04, 0.03, aruco_dict)
drawn_board = charuco_board.draw((400,500))
cv2.imwrite('charuco.png',drawn_board)

# Generate list of corners and IDs for calibration
allCorners = []
allIds = []

# Variable that tracks how many frames we've taken for calibration
total_calib_frames = 0

try:
    while(True):
        # Grab a frame
        read_success, frame = video.read()
        if not read_success:
            print('Cannot read video file!')
            exit()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(frame, aruco_dict)

        # Each time the user presses enter, save a snapshot of the camera output for calibration.
        # Otherwise, if the user presses esc, exit early.
        key = cv2.waitKey(1) & 0xff
        if key == 27:   # If we want to exit and calculate the calibration matrix
            break
        elif key == 13:  # If we want to take a snapshot
            if ids is not None:    # Make sure there are detected markers
                numCorners, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, charuco_board)
                if charucoCorners is not None and charucoIds is not None and len(charucoCorners)>3:  # Make sure markers are good
                    allCorners.append(charucoCorners)
                    allIds.append(charucoIds)
                    total_calib_frames += 1


        output_frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if total_calib_frames == 0:
            cv2.putText(output_frame, "Welcome to the Charuco Calibrator!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(output_frame, "Press 'k' to take a calibration screenshot", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(output_frame, "When you're done, press enter to calculate the calibration matrix.", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            cv2.putText(output_frame, "{} calibration frames taken".format(total_calib_frames), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow("Aruco-Time!", output_frame)

except (Exception, KeyboardInterrupt) as e:
    video.release()
    cv2.destroyAllWindows()
    raise e

video.release()
cv2.destroyAllWindows()

imgsize = frame.shape
#imgsize = frame.shape[::-1][1:3]
retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, charuco_board, imgsize, None, None)
print("Projection Error: \n{}".format(retval))
print("\nCameraMatrix: \n{}".format(repr(cameraMatrix)))
print("\nDistCoeffs: \n{}".format(repr(distCoeffs)))
#print("\nrvecs: \n{}".format(rvecs))
#print("\ntvecs: \n{}".format(tvecs))
