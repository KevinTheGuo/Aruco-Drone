# Generate a single marker

import cv2
import cv2.aruco
import numpy as np

MARKER_ID = 120

# Generate our specific aruco dict
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
# Draw a marker from this dictionary
drawn_marker = cv2.aruco.drawMarker(aruco_dict, MARKER_ID, 500, None)

cv2.imwrite('markers/aruco_{}.png'.format(MARKER_ID), drawn_marker)


# -----------------------------------------------------------------------------

# Generate a range of markers

MARKER_ID_START = 320
MARKER_ID_END = 329

for i in range(MARKER_ID_START, MARKER_ID_END+1):
    # Generate our specific aruco dict
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    # Draw a marker from this dictionary
    drawn_marker = cv2.aruco.drawMarker(aruco_dict, i, 500, None)

    cv2.imwrite('markers/aruco_{}.png'.format(i), drawn_marker)
