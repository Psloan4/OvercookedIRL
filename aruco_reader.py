# =========================
# Imports
# =========================
import argparse
import time
import sys

import cv2
import imutils
import numpy as np

# =========================
# Argument parsing
# =========================
ap = argparse.ArgumentParser()
ap.add_argument(
    "-t", "--type",
    type=str,
    default="DICT_ARUCO_ORIGINAL",
    help="type of ArUco tag to detect"
)
args = vars(ap.parse_args())

# =========================
# ArUco dictionaries
# =========================
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

if ARUCO_DICT.get(args["type"], None) is None:
    print(f"[ERROR] ArUCo tag '{args['type']}' is not supported")
    sys.exit(1)

print(f"[INFO] Detecting '{args['type']}' tags...")

# =========================
# ArUco detector setup
# =========================
try:
    arucoDict = cv2.aruco.getPredefinedDictionary(
        ARUCO_DICT[args["type"]]
    )
except AttributeError:
    arucoDict = cv2.aruco.Dictionary_get(
        ARUCO_DICT[args["type"]]
    )

try:
    arucoParams = cv2.aruco.DetectorParameters_create()
    detector = None
except AttributeError:
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# =========================
# Camera initialization
# =========================
print("[INFO] Opening camera...")

CAMERA_INDEX = 6 # change to 6 if needed

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("[ERROR] Could not open camera")
    sys.exit(1)

time.sleep(1.0)

# =========================
# Main loop
# =========================
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Resize for performance (optional)
    frame = imutils.resize(frame, width=1000)

    # Keep color frame for display
    display = frame.copy()

    # Grayscale ONLY for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers (supports old & new OpenCV)
    if detector is None:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, arucoDict, parameters=arucoParams
        )
    else:
        corners, ids, rejected = detector.detectMarkers(gray)

    # Draw detections
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(display, corners, ids)

    cv2.imshow("ArUco Detection (Color)", display)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# =========================
# Cleanup
# =========================
cap.release()
cv2.destroyAllWindows()
