import cv2
import numpy as np


class ArucoTagDetector:
    """
    Detect ArUco/AprilTag-style markers in an input image and return detected tag IDs.

    Usage:
        det = ArucoTagDetector(tag_type="DICT_ARUCO_ORIGINAL", resize_width=1000)
        ids = det.detect_ids(frame)          # -> list[int]
        best = det.detect_first_id(frame)    # -> int | None
        corners, ids = det.detect(frame)     # -> (corners, ids_list)
    """

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
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }

    def __init__(self, tag_type: str = "DICT_ARUCO_ORIGINAL", resize_width: int | None = None):
        if tag_type not in self.ARUCO_DICT:
            raise ValueError(f"Unsupported tag_type '{tag_type}'. Options: {list(self.ARUCO_DICT.keys())}")

        self.tag_type = tag_type
        self.resize_width = resize_width

        # Dictionary
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[tag_type])
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[tag_type])

        # Parameters + detector (supports old & new OpenCV)
        self.detector = None
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def _resize_keep_aspect(self, image: np.ndarray, width: int) -> np.ndarray:
        h, w = image.shape[:2]
        if w == 0:
            return image
        scale = width / float(w)
        new_h = int(h * scale)
        return cv2.resize(image, (width, new_h), interpolation=cv2.INTER_AREA)

    def detect(self, image_bgr: np.ndarray):
        """
        Returns:
            corners: list of detected marker corners (OpenCV format)
            ids: list[int] (empty if none detected)
        """
        if image_bgr is None:
            raise ValueError("image_bgr is None")
        if not isinstance(image_bgr, np.ndarray):
            raise TypeError("image_bgr must be a numpy ndarray (BGR image)")

        frame = image_bgr
        if self.resize_width is not None:
            frame = self._resize_keep_aspect(frame, self.resize_width)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is None:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        else:
            corners, ids, _rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return corners, []

        # ids shape is (N, 1) -> flatten to python list
        ids_list = [int(x) for x in ids.flatten().tolist()]
        return corners, ids_list

    def detect_ids(self, image_bgr: np.ndarray) -> list[int]:
        """Convenience: return just the list of IDs."""
        _corners, ids = self.detect(image_bgr)
        return ids

    def detect_first_id(self, image_bgr: np.ndarray) -> int | None:
        """Convenience: return first detected ID (or None)."""
        ids = self.detect_ids(image_bgr)
        return ids[0] if ids else None

    def annotate(self, image_bgr: np.ndarray, draw_ids: bool = True) -> np.ndarray:
        """
        Returns a copy of the image with detected markers drawn.
        Note: If resize_width is set, annotation occurs on the resized frame.
        """
        frame = image_bgr.copy()
        if self.resize_width is not None:
            frame = self._resize_keep_aspect(frame, self.resize_width)

        corners, ids = self.detect(frame)

        out = frame.copy()
        if ids:
            # Need ids as (N,1) numpy array for drawDetectedMarkers in some versions
            ids_np = np.array(ids, dtype=np.int32).reshape(-1, 1) if draw_ids else None
            cv2.aruco.drawDetectedMarkers(out, corners, ids_np)
        return out
