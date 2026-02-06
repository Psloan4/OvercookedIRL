import cv2
import numpy as np


class ArucoTagDetector:
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

        # Parameters (supports old & new OpenCV)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # --------------------------------------------------
        # LOWER EFFECTIVE "CONFIDENCE" (higher recall)
        # --------------------------------------------------

        # Allow smaller / farther tags
        self.aruco_params.minMarkerPerimeterRate = 0.003
        self.aruco_params.maxMarkerPerimeterRate = 8.0

        # Accept more distorted squares
        self.aruco_params.polygonalApproxAccuracyRate = 0.08

        # Allow more bit errors during decoding (main confidence-like knob)
        self.aruco_params.errorCorrectionRate = 0.9

        # Corner refinement often rejects marginal detections
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE

        # More aggressive adaptive thresholding
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 11
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.adaptiveThreshConstant = 5


        # --------------------------------------------------

        # Detector (new OpenCV). For old OpenCV we will call cv2.aruco.detectMarkers directly.
        self.detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
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
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        if self.detector is None:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )
        else:
            corners, ids, _rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return corners, []

        ids_list = [int(x) for x in ids.flatten().tolist()]
        return corners, ids_list

    def detect_ids(self, image_bgr: np.ndarray) -> list[int]:
        _corners, ids = self.detect(image_bgr)
        return ids

    def detect_first_id(self, image_bgr: np.ndarray) -> int | None:
        ids = self.detect_ids(image_bgr)
        return ids[0] if ids else None

    def annotate(self, image_bgr: np.ndarray, draw_ids: bool = True) -> np.ndarray:
        frame = image_bgr.copy()
        if self.resize_width is not None:
            frame = self._resize_keep_aspect(frame, self.resize_width)

        corners, ids = self.detect(frame)

        out = frame.copy()
        if ids:
            ids_np = np.array(ids, dtype=np.int32).reshape(-1, 1) if draw_ids else None
            cv2.aruco.drawDetectedMarkers(out, corners, ids_np)
        return out
