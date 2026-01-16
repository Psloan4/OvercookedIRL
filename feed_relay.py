import cv2

class FeedRelay:
    def __init__(self, feed_fp):
        self.cap = cv2.VideoCapture(feed_fp)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video feed: {feed_fp}")

        # name -> (x, y, w, h)
        self.stations = {}

    def get_image_raw(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to read frame from feed")
        return frame

    def release(self):
        self.cap.release()

    def register_station(self, name, x, y, w, h):
        """
        Register a rectangular station ROI.
        Assumes:
            (x, y) = top-left corner in pixels
            w = width in pixels (x direction)
            h = height in pixels (y direction)
        """
        if not isinstance(name, str) or not name:
            raise ValueError("name must be a non-empty string")

        x, y, w, h = int(x), int(y), int(w), int(h)

        if x < 0 or y < 0 or w <= 0 or h <= 0:
            raise ValueError("x,y must be >= 0 and w,h must be > 0")

        self.stations[name] = (x, y, w, h)

    def get_station_image(self, name):
        if name not in self.stations:
            raise KeyError(f"Station not found: {name}")

        frame = self.get_image_raw()
        H, W = frame.shape[:2]
        x, y, w, h = self.stations[name]

        # Clamp ROI to frame bounds
        x1 = max(0, min(W, x))
        y1 = max(0, min(H, y))
        x2 = max(0, min(W, x + w))
        y2 = max(0, min(H, y + h))

        if x2 <= x1 or y2 <= y1:
            raise ValueError(
                f"Station ROI '{name}' out of bounds: {(x, y, w, h)} for frame {W}x{H}"
            )

        return frame[y1:y2, x1:x2]
