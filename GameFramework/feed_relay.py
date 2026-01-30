import cv2


class FeedRelay:
    def __init__(self, feed_fp):
        self.cap = cv2.VideoCapture(feed_fp)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video feed: {feed_fp}")

        self.frame = None
        self.frame_height = None
        self.frame_width = None

    def update_image(self):
        """
        Grab the latest frame from the video feed.
        Call this once per tick.
        """
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to read frame from feed")

        self.frame = frame
        self.frame_height, self.frame_width = frame.shape[:2]

    def get_sub_section(self, x, y, w, h):
        """
        Return a cropped subsection of the most recent frame.

        (x, y) = top-left corner in pixels
        w = width in pixels
        h = height in pixels
        """
        if self.frame is None:
            raise RuntimeError("No frame available. Call update_image() first.")

        x, y, w, h = int(x), int(y), int(w), int(h)

        if x < 0 or y < 0 or w <= 0 or h <= 0:
            raise ValueError("x,y must be >= 0 and w,h must be > 0")

        # Clamp to frame bounds
        x1 = max(0, min(self.frame_width, x))
        y1 = max(0, min(self.frame_height, y))
        x2 = max(0, min(self.frame_width, x + w))
        y2 = max(0, min(self.frame_height, y + h))

        if x2 <= x1 or y2 <= y1:
            raise ValueError(
                f"ROI out of bounds: {(x, y, w, h)} "
                f"for frame {self.frame_width}x{self.frame_height}"
            )

        return self.frame[y1:y2, x1:x2]

    def release(self):
        self.cap.release()
