import cv2
import threading


class FeedRelay:
    """
    Low-latency frame source.

    A background thread continuously drains the capture so OpenCV's internal
    receive buffer never backs up. `update_image()` just snapshots the most
    recent decoded frame. On a network MJPEG stream this is the difference
    between "always the freshest frame" and "frames that fall seconds behind
    reality as the game runs".
    """

    def __init__(self, feed_fp):
        self.cap = cv2.VideoCapture(feed_fp)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video feed: {feed_fp}")

        # Ask the backend to keep at most one buffered frame (honored by some
        # backends; the reader thread below handles the rest).
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        self.frame = None
        self.frame_height = None
        self.frame_width = None

        self._latest = None
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        """Continuously grab frames, keeping only the newest one."""
        while self._running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                # Transient network/decode hiccup; keep going.
                continue
            with self._lock:
                self._latest = frame

    def update_image(self):
        """
        Snapshot the latest frame from the reader thread.
        Call this once per tick.
        """
        with self._lock:
            frame = self._latest

        if frame is None:
            # No frame has arrived yet. Don't crash the game over a transient
            # gap; keep the previous frame if we have one.
            if self.frame is None:
                raise RuntimeError("Failed to read frame from feed (no frames yet)")
            return

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

    def get_frame(self):
        return self.frame

    def release(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.cap.release()
