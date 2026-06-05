"""
Lightweight MJPEG-over-HTTP server for sharing camera feeds across the LAN.

Usage (from the capture script, e.g. multiplecamreader.py):

    from stream_server import start_server, publish

    start_server(port=8080)          # call once, after cameras are opened
    ...
    publish("cam/0", annotated_frame)   # call each loop, per feed
    publish("dashboard", dashboard)

On another computer, open any feed by URL:

    http://<capture-pc-ip>:8080/            <- index page listing live feeds
    http://<capture-pc-ip>:8080/cam/0       <- single camera
    http://<capture-pc-ip>:8080/dashboard   <- combined view

Because each endpoint is a standard MJPEG stream, it works in a browser and
in OpenCV directly:  cv2.VideoCapture("http://<ip>:8080/cam/0")
"""

import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

# name -> latest frame (numpy array). Encoding happens per-client in the
# handler, so feeds with no viewers cost nothing.
_frames = {}
_lock = threading.Lock()

_stream_fps = 30
_jpeg_quality = 70


def publish(name, frame):
    """Make `frame` the latest image for feed `name`. Call once per loop."""
    with _lock:
        _frames[name] = frame


def _get_frame(name):
    with _lock:
        return _frames.get(name)


def _feed_names():
    with _lock:
        return sorted(_frames.keys())


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/")

        if path in ("", "/"):
            self._serve_index()
            return

        name = path.lstrip("/")

        # Wait briefly for the first frame in case a viewer connects before the
        # capture loop has produced anything yet.
        deadline = time.monotonic() + 2.0
        while _get_frame(name) is None and time.monotonic() < deadline:
            time.sleep(0.05)

        if _get_frame(name) is None:
            self.send_error(404, f"Unknown or not-yet-available feed: {name}")
            return

        self._serve_stream(name)

    def _serve_index(self):
        names = _feed_names()
        items = "".join(
            f'<div style="display:inline-block;margin:8px;text-align:center">'
            f'<div>{n}</div><img src="/{n}" width="480"></div>'
            for n in names
        )
        body = (
            "<html><head><title>Camera streams</title></head>"
            "<body style='font-family:sans-serif;background:#111;color:#eee'>"
            "<h2>Live camera streams</h2>"
            f"{items or '<p>No feeds published yet.</p>'}"
            "</body></html>"
        ).encode()

        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self, name):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.end_headers()

        interval = 1.0 / _stream_fps
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, _jpeg_quality]

        try:
            while True:
                frame = _get_frame(name)
                if frame is not None:
                    ok, jpg = cv2.imencode(".jpg", frame, encode_params)
                    if ok:
                        data = jpg.tobytes()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(data)}\r\n\r\n".encode()
                        )
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError):
            # Viewer disconnected; just end this request quietly.
            pass

    def log_message(self, *args):
        # Silence per-request console spam.
        pass


def start_server(host="0.0.0.0", port=8080, fps=30, quality=70):
    """
    Start the MJPEG server in a background daemon thread and return it.

    host="0.0.0.0" listens on all interfaces so other machines on the LAN can
    connect. fps caps how fast frames are pushed per client; quality is the
    JPEG quality (1-100).
    """
    global _stream_fps, _jpeg_quality
    _stream_fps = fps
    _jpeg_quality = quality

    server = ThreadingHTTPServer((host, port), _Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
