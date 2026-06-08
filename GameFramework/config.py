# --- Camera sources ---------------------------------------------------------
# On the camera PC (PC-B) the cameras are local indices, but on the dev PC
# (PC-A) there are no cameras attached, so we read them over the network from
# the MJPEG stream that multiplecamreader.py --stream serves.
#
# Set CAMERA_HOST to PC-B's LAN IP (it's printed when you start the stream).
# To run locally on PC-B instead, set STATION_CAMERA_DEV = 0 / FINAL_CAMERA_DEV = 1.
CAMERA_HOST = "10.55.11.161"   # <-- change to PC-B's IP
CAMERA_PORT = 8080

STATION_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/4"
FINAL_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/4"
GAME_SECONDS = 300 #Set to 5 minutes for testing purposes
TICK_MS = 16  # ~60 FPS UI update

switch = True
STATION_DEFS = [
    dict(x=7, y=110, w=155, h=322, scan_time=12, type="1",  show_window=switch, covered=None),
    dict(x=7 + 155, y=110, w=253, h=155, scan_time=6, type="2a", show_window=switch, covered=50),
    dict(x=7 + 155, y=110 + 155, w=253, h=167, scan_time=6, type="2b", show_window=switch, covered=150),
    dict(x=7 + 155 + 253, y=110, w=196, h=322, scan_time=12, type="3",  show_window=switch, covered=None),
]

GRID_PLACEMENT = {
    "1":  (0, 0, 1, 2),  # row, col, colSpan, rowSpan
    "2a": (1, 0, 1, 1),
    "2b": (1, 1, 1, 1),
    "3":  (2, 0, 1, 2),
}
