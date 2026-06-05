STATION_CAMERA_DEV = 0
FINAL_CAMERA_DEV = 1
GAME_SECONDS = 300 #Set to 5 minutes for testing purposes
TICK_MS = 16  # ~60 FPS UI update

STATION_DEFS = [
    dict(x=7, y=79, w=201, h=331, scan_time=12, type="1",  show_window=True, covered=None),
    dict(x=7 + 201, y=79, w=263, h=178, scan_time=6, type="2a", show_window=True, covered=50),
    dict(x=7 + 201, y=79 + 178, w=263, h=153, scan_time=6, type="2b", show_window=True, covered=150),
    dict(x=7 + 201 + 263, y=79, w=164, h=333, scan_time=12, type="3",  show_window=True, covered=None),
]

GRID_PLACEMENT = {
    "1":  (0, 0, 1, 2),  # row, col, colSpan, rowSpan
    "2a": (1, 0, 1, 1),
    "2b": (1, 1, 1, 1),
    "3":  (2, 0, 1, 2),
}
