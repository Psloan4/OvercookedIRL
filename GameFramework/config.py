CAMERA_DEV = "/dev/video6"
GAME_SECONDS = 120
TICK_MS = 16  # ~60 FPS UI update

STATION_DEFS = [
    dict(x=7, y=79, w=201, h=331, scan_time=2, type="1",  show_window=True),
    dict(x=7 + 201, y=79, w=263, h=178, scan_time=2, type="2a", show_window=False),
    dict(x=7 + 201, y=79 + 178, w=263, h=153, scan_time=2, type="2b", show_window=False),
    dict(x=7 + 201 + 263, y=79, w=164, h=333, scan_time=2, type="3",  show_window=True),
]

GRID_PLACEMENT = {
    "1":  (0, 0, 2, 1),  # row, col, rowSpan, colSpan
    "2a": (0, 1, 1, 1),
    "2b": (1, 1, 1, 1),
    "3":  (0, 2, 2, 1),
}
