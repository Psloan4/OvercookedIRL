STATION_CAMERA_DEV = 0
FINAL_CAMERA_DEV = 1
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
    "1":  (0, 0, 2, 1),  # row, col, colSpan, rowSpan
    "2a": (0, 1, 1, 1),
    "2b": (1, 1, 1, 1),
    "3":  (0, 2, 2, 1),
}
