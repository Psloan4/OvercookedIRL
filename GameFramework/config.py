# --- Camera sources ---------------------------------------------------------
# On the camera PC (PC-B) the cameras are local indices, but on the dev PC
# (PC-A) there are no cameras attached, so we read them over the network from
# the MJPEG stream that multiplecamreader.py --stream serves.
#
# Set CAMERA_HOST to PC-B's LAN IP (it's printed when you start the stream).
# To run locally on PC-B instead, set STATION_CAMERA_DEV = 0 / FINAL_CAMERA_DEV = 1.
CAMERA_HOST = "10.55.11.161"   # <-- change to PC-B's IP
CAMERA_PORT = 8080

STATION_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/0"
FINAL_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/4"
GAME_SECONDS = 300 # in seconds
TICK_MS = 16  # ~60 FPS UI update

STATION_DEFS = [
    dict(x=7, y=110, w=155, h=322, scan_time=12, type="1",  show_window=True, covered=None),
    dict(x=7 + 155, y=110, w=253, h=155, scan_time=6, type="2a", show_window=True, covered=50),
    dict(x=7 + 155, y=110 + 155, w=253, h=167, scan_time=6, type="2b", show_window=True, covered=150),
    dict(x=7 + 155 + 253, y=110, w=196, h=322, scan_time=12, type="3",  show_window=True, covered=None),
]

GRID_PLACEMENT = {
    "1":  (0, 0, 2, 1),  # row, col, colSpan, rowSpan
    "2a": (0, 1, 1, 1),
    "2b": (1, 1, 1, 1),
    "3":  (0, 2, 2, 1),
}

IDS = { #Currently supports burgers and fries -- soon to add Player
    0: "BURGER",
    1: "BURGER",
    2: "BURGER",
    3: "FRIES",
    4: "FRIES",
}

BURGER = [
    ["1"], #Raw Patty
    ["2a", "2b"], #I dunno add cheese or smth
    ["3"], #cook again for some reason
    ["complete"]
]

FRIES = [
    ["2a","2b"], #Cut into slices
    ["1","3"], #cook
    ["complete"]
]

# --- Spatial table view -----------------------------------------------------
# Real-world table size (cm). The on-screen table is locked to this aspect ratio.
TABLE_CM = (117, 62)

# Bounding box of the table within the camera frame, in pixels: (x, y, w, h).
TABLE_REGION = (7, 110, 604, 322)

# Item image per (type, stage): the picture changes as an item progresses.
ASSET_MAP = {
    "BURGER": {
        "1":        "patty.png",          # raw patty
        "2a":       "grilled_patty.png",
        "2b":       "cheesy_patty.png",
        "3":        "burger.png",
        "complete": "burger_complete.png",
    },
    "FRIES": {
        "2a":       "fries_2a.png",
        "2b":       "fries_2b.png",
        "1":        "fries_1.png",
        "3":        "fries_3.png",
        "complete": "fries_complete.png",
    },
}

# hi my name is bryson and i like to eat food and barbecue and i like overcooked even though i dont really play it. but i love overcookedirl even moreeeeeeeee.