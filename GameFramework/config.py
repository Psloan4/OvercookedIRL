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
FINAL_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/1"
GAME_SECONDS = 300 # in seconds
TICK_MS = 16  # ~60 FPS UI update

# --- Player presence --------------------------------------------------------
# Stations 2a and 2b only scan while a player is standing beside them. Each
# player wears an ArUco tag (4x4_50) on their head; if any of these tags is
# detected inside the station's zone on its dedicated camera, a player counts
# as present and the scan may progress. Stepping out of the zone resets the
# in-progress scan to zero.
PLAYER_TAG_IDS = {11, 12}   # head tags, one per player (kept clear of food ids 0-10)

PLAYER_CAMS = {
    "2a": f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/4",
    "2b": f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/2",
}
PLAYER_ZONES = {
    "2a": dict(x=2, y=0,  w=632, h=421),   # camera 4
    "2b": dict(x=1, y=31, w=636, h=395),   # camera 2
}

# --- Destination colours (UI guidance) --------------------------------------
# Each station has a signature colour. An item is tinted with the colour of the
# station it should go to NEXT, so players can match item -> zone by colour.
STATION_COLORS = {
    "1":  "#ef4444",   # Cooking   (red)
    "2a": "#3b82f6",   # Slicing   (blue)
    "2b": "#22c55e",   # Combining (green)
    "3":  "#facc15",   # Plating   (yellow)
    "4":  "rainbow",   # Delivery  (rainbow ring; no on-table zone to tint)
}

# Item stage -> the station it should be taken to next.
STAGE_DESTINATION = {
    "raw_patty":       "1",   # cook
    "sliced_fries":    "1",   # cook
    "raw_potato":      "2a",  # slice
    "cheese_patty":    "2b",  # combine
    "assembled_patty": "3",   # plate
    "cooked_fries":    "3",   # plate
    "cooked_patty":    "4",   # overcooked -> delivery resets it
    "complete":        "4",   # deliver
}

# Item stage -> colour of its destination station (used to tint item rings).
STAGE_COLORS = {
    stage: STATION_COLORS[dest] for stage, dest in STAGE_DESTINATION.items()
}

STATION_DEFS = [
    #Station 1: Cooking
    dict(x=7, y=110, w=155, h=322, scan_time=12,
         type=tuple(["raw_patty", "cooked_patty", "cheese_patty", "sliced_fries", "cooked_fries"]),
         burn_type=tuple(["cooked_patty", "cheese_patty", "cooked_fries"]),
         show_window=True, covered=None, color=STATION_COLORS["1"]),
    #Station 2a: Slicing
    dict(x=7 + 155, y=110, w=253, h=155, scan_time=6,
         type=tuple(["2a", "grilled_patty", "raw_potato"]),
         burn_type=tuple([]),
         show_window=True, covered=50,
         player_zone="2a", color=STATION_COLORS["2a"]),
    #Station 2b: Combine (Combining functionality not currently implemented, currently just proccesses items)
    dict(x=7 + 155, y=110 + 155, w=253, h=167, scan_time=6,
         type= tuple(["cheese_patty"]),
         burn_type=tuple([]),
         show_window=True, covered=150,
         player_zone="2b", color=STATION_COLORS["2b"]),
    #Station 3: Plating
    dict(x=7 + 155 + 253, y=110, w=196, h=322, scan_time=12,
         type=tuple(["assembled_patty", "cooked_fries"]),
         burn_type=tuple([]),
         show_window=True, covered=None, color=STATION_COLORS["3"]),
]

FINAL_STATION_DEF = dict(
    x=186,
    y=9,
    w=223,
    h=84,
    type="4",
    required_frames=2,
    show_window=True,
)

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
    3: "BURGER",
    4: "BURGER",
    5: "BURGER",
    6: "BURGER",
    7: "FRIES",
    8: "FRIES",
    9: "FRIES",
    10: "FRIES",
    11: "PLAYER",  # head tag, player 1
    12: "PLAYER",  # head tag, player 2
    17: "THE GHOST" #sometimes the camera hallucinates tag 17
}

#Recipies for each food type -- progresses to a random item in the next step when progressed at the appropriate station until complete.
#The burnt state is placed on the end so it never occurs in regular progression, and each food can have it's own burnt sprite.
BURGER = [
    ["raw_patty"], 
    ["cooked_patty", "cheese_patty"], 
    ["assembled_patty"], 
    ["complete"],
    ["burnt_patty"]
]

FRIES = [
    ["raw_potato"],
    ["sliced_fries"],
    ["cooked_fries"], 
    ["complete"],
    ["burnt_fries"]
]

# --- Spatial table view -----------------------------------------------------
# Real-world table size (cm). The on-screen table is locked to this aspect ratio.
TABLE_CM = (117, 62)

# Bounding box of the table within the camera frame, in pixels: (x, y, w, h).
TABLE_REGION = (7, 110, 604, 322)

# Item image per (type, stage): the picture changes as an item progresses.
ASSET_MAP = {
    "BURGER": {
        "raw_patty":        "patty.png",
        "cooked_patty":     "grilled_patty.png",
        "cheese_patty":     "cheesy_patty.png",
        "assembled_patty":  "burger.png",
        "complete":         "burger_complete.png",
        "burnt_patty":      "lord_crandy_bw.png"
    },
    "FRIES": {
        "raw_potato":       "potato.png",
        "sliced_fries":     "raw_fries.png",
        "cooked_fries":     "poop_potato.png",
        "complete":         "finished_fries.png",
        "burnt_fries":      "lord_crandy_bw.png"
    },
}

# hi my name is bryson and i like to eat food and barbecue and i like overcooked even though i dont really play it. but i love overcookedirl even moreeeeeeeee.
