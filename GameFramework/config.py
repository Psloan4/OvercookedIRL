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
GAME_SECONDS = 150 # in seconds
TICK_MS = 16  # ~60 FPS UI update

# --- Player presence --------------------------------------------------------
PLAYER_TAG_IDS = {11, 12}   # head tags, one per player (kept clear of food ids 0-10)

PLAYER_CAMS = {
    "2a": f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/4",
    "2b": f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/2",
    "3":  f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/3",
}
PLAYER_ZONES = {
    "2a": dict(x=2, y=0,  w=632, h=421),   # camera 4
    "2b": dict(x=1, y=31, w=636, h=395),   # camera 2
    "3":  dict(x=25, y=165, w=603, h=311), # camera 3
}

# --- Destination colours (UI guidance) --------------------------------------
# Each station has a signature colour so players can match item -> zone by colour.
STATION_COLORS = {
    "1":  "#ef4444",   # Cooking   (red)
    "2a": "#3b82f5",   # Slicing   (blue)
    "2b": "#22c55e",   # Combining (green)
    "3":  "#facc15",   # Plating   (yellow)
    "4":  "rainbow",   # Delivery  (rainbow ring; no on-table zone to tint)
}

STATION_DEFS = [
    #Station 1: Cooking
    dict(name="Cooking",
         x=7, y=110, w=155, h=322, scan_time=12, burn_time=12,
         type=tuple(["raw_patty", "cooked_patty", "cheese_patty", "sliced_fries", "cooked_fries", "raw_pizza", "cooked_pizza"]),
         burn_type=tuple(["cooked_patty", "cheese_patty", "cooked_fries", "cooked_pizza"]),
         combinable=[],
         color=STATION_COLORS["1"]),
    #Station 2a: Slicing
    dict(name="Slicing",
         x=7 + 155, y=110, w=253, h=155, scan_time=4,
         type=tuple(["2a", "raw_potato", "cheese_block", "cooked_pizza"]),
         burn_type=tuple([]),
         combinable=[],
         player_zone="2a",
         color=STATION_COLORS["2a"],
         cook_one = True),
    #Station 2b: Combine 
    dict(name="Assembling",
         x=7 + 155, y=110 + 155, w=253, h=167, scan_time=2,
         type= tuple(["cheese_patty", "cooked_patty", "sliced_cheese", "cooked_fries", "dough", "sauced_dough"]),
         burn_type=tuple([]),
         combinable=["sliced_cheese","cooked_patty", "cooked_fries", "sauced_dough"],
         player_zone="2b",
         color=STATION_COLORS["2b"],
         cook_one=True,
         combine_both=False),
    #Station 3: Plating
    dict(name="Plating",
         x=7 + 155 + 253, y=110, w=196, h=322, scan_time=4, burn_time = 14,
         type=tuple(["assembled_burger", "cooked_fries", "cone", "vanilla", "chocolate", "strawberry", "cheese_fries"]),
         burn_type=tuple(["vanilla", "chocolate", "strawberry"]),
         combinable=[],
         player_zone="3",
         color=STATION_COLORS["3"],
         cook_one=True),
]

FINAL_STATION_DEF = dict(
    x=186,
    y=9,
    w=223,
    h=84,
    type="4",
    required_frames=2,
    show_window=True,  # pop the delivery station out into its own window
    embed_window=False,  # instead/also show it as a panel inside the main window
)

#Grace seconds determines the amount of time a station can miss a tag before considering it gone
GRACE_SECONDS = 1.0

GRID_PLACEMENT = {
    "1":  (0, 0, 2, 1),  # row, col, colSpan, rowSpan
    "2a": (0, 1, 1, 1),
    "2b": (1, 1, 1, 1),
    "3":  (0, 2, 2, 1),
}

IDS = {
    0: "BURGER",
    1: "BURGER",
    2: "CHEESE",
    3: "CHEESE",
    4: "CHEESE",
    5: "PIZZA",
    6: "CONE",
    7: "CONE",
    8: "PIZZA",
    9: "FRIES",
    10: "FRIES",
    11: "PLAYER",  # head tag, player 1
    12: "PLAYER",  # head tag, player 2
    17: "THE GHOST" #sometimes the camera hallucinates tag 17
}

#Recipies for each food type -- progresses to a random item in the next step until complete.
#The burnt state is placed on the end so it never occurs in regular progression
#Cone uses it's own system of the burnt state to ensure that the color is the same when it melts, seen in item.py
RECIPIES = {
    "BURGER":   [["raw_patty"], ["cooked_patty"], ["assembled_burger"], ["complete_burger"], ["burnt_patty"]],
    "FRIES":    [["raw_potato"], ["sliced_fries"], ["cooked_fries"], ["complete_fries"], ["burnt_fries"]],
    "CHEESE":   [["cheese_block"], ["sliced_cheese"]],
    "BUNS":     [["fresh_bun"], ["sliced_bun"]],
    "CONE":     [["cone"], ["vanilla", "chocolate", "strawberry"]],
    "PIZZA":    [["dough"], ["sauced_dough"]]
}

#List of starting states so final_station knows not to change these
BASE_STATES = [
    "raw_patty",
    "raw_potato",
    "cheese_block",
    "cone",
    "dough",
]

# Combining station will turn both tags in the set to these future states
# COMBINATIONS = {
#     frozenset({"cooked_patty","sliced_cheese"}): [["cheese_patty"],["assembled_burger"],["complete_burger"],["burnt_patty"]],
#     frozenset({"cooked_patty", "sliced_bun"}): [["assembled_burger"],["complete_burger"]],
#     frozenset({"cooked_fries","sliced_cheese"}): [["cheese_fries"],["complete_cheese_fries"]],
#     frozenset({"sauced_dough", "sliced_cheese"}): [["raw_pizza"], ["cooked_pizza"], ["complete_pizza"]],
    
# }

COMBINATIONS = [
    dict(base="cooked_patty", add_on="sliced_cheese", states=[["cheese_patty"],["assembled_burger"],["complete_burger"],["burnt_patty"]]),
    dict(base="cooked_fries", add_on="sliced_cheese", states=[["cheese_fries"],["complete_cheese_fries"]]),
    dict(base="sauced_dough", add_on="sliced_cheese", states=[["raw_pizza"], ["cooked_pizza"], ["complete_pizza"]]),
]

#List of states that are complete and eligible as an order, and the chances of it being chosen
COMPLETE_STATES = {
    "complete_burger": 20, #does not actually have to add up to 100, but doing so makes it easy to read as percentages
    "complete_fries": 20,
    "ice_cream": 20,
    "complete_cheese_fries": 20,
    "complete_pizza": 20
}

#List of ice cream flavors -- used so that all ice cream can be counted as complete and the random flavor is just for fun
ICE_CREAM_FLAVORS = [
    "vanilla", "chocolate", "strawberry"
]

# Which ASSET_MAP item type owns each complete state's icon (for order tickets).
COMPLETE_STATE_ITEM_TYPE = {
    "complete_burger": "BURGER",
    "complete_fries":  "FRIES",
    "complete_cheese_fries": "FRIES",
    "vanilla":         "CONE",
    "chocolate":       "CONE",
    "strawberry":      "CONE",
}

# Item stage -> the station it should be taken to next.
STAGE_DESTINATION = {
    "raw_patty":       "1",   # cook
    "sliced_fries":    "1",   # cook
    "raw_potato":      "2a",  # slice
    "cheese_block":    "2a",  # slice
    "cheese_patty":    "2b",  # combine
    "sliced_cheese":   "2b",  # combine
    "cone":            "3",   # plate
    "assembled_burger":"3",   # plate
    "cooked_fries":    "3",   # plate
    "cheese_fries":    "3",   # plate
    "cooked_patty":    "2b",  # combine (same path as cheese_patty)
    "complete":        "4",   # deliver
}

#Sets the destination of all complete states to the delivery station
for state in COMPLETE_STATES.keys():
    STAGE_DESTINATION[state] = "4"

# Item stage -> colour of its destination station (used to tint item rings).
STAGE_COLORS = {
    stage: STATION_COLORS[dest] for stage, dest in STAGE_DESTINATION.items()
}

STAGE_COLORS["cooked_fries"] = "green_yellow"

# --- Spatial table view -----------------------------------------------------
# Real-world table size (cm). The on-screen table is locked to this aspect ratio.
TABLE_CM = (117, 62)

# Bounding box of the table within the camera frame, in pixels: (x, y, w, h).
TABLE_REGION = (7, 110, 604, 322)

# Item image per (type, stage): the picture changes as an item progresses.
ASSET_MAP = {
    "BURGER": {
        "raw_patty":        "raw_patty.png",
        "cooked_patty":     "cooked_patty.png",
        "cheese_patty":     "cheese_patty.png",
        "assembled_burger": "assembled_burger.png",
        "complete_burger":  "finished_burger.png",
        "burnt_patty":      "burnt_patty.png"
    },
    "FRIES": {
        "raw_potato":       "potato.png",
        "sliced_fries":     "raw_fries.png",
        "cooked_fries":     "cooked_fries.png",
        "complete_fries":   "finished_fries.png",
        "burnt_fries":      "burnt_fries.png",
        "cheese_fries":     "cheese_fries.png",
        "complete_cheese_fries":    "Finished_cheese_fries.png",
    },
    "CHEESE": {
        "cheese_block":     "cheese_block.png",
        "sliced_cheese":    "sliced_cheese.png",
        "cheese_patty":     "cheese_patty.png",
        "assembled_burger": "assembled_burger.png",
        "complete_burger":  "finished_burger.png",
        "burnt_patty":      "burnt_patty.png",
        "cheese_fries":     "cheese_fries.png",
        "complete_cheese_fries":    "Finished_cheese_fries.png",
        "raw_pizza":        "poop_potato.png",
        "cooked_pizza":     "chocolate_melted.png",
        "complete_pizza":   "lord_crandy_bw.png",
    },
    "CONE": {
        "cone":             "cone.png",
        "vanilla":          "vanilla.png",
        "chocolate":        "chocolate.png",
        "strawberry":       "strawberry.png",
        "vanilla_melted":   "vanilla_melted.png",
        "chocolate_melted": "chocolate_melted.png",
        "strawberry_melted":"strawberry_melted.png",
    },
    "PIZZA": {
        "dough":            "vanilla_melted.png",
        "sauced_dough":     "potato.png",
        "raw_pizza":        "poop_potato.png",
        "cooked_pizza":     "chocolate_melted.png",
        "complete_pizza":   "lord_crandy_bw.png",
    },
    "THE GHOST": {
        "inert":            "lord_crandy_bw.png"
    },
    "PLAYER": {
    }
}

# hi my name is bryson and i like to eat food and barbecue and i like overcooked even though i dont really play it. but i love overcookedirl even moreeeeeeeee.
