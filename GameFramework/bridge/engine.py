"""The rules of OvercookedIRL, driven by observations instead of cameras.

This is the live game's rule set with the camera taken out. It builds the real
Station objects and reuses item.py / order.py / delivery.py / final_station.py
untouched, so rules cannot drift from the game. What it does NOT own is the
physical world: where items are and where players stand arrive each tick as
observations from whatever simulates bodies -- Godot today, aruco tags in the
live game.

Coordinates are the same full-frame pixel space config.py uses for station
rects, so STATION_DEFS is the single source of truth for geometry too.
"""

from __future__ import annotations

from config import (
    STATION_DEFS,
    FINAL_STATION_DEF,
    GAME_SECONDS,
    GRACE_SECONDS,
    STATION_COLORS,
    STAGE_COLORS,
    PLAYER_ZONES,
    TABLE_REGION,
    TABLE_CM,
    ASSET_MAP,
    IDS,
)
from actions import NAME_TO_STYPE
from final_station import FinalStation
from item import ItemHandler
from order import OrderHandler
from station import Station

# Tags that are real food. Player head tags and the camera's phantom tag 17
# never become items.
FOOD_TAGS = tuple(t for t, kind in IDS.items() if kind not in ("PLAYER", "THE GHOST"))


def build_stations(item_handler, clock, debug=False):
    """The STATION_DEFS -> Station construction, in one place.

    Returns (stations, stype_by_station_id) where stype is the "1"/"2a" key
    that actions.py and the UI index statuses by.
    """
    stations, stype = [], {}
    for d in STATION_DEFS:
        st = Station(
            name=d["name"],
            x=d["x"], y=d["y"], w=d["w"], h=d["h"],
            scan_time=d["scan_time"],
            burn_time=d.get("burn_time", d["scan_time"]),
            type=d["type"],
            burn_type=d["burn_type"],
            combinable=d["combinable"],
            item_handler=item_handler,
            player_zone=d.get("player_zone"),
            cook_one=d.get("cook_one"),
            combine_both=d.get("combine_both"),
            debug=debug,
            clock=clock,
        )
        stations.append(st)
        stype[id(st)] = NAME_TO_STYPE[d["name"]]
    return stations, stype


def client_config() -> dict:
    """The subset of config.py a body simulator needs: geometry and cosmetics.

    Sent once on connect so Godot builds its stations, zones, colours and
    sprites from config.py rather than a hand-kept copy. The semantic rules
    (recipes, combinations, burn paths, order weights) are deliberately absent
    -- the client never interprets them, this engine does.
    """
    return {
        "game_seconds": GAME_SECONDS,
        "grace_seconds": GRACE_SECONDS,
        "table_cm": list(TABLE_CM),
        "table_region": list(TABLE_REGION),
        "stations": [
            {
                "stype": NAME_TO_STYPE[d["name"]],
                "name": d["name"],
                "x": d["x"], "y": d["y"], "w": d["w"], "h": d["h"],
                "player_zone": d.get("player_zone"),
                "color": d.get("color"),
            }
            for d in STATION_DEFS
        ],
        "final_station": {
            k: FINAL_STATION_DEF[k] for k in ("x", "y", "w", "h", "color")
        },
        "player_zones": {k: dict(v) for k, v in PLAYER_ZONES.items()},
        "station_colors": dict(STATION_COLORS),
        "stage_colors": dict(STAGE_COLORS),
        "assets": {k: dict(v) for k, v in ASSET_MAP.items()},
        "food_tags": list(FOOD_TAGS),
        "tag_types": {str(t): IDS[t] for t in FOOD_TAGS},
    }


class Engine:
    """One game's worth of rules. The caller owns the clock and the bodies."""

    def __init__(self, debug=False):
        self.debug = debug
        self.now = 0.0
        self.points = 0
        self.started = False
        clock = lambda: self.now
        self.items = ItemHandler()
        self.orders = OrderHandler(DEBUG=debug, clock=clock)
        self.stations, self._stype = build_stations(self.items, clock, debug)
        # feed_relay is None: we only ever call process(), never _tick().
        self.final = FinalStation(None, self.items, self.orders, FINAL_STATION_DEF)

    def start(self):
        """Begin the round. Items are created lazily, exactly as the camera
        does when it first sees a tag."""
        self.points = 0
        self.now = 0.0
        self.items.clear()
        self.orders.clear()
        for st in self.stations:
            st.reset()
        self.final.reset()
        self.orders.start_game()
        self.started = True

    def tick(self, now: float, tags, players: dict) -> dict:
        """Advance the rules to game time `now`.

        tags:    iterable of (tag_id, x, y) in full-frame pixel coords.
        players: {zone_key: bool}. Missing zones are treated as present, the
                 same fail-open the live game uses for a dead camera.
        """
        if not self.started:
            self.start()
        self.now = float(now)
        tags = [(int(t), float(x), float(y)) for t, x, y in tags]

        statuses, scans, burning, combine_ready = {}, {}, {}, set()
        for st in self.stations:
            ids = [t for (t, x, y) in tags if st.contains(x, y)]
            present = players.get(st.player_zone, True)
            status = st._tick(ids, present)
            statuses[self._stype[id(st)]] = status
            scans.update(status.get("scans", {}))
            burning.update(status.get("burning", {}))
            combine_ready.update(status.get("combine_ready", {}))

        final_status = self.final.process(tags)
        for record in final_status.get("delivered_items", []):
            self.points += record["points"]

        self.orders._tick()

        return {
            "now": self.now,
            "points": self.points,
            "time_left": max(0.0, GAME_SECONDS - self.now),
            "items": {
                str(t): {"state": it.state, "type": it.type}
                for t, it in self.items.items.items()
            },
            "scans": {str(t): p for t, p in scans.items()},
            "burning": {str(t): b for t, b in burning.items() if b},
            "combine_ready": [str(t) for t in combine_ready],
            "stations": {
                k: {
                    "state": s["state"],
                    "player_present": s.get("player_present", True),
                    "ids": [str(i) for i in s.get("ids", [])],
                    "completed": [str(i) for i in s.get("completed", [])],
                }
                for k, s in statuses.items()
            },
            "orders": [{"type": o.type, "time": o.time} for o in self.orders.orders],
            "delivered": final_status.get("delivered_items", []),
            "delivery_scans": {str(t): p for t, p in final_status.get("scans", {}).items()},
        }
