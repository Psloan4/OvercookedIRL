"""Drive the Engine through a scripted burger, with no sockets and no Godot.

Separates "are the rules reachable through the observation interface" from
"does the wire protocol work". Run: python -m bridge.selftest
"""

from bridge.engine import Engine
from config import STATION_DEFS, FINAL_STATION_DEF
import order as order_mod

DT = 1.0 / 60.0

# Region centres, straight from config -- no hardcoded pixels.
CENTRE = {
    stype: (d["x"] + d["w"] / 2, d["y"] + d["h"] / 2)
    for stype, d in zip(["1", "2a", "2b", "3"], STATION_DEFS)
}
CENTRE["4"] = (FINAL_STATION_DEF["x"] + FINAL_STATION_DEF["w"] / 2,
               FINAL_STATION_DEF["y"] + FINAL_STATION_DEF["h"] / 2)
TABLE = (600.0, 400.0)   # a spot inside no station


class Rig:
    def __init__(self):
        self.e = Engine()
        self.e.start()
        self.at = {}          # tag -> (x, y)
        self.now = 0.0
        self.last = {}
        self.delivered = []

    def place(self, tag, where):
        self.at[tag] = CENTRE.get(where, where) if isinstance(where, str) else where

    def run(self, seconds, players=("2a", "2b", "3")):
        present = {z: True for z in players}
        n = int(round(seconds / DT))
        for _ in range(n):
            self.now += DT
            tags = [(t, x, y) for t, (x, y) in self.at.items()]
            self.last = self.e.tick(self.now, tags, present)
            self.delivered += self.last["delivered"]
        return self.last

    def state(self, tag):
        return self.last["items"].get(str(tag), {}).get("state")


def main():
    r = Rig()
    # Force a burger order so delivery has something to match.
    r.e.orders.clear()
    o = order_mod.Order(0.0)
    o.type = "complete_burger"
    r.e.orders.orders.append(o)
    r.e.orders.order_num = 1

    # Tag 0 is a BURGER (raw_patty), tag 2 is CHEESE (cheese_block).
    r.place(0, TABLE); r.place(2, TABLE)
    r.run(0.5)
    print(f"  start                 tag0={r.state(0)}  tag2={r.state(2)}")
    assert r.state(0) == "raw_patty" and r.state(2) == "cheese_block"

    r.place(0, "1")                      # cook, ungated, 12s
    r.run(13)
    print(f"  after 13s at Cooking  tag0={r.state(0)}")
    assert r.state(0) == "cooked_patty", r.state(0)

    r.place(2, "2a")                     # slice, gated, 4s
    r.run(5)
    print(f"  after 5s at Slicing   tag2={r.state(2)}")
    assert r.state(2) == "sliced_cheese", r.state(2)

    # Both to Assembling. cook_one means they scan one at a time (2s each),
    # then the combine fires: patty -> cheese_patty, cheese -> trash.
    r.place(0, "2b"); r.place(2, "2b")
    r.run(6)
    print(f"  after 6s at Assembling tag0={r.state(0)}  tag2={r.state(2)}")
    assert r.state(0) in ("cheese_patty", "assembled_burger"), r.state(0)
    assert r.state(2) == "trash", r.state(2)

    r.place(2, TABLE)                    # get the trash out of the station
    r.run(4)                             # cheese_patty -> assembled_burger
    print(f"  after 4s more          tag0={r.state(0)}")
    assert r.state(0) == "assembled_burger", r.state(0)

    r.place(0, "3")                      # plate, gated, 4s
    r.run(5)
    print(f"  after 5s at Plating   tag0={r.state(0)}")
    assert r.state(0) == "complete_burger", r.state(0)

    pts_before = r.last["points"]
    r.place(0, "4")                      # deliver
    r.delivered.clear()
    r.run(0.5)
    print(f"  after delivery        points={pts_before} -> {r.last['points']}"
          f"  delivered={r.delivered}")
    assert r.last["points"] == pts_before + 10, r.last["points"]
    assert r.state(0) == "raw_patty", f"tag should recycle, got {r.state(0)}"

    print("\n  full burger scored end-to-end through the observation interface")


if __name__ == "__main__":
    main()
