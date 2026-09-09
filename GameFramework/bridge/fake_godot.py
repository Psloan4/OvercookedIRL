"""A stand-in for the Godot client, speaking the real wire protocol.

Same messages, same framing, same order as bridge.gd -- so it exercises the
socket path end to end. Lets the bridge be tested (and regression-tested in
CI) without launching the engine.

Run: python -m bridge.fake_godot          (needs bridge.server running)
     python -m bridge.fake_godot --spawn  (starts its own server)
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time

from bridge.protocol import HOST, PORT, LineReader, encode
from config import STATION_DEFS, FINAL_STATION_DEF

DT = 1.0 / 60.0

CENTRE = {
    stype: (d["x"] + d["w"] / 2, d["y"] + d["h"] / 2)
    for stype, d in zip(["1", "2a", "2b", "3"], STATION_DEFS)
}
CENTRE["4"] = (FINAL_STATION_DEF["x"] + FINAL_STATION_DEF["w"] / 2,
               FINAL_STATION_DEF["y"] + FINAL_STATION_DEF["h"] / 2)
TABLE = (600.0, 400.0)


class Client:
    def __init__(self, host=HOST, port=PORT):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.reader = LineReader()
        self.pending = []
        self.at = {}
        self.now = 0.0
        self.state = {}
        self.delivered = []
        self.config = None
        self.config = self._await("config")

    def _await(self, kind):
        while True:
            for msg in self.pending:
                if msg.get("type") == kind:
                    self.pending.remove(msg)
                    return msg
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("server closed")
            self.pending += list(self.reader.feed(chunk))

    def place(self, tag, where):
        self.at[tag] = CENTRE[where] if isinstance(where, str) else where

    def run(self, seconds, players=("2a", "2b", "3")):
        present = {z: True for z in players}
        for _ in range(int(round(seconds / DT))):
            self.now += DT
            self.sock.sendall(encode({
                "type": "observe",
                "now": self.now,
                "tags": [[t, x, y] for t, (x, y) in self.at.items()],
                "players": present,
            }))
            self.state = self._await("state")
            self.delivered += self.state["delivered"]
        return self.state

    def item(self, tag):
        return self.state["items"].get(str(tag), {}).get("state")

    def close(self):
        self.sock.close()


def main(spawn=False, port=PORT):
    proc = None
    if spawn:
        proc = subprocess.Popen([sys.executable, "-m", "bridge.server",
                                 "--port", str(port)],
                                stdout=subprocess.DEVNULL)
        time.sleep(1.2)
    try:
        c = Client(port=port)
        print(f"  handshake: {len(c.config['stations'])} stations, "
              f"{len(c.config['food_tags'])} food tags, "
              f"table {c.config['table_cm'][0]}x{c.config['table_cm'][1]}cm")

        c.place(0, TABLE)
        c.run(0.5)
        print(f"  tag0 initial          {c.item(0)}")
        assert c.item(0) == "raw_patty", c.item(0)

        c.place(0, "1")
        c.run(13)
        print(f"  after Cooking         {c.item(0)}")
        assert c.item(0) == "cooked_patty", c.item(0)

        # Gating: hold at Plating with NO player in zone 3 -> no progress.
        c.place(0, "3")
        c.run(6, players=())
        print(f"  6s at Plating, nobody there  {c.item(0)}  (must not advance)")
        assert c.item(0) == "cooked_patty", c.item(0)

        # Burning: Cooking burns cooked_patty after burn_time 12s.
        c.place(0, "1")
        c.run(13)
        print(f"  13s more at Cooking   {c.item(0)}  (burn path)")
        assert c.item(0) == "burnt_patty", c.item(0)

        # Delivery doubles as the bin: burnt item is removed, scores nothing.
        pts = c.state["points"]
        c.place(0, "4")
        c.run(0.5)
        print(f"  binned burnt patty    points {pts} -> {c.state['points']}, "
              f"tag recycled to {c.item(0)}")
        assert c.state["points"] == pts
        assert c.item(0) == "raw_patty", c.item(0)

        print(f"  orders open: {[o['type'] for o in c.state['orders']]}")
        print("\n  protocol + rules verified over a real socket")
        c.close()
    finally:
        if proc:
            proc.terminate()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn", action="store_true")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args()
    main(a.spawn, a.port)
