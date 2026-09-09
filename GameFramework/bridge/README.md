# Godot <-> Python bridge

Godot owns the bodies. Python owns the rules. They talk newline-delimited JSON
over TCP (default `127.0.0.1:8777`).

The point: the simulator runs the *same* `station.py` / `item.py` / `order.py` /
`delivery.py` / `final_station.py` the live game runs, so it can't disagree with
the real game about what an item becomes. Editing `config.py` changes both.

## Run

    cd GameFramework
    python -m bridge.server          # then open bridge/bridge_map.tscn in Godot (F6)

Enter starts a round and restarts after time is up. P1 drives WASD + grab,
P2 the arrow keys + its grab (see project.godot). The table is solid: walk
around it and drop items across the edge onto a station.

## Test without Godot

    python -m bridge.selftest             # rules through the observation interface
    python -m bridge.fake_godot --spawn   # same, over a real socket

## Files

| | |
|---|---|
| `engine.py` | rules driven by observations instead of cameras; `build_stations()` + `client_config()` |
| `protocol.py` | wire format, `LineReader` framing |
| `server.py` | TCP loop |
| `selftest.py` | scripted burger, no sockets |
| `fake_godot.py` | stand-in client speaking the real protocol |

## The interface

Everything the rules need, per tick:

- `tags`: `[[tag_id, x, y], ...]` in `config.py`'s full-frame pixel coords
- `players`: `{zone_key: bool}` — missing zones fail open, as the live game does
- `now`: game seconds. **Godot owns the clock** (via `station.py`'s `clock=`
  hook), so pausing Godot pauses the rules and faster-than-realtime works.

## What does not transfer from config.py

`PLAYER_ZONES` rects live in each gated station's *own camera frame*, not table
space, so Godot can't use them. `bridge_map.gd` derives presence from proximity
to the station rect instead (`PRESENCE_REACH`), assigning each body to the nearest
gated station only -- the gated rects sit side by side, so any reachable padding
would otherwise overlap them and let one body attend all three.

The four station rects exactly tile `TABLE_REGION`, and each one touches a
different outer edge (Cooking left, Slicing top, Assembling bottom, Plating
right), so a solid table with bodies walking around it reaches everything.

Item sprites aren't wired: `ASSET_MAP` names files under `GameFramework/assets/`,
outside `res://`. Items draw as colour-coded rects with a state label for now.
