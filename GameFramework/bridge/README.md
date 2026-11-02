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

`--players 1` plays solo: the handshake tells Godot to spawn p1 only, and the
logs list p1 alone. The rules are unchanged, so a lone body still has to stand
at a gated station for its scan to finish.

    python -m bridge.server --players 1 --p1 alice --note "solo"

## Bots

`--p1-bot MODULE:CLASS` (or `--p2-bot`) hands that body to a Python policy.
Each tick the server calls `policy.act(obs, state)` with the same obs/state
pair a log line stores, and sends the answer back on the state reply as
`commands`; Godot drives the body from it instead of the keyboard. The
interface is in `bots.py`, with `Idle` and `Seek` (fetches the nearest item, a
loop test rather than a cook) as examples. A bot's moves land in the log's
`inputs` like a human's, and its header entry is `kind: bot` with the policy.

    python -m bridge.server --players 1 --p1-bot bridge.bots:Seek
    python -m bridge.server --p1 alice --p2-bot mymodels.bc:Policy   # human + bot

Rounds still start on Enter.

## Game logs

Every round is saved to `GameFramework/logs/<time>_seed<N>.jsonl.gz`, one
JSON line per tick: what Godot sent (`obs`: tags, presence, plus each body's
pose and the inputs it acted on that frame) and what the engine answered
(`state`). A header records git commit, rules hash, seed and players.

    python -m bridge.server --p1 alice --p2 bob --note "pairing: human+human"
    python -m bridge.replay --latest      # re-runs the engine, must match

The seed drives every random draw (orders, item variants), so seed + obs
rebuild each state exactly; `replay` checks that. `--no-log` turns it off,
`--seed N` fixes the seed.

## Training data

    python -m bridge.dataset              # logs/*.jsonl.gz -> logs/dataset.npz
    python -m bridge.dataset --every 6 --include-bots -o mix.npz a.jsonl.gz b.jsonl.gz

One example per body per decision, from that body's point of view: `me`, the
`partner` (zeroed with `partner_present` 0 in a solo round), stations, every
food item (position, position relative to me, state, who holds it, scan
progress) and the open orders. So solo and two-player rounds, p1 and p2, go
into one set and one model can drive either body. `X` holds the features
(named in `features`), `A` the throttle/turn/grab taken, and `episode`,
`player`, `tick` point back at the log line.

`--every 6` (the default) keeps 10 decisions a second; a grab anywhere in the
window counts. Bot-driven bodies are left out unless `--include-bots`; logs
recorded under different rules are flagged, or dropped with `--strict-rules`.

A learned policy builds its input with `dataset.featurize(obs, state, me)`
inside `act()`, the same function the dataset used.

## Test without Godot

    python -m bridge.selftest             # rules through the observation interface
    python -m bridge.fake_godot --spawn   # same, over a real socket

## Files

| | |
|---|---|
| `engine.py` | rules driven by observations instead of cameras; `build_stations()` + `client_config()` |
| `protocol.py` | wire format, `LineReader` framing |
| `server.py` | TCP loop; runs bot policies |
| `bots.py` | policy interface and example bots |
| `selftest.py` | scripted burger, no sockets |
| `fake_godot.py` | stand-in client speaking the real protocol |
| `logger.py` | per-round gzipped JSONL game logs |
| `replay.py` | re-runs a log through the engine and checks it matches |
| `dataset.py` | logs -> per-player training examples; `featurize()` for bots |

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

`FINAL_STATION_DEF`'s rect had the same problem -- it is a crop in the *delivery*
camera's frame, not table space. So the board's real size lives in
`FINAL_STATION_CM` (2.5ft x 1ft, five sections of two blocks) and
`FINAL_STATION_TABLE_RECT` derives its place in table space from that: stood on
its long edge off the table's left edge, alongside the Cooking station, as the
live game's UI lays it out. The bridge engine and Godot use the derived rect;
the live game still uses the crop.

Bodies are sized off the Cooking station (`BODY_FRACTION`, a third of its short
side) and `WALK_MARGIN` sets the lane around the table. Both grow the arena past
the window, so `bridge_map.gd` zooms the camera to fit and undoes that zoom on
the HUD.

The four station rects exactly tile `TABLE_REGION`, and each one touches a
different outer edge (Cooking left, Slicing top, Assembling bottom, Plating
right), so a solid table with bodies walking around it reaches everything.

Item art is the live game's own PNGs. `ASSET_MAP` names files under
`GameFramework/assets/`, outside `res://`, so importing them into the Godot
project would mean a second copy that drifts. Instead `client_config()` sends
`assets_dir` as an absolute path and `bridge_item.gd` loads each file at
runtime, cached per path. A state with no entry in `ASSET_MAP` falls back to the
colour-coded rect.

Under each item is the live game's scan bar -- same colours as
`QProgressBar#ScanBar` in `style.py`, and a burn drains it instead of filling
it.
