"""Check a game log: re-run the engine on its observations and compare.

A match means the log is complete -- seed + obs rebuild every state -- so it
can be trusted as training data. Also prints a short summary of the round.

Run: python -m bridge.replay logs/<file>.jsonl.gz [more files...]
     python -m bridge.replay --latest
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

from bridge.engine import Engine
from bridge.logger import DEFAULT_DIR, SCHEMA, read, rules_hash


def _norm(obj):
    # Same round trip the logged copy went through.
    return json.loads(json.dumps(obj))


def _first_diff(a, b, path="state"):
    if type(a) is not type(b):
        return f"{path}: {a!r} != {b!r}"
    if isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                return f"{path}.{k}: missing on one side"
            d = _first_diff(a[k], b[k], f"{path}.{k}")
            if d:
                return d
    elif isinstance(a, list):
        if len(a) != len(b):
            return f"{path}: length {len(a)} != {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            d = _first_diff(x, y, f"{path}[{i}]")
            if d:
                return d
    elif a != b:
        return f"{path}: {a!r} != {b!r}"
    return None


def check(path) -> bool:
    lines = read(path)
    header = next(lines, None)
    if not header or header.get("kind") != "header":
        print(f"  {path}: no header")
        return False
    if header.get("schema") != SCHEMA:
        print(f"  warning: schema {header.get('schema')}, replay expects {SCHEMA}")
    if header.get("rules_hash") != rules_hash():
        print("  warning: rules changed since this was recorded "
              f"(git {header.get('git')}); a mismatch may be expected")

    engine = Engine()
    engine.start(header["seed"])

    ticks, end, last = 0, None, {}
    deliveries, grabs, dist = 0, {}, {}
    prev_pos, prev_grab = {}, {}
    for line in lines:
        if line["kind"] == "end":
            end = line
            break
        if line["kind"] != "tick":
            continue
        obs, logged = line["obs"], line["state"]
        got = _norm(engine.tick(obs["now"], obs["tags"], obs["players"]))
        diff = _first_diff(got, logged)
        if diff:
            print(f"  MISMATCH at tick {ticks} (t={obs['now']:.3f}): {diff}")
            return False
        ticks += 1
        last = logged
        deliveries += len(logged["delivered"])

        for name, body in obs.get("bodies", {}).items():
            pos = (body["x"], body["y"])
            if name in prev_pos:
                dist[name] = dist.get(name, 0.0) + math.dist(prev_pos[name], pos)
            prev_pos[name] = pos
        for name, inp in obs.get("inputs", {}).items():
            if inp.get("grab") and not prev_grab.get(name):
                grabs[name] = grabs.get(name, 0) + 1
            prev_grab[name] = inp.get("grab")

    players = ", ".join(f"{k}={v.get('id')}({v.get('kind')})"
                        for k, v in header.get("players", {}).items())
    print(f"  seed {header['seed']}  git {header.get('git')}  {players}")
    print(f"  {ticks} ticks, {last.get('now', 0.0):.1f}s, "
          f"{last.get('points', 0)} points, {deliveries} deliveries"
          + ("" if end else "  (no end line: truncated)"))
    for name in sorted(set(dist) | set(grabs)):
        print(f"  {name}: walked {dist.get(name, 0.0):.0f}px, "
              f"{grabs.get(name, 0)} grab presses")
    if not prev_pos:
        print("  (no bodies/inputs in this log -- not from Godot?)")
    print("  replay OK")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--latest", action="store_true")
    a = ap.parse_args()
    paths = a.paths
    if a.latest or not paths:
        found = sorted(glob.glob(os.path.join(DEFAULT_DIR, "*.jsonl.gz")))
        paths = found[-1:]
    if not paths:
        print("no logs found")
        sys.exit(1)
    ok = True
    for p in paths:
        print(p)
        ok &= check(p)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
