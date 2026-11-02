"""Game logs -> imitation-learning examples, one per body per decision.

    python -m bridge.dataset                         # logs/*.jsonl.gz -> logs/dataset.npz
    python -m bridge.dataset logs/a.jsonl.gz -o a.npz --every 6

Each example is one body's point of view: featurize(obs, state, me) of the
obs/state pair a log line stores, and the throttle/turn/grab that body acted
on. A bot's act() gets that same pair, so a learned policy calls featurize()
at play time and sees exactly what it was trained on.

Point of view, not player name: "me" is the body acting, "partner" the other
one. A solo round gives one example per decision with the partner block zeroed
and partner_present 0; a two-player round gives two, one from each side. So
solo and paired rounds, p1 and p2, share one training set, and one model can
drive either body.

Positions are centred on the table and scaled by SCALE. Items and the partner
also appear in my frame (fwd = ahead of me, right = to my right), which is
what steering needs and doesn't care which lane a body started in.

--every N keeps every Nth tick (60 Hz logs; 6 -> 10 decisions/s). Throttle and
turn are the window's first tick; grab is whether it was pressed anywhere in
the window, so no press is dropped. A policy trained this way should hold its
throttle/turn for N ticks and send each grab once.
"""

from __future__ import annotations

import argparse
import glob
import math
import os

import numpy as np

from bridge import logger
from bridge.engine import FOOD_TAGS
from config import (COMBINATIONS, COMPLETE_STATES, GAME_SECONDS,
                    ICE_CREAM_FLAVORS, PLAYER_ZONES, RECIPIES, STATION_DEFS,
                    TABLE_REGION)
from actions import NAME_TO_STYPE

_tx, _ty, _tw, _th = TABLE_REGION
CENTRE = (_tx + _tw / 2.0, _ty + _th / 2.0)
SCALE = 300.0

ZONES = tuple(PLAYER_ZONES)                       # gated stations, by zone key
STYPES = tuple(NAME_TO_STYPE[d["name"]] for d in STATION_DEFS)
ORDER_TYPES = tuple(COMPLETE_STATES)
MAX_ORDERS = 5


def _states():
    """Every item state the rules can produce, from config.py, in a fixed order."""
    out = ["inert"]
    for steps in RECIPIES.values():
        out += [s for step in steps for s in step]
    for c in COMBINATIONS:
        out += [s for step in c["states"] for s in step]
    out += list(COMPLETE_STATES)
    out += [f + "_melted" for f in ICE_CREAM_FLAVORS]
    return tuple(dict.fromkeys(out))


STATES = _states()
_STATE_INDEX = {s: i for i, s in enumerate(STATES)}


def _feature_names():
    n = ["time_left"]
    n += ["me_x", "me_y", "me_sin", "me_cos", "me_carrying"]
    n += [f"me_zone_{z}" for z in ZONES]
    n += ["partner_present", "partner_x", "partner_y", "partner_fwd",
          "partner_right", "partner_rel_sin", "partner_rel_cos",
          "partner_carrying"]
    n += [f"partner_zone_{z}" for z in ZONES]
    for s in STYPES:
        n += [f"st{s}_scanning", f"st{s}_attended"]
    for t in FOOD_TAGS:
        p = f"item{t}_"
        n += [p + k for k in ("present", "x", "y", "fwd", "right", "held_me",
                              "held_partner", "scan", "burning",
                              "combine_ready", "delivery_scan", "state_other")]
        n += [p + "state_" + s for s in STATES]
    for i in range(MAX_ORDERS):
        n += [f"order{i}_present", f"order{i}_age"]
        n += [f"order{i}_{o}" for o in ORDER_TYPES]
    return tuple(n)


FEATURES = _feature_names()
ACTIONS = ("throttle", "turn", "grab")


def featurize(obs, state, me) -> np.ndarray:
    """One body's view of a tick, laid out as FEATURES. obs/state are what
    Policy.act() receives (and what one log line stores); me is "p1" or "p2"."""
    f = []
    bodies = obs.get("bodies", {})
    b = bodies[me]
    mx, my, rot = float(b["x"]), float(b["y"]), float(b["rot"])
    fwd = (math.sin(rot), -math.cos(rot))
    right = (math.cos(rot), math.sin(rot))

    def absolute(x, y):
        return [(x - CENTRE[0]) / SCALE, (y - CENTRE[1]) / SCALE]

    def mine(x, y):
        dx, dy = x - mx, y - my
        return [(dx * fwd[0] + dy * fwd[1]) / SCALE,
                (dx * right[0] + dy * right[1]) / SCALE]

    def zone(z):
        return [float(z == k) for k in ZONES]

    f.append(float(state.get("time_left", 0.0)) / GAME_SECONDS)
    f += absolute(mx, my) + [math.sin(rot), math.cos(rot),
                             float(b.get("carrying") is not None)]
    f += zone(b.get("zone"))

    others = [n for n in bodies if n != me]
    p = bodies[others[0]] if others else None
    if p is None:
        f += [0.0] * (8 + len(ZONES))
    else:
        px, py, prot = float(p["x"]), float(p["y"]), float(p["rot"])
        f += [1.0] + absolute(px, py) + mine(px, py)
        f += [math.sin(prot - rot), math.cos(prot - rot),
              float(p.get("carrying") is not None)]
        f += zone(p.get("zone"))

    stations = state.get("stations", {})
    for s in STYPES:
        st = stations.get(s, {})
        f += [float(st.get("state") == "scanning"),
              float(bool(st.get("player_present", True)))]

    where = {int(t): (float(x), float(y)) for t, x, y in obs.get("tags", [])}
    items = state.get("items", {})
    scans = state.get("scans", {})
    burning = state.get("burning", {})
    ready = set(state.get("combine_ready", []))
    delivery = state.get("delivery_scans", {})
    held_me = b.get("carrying")
    held_partner = p.get("carrying") if p is not None else None
    for t in FOOD_TAGS:
        k = str(t)
        onehot = [0.0] * len(STATES)
        if t not in where:
            f += [0.0] * 12 + onehot
            continue
        x, y = where[t]
        s = items.get(k, {}).get("state")
        i = _STATE_INDEX.get(s)
        if i is not None:
            onehot[i] = 1.0
        f += [1.0] + absolute(x, y) + mine(x, y)
        f += [float(held_me == t), float(held_partner == t),
              float(scans.get(k, 0.0)), float(bool(burning.get(k))),
              float(k in ready), float(delivery.get(k, 0.0)),
              float(s is not None and i is None)]
        f += onehot

    now = float(state.get("now", obs.get("now", 0.0)))
    orders = state.get("orders", [])[:MAX_ORDERS]
    for i in range(MAX_ORDERS):
        if i < len(orders):
            o = orders[i]
            f += [1.0, (now - float(o["time"])) / GAME_SECONDS]
            f += [float(o["type"] == t) for t in ORDER_TYPES]
        else:
            f += [0.0] * (2 + len(ORDER_TYPES))

    return np.asarray(f, dtype=np.float32)


def action(inputs) -> np.ndarray:
    return np.asarray([float(inputs.get("throttle", 0.0)),
                       float(inputs.get("turn", 0.0)),
                       float(bool(inputs.get("grab")))], dtype=np.float32)


def from_log(path, every=1, include_bots=False):
    """(header, {player: (X, A, ticks)}) for one log. Bot-driven bodies are
    left out unless include_bots, so a dataset imitates people by default."""
    lines = list(logger.read(path))
    if not lines or lines[0].get("kind") != "header":
        raise ValueError(f"{path}: no header")
    header = lines[0]
    if header.get("schema") != logger.SCHEMA:
        raise ValueError(f"{path}: schema {header.get('schema')}, "
                         f"expected {logger.SCHEMA}")
    # The reset tick carries no bodies; nothing acted on it.
    ticks = [(i, l) for i, l in enumerate(lines)
             if l.get("kind") == "tick" and l["obs"].get("bodies")]
    roster = header.get("players", {})
    out = {}
    for me in roster:
        if roster[me].get("kind") == "bot" and not include_bots:
            continue
        X, A, T = [], [], []
        for w in range(0, len(ticks), every):
            window = ticks[w:w + every]
            i, first = window[0]
            if me not in first["obs"]["bodies"]:
                continue
            a = action(first["obs"].get("inputs", {}).get(me, {}))
            a[2] = float(any(l["obs"].get("inputs", {}).get(me, {}).get("grab")
                             for _, l in window))
            X.append(featurize(first["obs"], first["state"], me))
            A.append(a)
            T.append(i)
        if X:
            out[me] = (np.stack(X), np.stack(A), np.asarray(T, dtype=np.int32))
    return header, out


def build(paths, every=1, include_bots=False, strict_rules=False):
    """Stacks every log's examples. Returns the arrays np.savez writes."""
    current = logger.rules_hash()
    X, A, E, P, T, files, skipped = [], [], [], [], [], [], []
    for path in paths:
        try:
            header, views = from_log(path, every, include_bots)
        except (ValueError, KeyError) as e:
            skipped.append(f"{os.path.basename(path)}: {e}")
            continue
        stale = header.get("rules_hash") != current
        if stale and strict_rules:
            skipped.append(f"{os.path.basename(path)}: rules "
                           f"{header.get('rules_hash')} != current {current}")
            continue
        ep = len(files)
        files.append(path)
        for me, (x, a, t) in views.items():
            X.append(x)
            A.append(a)
            E.append(np.full(len(x), ep, dtype=np.int32))
            P.append(np.full(len(x), me))
            T.append(t)
        n = sum(len(v[0]) for v in views.values())
        print(f"  {os.path.basename(path)}  {'+'.join(views) or '-'}  "
              f"{n} examples{'  (older rules)' if stale else ''}")
    for s in skipped:
        print(f"  skipped {s}")
    if not X:
        raise SystemExit("no examples")
    return {
        "X": np.concatenate(X), "A": np.concatenate(A),
        "episode": np.concatenate(E), "player": np.concatenate(P),
        "tick": np.concatenate(T),
        "features": np.asarray(FEATURES), "actions": np.asarray(ACTIONS),
        "files": np.asarray(files), "every": np.int32(every),
        "rules_hash": np.asarray(current),
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="*",
                    help="log files (default: every log in the log dir)")
    ap.add_argument("-o", "--out", default=os.path.join(logger.DEFAULT_DIR,
                                                        "dataset.npz"))
    ap.add_argument("--every", type=int, default=6,
                    help="keep every Nth tick (default 6: 10 decisions/s)")
    ap.add_argument("--include-bots", action="store_true",
                    help="also learn from bot-driven bodies")
    ap.add_argument("--strict-rules", action="store_true",
                    help="skip logs recorded under different rules")
    a = ap.parse_args()
    paths = a.logs or sorted(glob.glob(os.path.join(logger.DEFAULT_DIR,
                                                    "*.jsonl.gz")))
    data = build(paths, a.every, a.include_bots, a.strict_rules)
    np.savez_compressed(a.out, **data)
    X, A = data["X"], data["A"]
    solo = float((X[:, FEATURES.index("partner_present")] == 0).mean())
    print(f"{len(X)} examples x {X.shape[1]} features from {len(data['files'])} "
          f"logs, {solo:.0%} solo, {int(A[:, 2].sum())} grabs -> {a.out}")
