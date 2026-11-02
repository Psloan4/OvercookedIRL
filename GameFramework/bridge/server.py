"""Serve the OvercookedIRL rules to a body simulator over TCP.

Godot connects, streams observations, and gets item states back. The rules
run here, in the same modules the live game uses, so the simulator can never
disagree with the real game about what an item becomes.

Run: python -m bridge.server [--port 8777] [--debug]
     python -m bridge.server --p1 alice --p2 bob --note "first session"
     python -m bridge.server --players 1 --p1 alice   # solo: Godot spawns p1 only
     python -m bridge.server --players 1 --p1-bot bridge.bots:Seek
Each round is logged to GameFramework/logs/ (see bridge.logger).
"""

from __future__ import annotations

import argparse
import importlib
import random
import socket

from bridge.engine import Engine, client_config
from bridge.logger import DEFAULT_DIR, GameLogger
from bridge.protocol import HOST, PORT, LineReader, encode


PLAYERS = ("p1", "p2")


def new_seed(fixed=None):
    return fixed if fixed is not None else random.SystemRandom().randrange(2**31)


def load_bot(spec, name):
    """"pkg.module:Class" -> Class(name). See bridge.bots for the interface."""
    module, _, cls = spec.partition(":")
    if not cls:
        raise SystemExit(f"bot spec {spec!r} must look like module:Class")
    return getattr(importlib.import_module(module), cls)(name)


def command(raw) -> dict:
    """A policy's answer, clamped to what a keyboard could have produced."""
    raw = raw or {}
    clamp = lambda v: max(-1.0, min(1.0, float(v or 0.0)))
    return {"throttle": clamp(raw.get("throttle")),
            "turn": clamp(raw.get("turn")),
            "grab": bool(raw.get("grab"))}


def handle(conn, addr, debug=False, logger=None, seed=None, players=PLAYERS,
           bots=None):
    """bots: {player name: policy}. Those bodies take the policy's commands,
    sent back with each state, instead of the keyboard."""
    bots = bots or {}
    print(f"[bridge] client connected from {addr[0]}:{addr[1]}")
    engine = Engine(debug=debug)
    round_seed = new_seed(seed)
    engine.start(round_seed)
    reader = LineReader()
    ticks = 0
    game_seconds = client_config()["game_seconds"]
    # Which bodies Godot spawns; the rules don't care how many there are.
    config_msg = encode({"type": "config", **client_config(),
                         "players": list(players), "bots": list(bots)})

    def step(obs):
        snapshot = engine.tick(obs["now"], obs["tags"], obs["players"])
        if logger:
            logger.tick(obs, snapshot)
        return snapshot

    # Push the handshake immediately; a client that wants it again can ask.
    conn.sendall(config_msg)

    with conn:
        while True:
            chunk = conn.recv(65536)
            if not chunk:
                break
            for msg in reader.feed(chunk):
                kind = msg.get("type")
                if kind == "hello":
                    conn.sendall(config_msg)
                elif kind == "reset":
                    round_seed = new_seed(seed)
                    engine.start(round_seed)
                    ticks = 0
                    print(f"[bridge] round reset (seed {round_seed})")
                    for bot in bots.values():
                        if hasattr(bot, "reset"):
                            bot.reset(round_seed)
                    if logger:
                        logger.begin(round_seed, game_seconds)
                    snapshot = step({"now": 0.0, "tags": [], "players": {}})
                    conn.sendall(encode({"type": "state", **snapshot}))
                elif kind == "observe":
                    # A client that never resets still gets logged.
                    if logger and not logger.open:
                        logger.begin(round_seed, game_seconds)
                    obs = {k: v for k, v in msg.items() if k != "type"}
                    obs.setdefault("now", 0.0)
                    obs.setdefault("tags", [])
                    obs.setdefault("players", {})
                    snapshot = step(obs)
                    ticks += 1
                    for d in snapshot["delivered"]:
                        print(f"[bridge] +{d['points']} {d['state']}"
                              f"  (total {snapshot['points']})")
                    # Commands ride on the reply rather than in the snapshot,
                    # so the logged state stays exactly what replay rebuilds.
                    # What a bot body acted on comes back in the next obs's
                    # "inputs", logged like a human's.
                    reply = {"type": "state", **snapshot}
                    if bots:
                        reply["commands"] = {name: command(bot.act(obs, snapshot))
                                             for name, bot in bots.items()}
                    conn.sendall(encode(reply))
                elif kind == "error":
                    print(f"[bridge] {msg.get('message')}")
                else:
                    conn.sendall(encode({"type": "error",
                                         "message": f"unknown type {kind!r}"}))

    print(f"[bridge] client disconnected after {ticks} ticks, "
          f"final score {engine.points}")


def serve(host=HOST, port=PORT, debug=False, logger=None, seed=None,
          players=PLAYERS, bots=None):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(1)
        print(f"[bridge] rules engine listening on {host}:{port}")
        print("[bridge] waiting for Godot...")
        while True:
            conn, addr = srv.accept()
            try:
                handle(conn, addr, debug=debug, logger=logger, seed=seed,
                       players=players, bots=bots)
            except ConnectionError as e:
                print(f"[bridge] connection lost: {e}")
            finally:
                if logger:
                    logger.end()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--seed", type=int, help="same seed every round")
    ap.add_argument("--log-dir", default=DEFAULT_DIR)
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--players", type=int, default=2, choices=(1, 2),
                    help="bodies Godot spawns; 1 is p1 alone")
    ap.add_argument("--p1", default="anon", help="participant id for p1")
    ap.add_argument("--p2", default="anon", help="participant id for p2 (ignored solo)")
    ap.add_argument("--p1-kind", default="human", choices=("human", "bot"))
    ap.add_argument("--p2-kind", default="human", choices=("human", "bot"))
    ap.add_argument("--p1-bot", metavar="MODULE:CLASS",
                    help="drive p1 with this policy instead of the keyboard")
    ap.add_argument("--p2-bot", metavar="MODULE:CLASS",
                    help="drive p2 with this policy instead of the keyboard")
    ap.add_argument("--note", default="", help="free text saved in each log")
    a = ap.parse_args()
    players = PLAYERS[:a.players]
    who = {"p1": {"id": a.p1, "kind": a.p1_kind},
           "p2": {"id": a.p2, "kind": a.p2_kind}}
    bots = {}
    for p, spec in (("p1", a.p1_bot), ("p2", a.p2_bot)):
        if spec is None:
            continue
        if p not in players:
            ap.error(f"--{p}-bot needs {p} in the game (see --players)")
        bots[p] = load_bot(spec, p)
        who[p] = {"id": a.p1 if p == "p1" else a.p2, "kind": "bot", "policy": spec}
    logger = None if a.no_log else GameLogger(
        a.log_dir, players={p: who[p] for p in players}, note=a.note)
    serve(a.host, a.port, a.debug, logger, a.seed, players, bots)
