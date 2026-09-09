"""Serve the OvercookedIRL rules to a body simulator over TCP.

Godot connects, streams observations, and gets item states back. The rules
run here, in the same modules the live game uses, so the simulator can never
disagree with the real game about what an item becomes.

Run: python -m bridge.server [--port 8777] [--debug]
"""

from __future__ import annotations

import argparse
import socket

from bridge.engine import Engine, client_config
from bridge.protocol import HOST, PORT, LineReader, encode


def handle(conn, addr, debug=False):
    print(f"[bridge] client connected from {addr[0]}:{addr[1]}")
    engine = Engine(debug=debug)
    engine.start()
    reader = LineReader()
    ticks = 0

    # Push the handshake immediately; a client that wants it again can ask.
    conn.sendall(encode({"type": "config", **client_config()}))

    with conn:
        while True:
            chunk = conn.recv(65536)
            if not chunk:
                break
            for msg in reader.feed(chunk):
                kind = msg.get("type")
                if kind == "hello":
                    conn.sendall(encode({"type": "config", **client_config()}))
                elif kind == "reset":
                    engine.start()
                    ticks = 0
                    print("[bridge] round reset")
                    conn.sendall(encode({"type": "state",
                                         **engine.tick(0.0, [], {})}))
                elif kind == "observe":
                    snapshot = engine.tick(
                        msg.get("now", 0.0),
                        msg.get("tags", []),
                        msg.get("players", {}),
                    )
                    ticks += 1
                    for d in snapshot["delivered"]:
                        print(f"[bridge] +{d['points']} {d['state']}"
                              f"  (total {snapshot['points']})")
                    conn.sendall(encode({"type": "state", **snapshot}))
                elif kind == "error":
                    print(f"[bridge] {msg.get('message')}")
                else:
                    conn.sendall(encode({"type": "error",
                                         "message": f"unknown type {kind!r}"}))

    print(f"[bridge] client disconnected after {ticks} ticks, "
          f"final score {engine.points}")


def serve(host=HOST, port=PORT, debug=False):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(1)
        print(f"[bridge] rules engine listening on {host}:{port}")
        print("[bridge] waiting for Godot...")
        while True:
            conn, addr = srv.accept()
            try:
                handle(conn, addr, debug=debug)
            except ConnectionError as e:
                print(f"[bridge] connection lost: {e}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--debug", action="store_true")
    a = ap.parse_args()
    serve(a.host, a.port, a.debug)
