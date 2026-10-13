"""Per-round game logs: everything the engine was fed, plus what it answered.

One gzipped JSONL file per round. Lines, by "kind":

  header  schema, git commit, rules hash, seed, players, note
  tick    {"obs": observe message, "state": engine snapshot}
  end     final points and tick count

obs carries the engine inputs (now, tags, players) and, from Godot, the
per-player "bodies" and "inputs" the engine ignores but imitation needs.
Seed + obs is enough to rebuild every state; see bridge.replay.
"""

from __future__ import annotations

import datetime
import gzip
import hashlib
import json
import os
import subprocess

SCHEMA = 1

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_DIR = os.path.join(ROOT, "logs")

# Modules whose edits change the rules a log was recorded under.
RULES_FILES = ("config.py", "station.py", "item.py", "order.py",
               "delivery.py", "final_station.py", "actions.py",
               os.path.join("bridge", "engine.py"))


def rules_hash() -> str:
    h = hashlib.sha256()
    for name in RULES_FILES:
        with open(os.path.join(ROOT, name), "rb") as f:
            h.update(f.read())
    return h.hexdigest()[:16]


def git_commit() -> str | None:
    """HEAD, with a -dirty suffix for uncommitted changes."""
    try:
        run = lambda *a: subprocess.run(["git", *a], cwd=ROOT, text=True,
                                        capture_output=True, check=True).stdout.strip()
        sha = run("rev-parse", "HEAD")
        return sha + ("-dirty" if run("status", "--porcelain") else "")
    except (OSError, subprocess.CalledProcessError):
        return None


class GameLogger:
    """Opens a file per round. players: {"p1": {"id":..,"kind":..}, ...}."""

    def __init__(self, log_dir=DEFAULT_DIR, players=None, note=""):
        self.log_dir = log_dir
        self.players = players or {}
        self.note = note
        self.path = None
        self._f = None
        self._ticks = 0
        self._points = 0
        # Constant for the server's lifetime, so computed once.
        self._git = git_commit()
        self._rules = rules_hash()

    def begin(self, seed, game_seconds):
        self.end()
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(self.log_dir, f"{stamp}_seed{seed}.jsonl.gz")
        self._f = gzip.open(self.path, "wt", encoding="utf-8")
        self._ticks = 0
        self._points = 0
        self._write({
            "kind": "header",
            "schema": SCHEMA,
            "started": datetime.datetime.now().isoformat(timespec="seconds"),
            "git": self._git,
            "rules_hash": self._rules,
            "seed": seed,
            "game_seconds": game_seconds,
            "players": self.players,
            "note": self.note,
        })

    def tick(self, obs, state):
        if self._f is None:
            return
        self._ticks += 1
        self._points = state.get("points", 0)
        self._write({"kind": "tick", "obs": obs, "state": state})

    def end(self):
        if self._f is None:
            return
        self._write({"kind": "end", "ticks": self._ticks, "points": self._points})
        self._f.close()
        self._f = None
        print(f"[log] wrote {self._ticks} ticks -> {self.path}")

    @property
    def open(self):
        return self._f is not None

    def _write(self, obj):
        self._f.write(json.dumps(obj, separators=(",", ":")) + "\n")


def read(path):
    """Yields each line of a log as a dict. Tolerates a truncated tail."""
    with gzip.open(path, "rt", encoding="utf-8") as f:
        try:
            for line in f:
                if line.strip():
                    yield json.loads(line)
        except (EOFError, json.JSONDecodeError):
            return
