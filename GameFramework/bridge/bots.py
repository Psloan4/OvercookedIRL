"""Policies that drive a body in place of the keyboard.

    python -m bridge.server --players 1 --p1-bot bridge.bots:Seek

A policy is any class the server can build as Policy(name) -- name is the
body it drives, "p1" or "p2" -- with:

  act(obs, state) -> {"throttle": -1..1, "turn": -1..1, "grab": bool}
      Called once per tick with the observe message Godot just sent (now,
      tags, players, and per-body "bodies" pose and "inputs") and the engine
      snapshot that tick produced -- the same pair a game log stores per line,
      so a model trained on logs sees exactly this at play time.
      throttle/turn hold like a held key until the next answer; grab is one
      press (pick up if empty-handed, else put down), so send it once rather
      than every tick or the body grabs and drops on alternate frames.
  reset(seed)   optional; called at the start of every round.

Coordinates are config.py's full-frame pixels. A body faces
(sin rot, -cos rot): rot 0 is up the screen, positive turns clockwise.

Godot applies a command a frame or two after the observation it answers, the
same lag a human has between seeing and pressing.
"""

from __future__ import annotations

import math

# Godot sizes the body off the Cooking station and puts the grab point at the
# arm tip (bridge_map.gd BODY_*, bridge_player.gd NOSE_*). With the current
# config that lands about here; Seek only needs it roughly.
ARM_REACH = 79.0
GRAB_SLACK = 25.0


class Idle:
    """Stands still. A placeholder body for a human to play around."""

    def __init__(self, name):
        self.name = name

    def act(self, obs, state):
        return {"throttle": 0.0, "turn": 0.0, "grab": False}


class Seek:
    """Walks to the nearest loose item, picks it up, carries it a moment and
    puts it down, then goes for another. No idea of recipes -- it exists to
    show the obs -> command loop closes, and as a template for a real policy.
    """

    HOLD_SECONDS = 1.5
    COOLDOWN = 0.4   # a grab's effect takes a frame or two to show in obs

    def __init__(self, name):
        self.name = name
        self.reset()

    def reset(self, seed=None):
        self._last_grab = -1e9
        self._picked_at = None
        self._skip = None    # the item just put down, so it moves on

    def act(self, obs, state):
        me = obs.get("bodies", {}).get(self.name)
        if me is None:
            return {}
        now = obs["now"]
        cooling = now - self._last_grab < self.COOLDOWN

        if me["carrying"] is not None:
            if self._picked_at is None:
                self._picked_at = now
            if now - self._picked_at > self.HOLD_SECONDS and not cooling:
                self._skip = me["carrying"]
                self._picked_at = None
                return self._press(now)
            return {"throttle": 0.3, "turn": 0.4}
        self._picked_at = None

        held = {b["carrying"] for b in obs["bodies"].values()}
        loose = [(t, x, y) for t, x, y in obs["tags"]
                 if t not in held and t != self._skip]
        if not loose:
            return {}
        _, tx, ty = min(loose, key=lambda t: math.hypot(t[1] - me["x"],
                                                        t[2] - me["y"]))
        dx, dy = tx - me["x"], ty - me["y"]
        dist = math.hypot(dx, dy)
        err = _wrap(math.atan2(dx, -dy) - me["rot"])

        if abs(dist - ARM_REACH) < GRAB_SLACK and abs(err) < 0.3 and not cooling:
            return self._press(now)
        # Turn first, then close in until the arm tip is over the item.
        throttle = max(-0.5, min(1.0, (dist - ARM_REACH) / 120.0))
        if abs(err) > 0.6:
            throttle = 0.0
        return {"throttle": throttle, "turn": max(-1.0, min(1.0, err * 2.0))}

    def _press(self, now):
        self._last_grab = now
        return {"throttle": 0.0, "turn": 0.0, "grab": True}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
