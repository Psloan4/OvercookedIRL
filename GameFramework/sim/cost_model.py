"""How long an action takes.

The only place action timing lives. Swap FlatCost for a from->to matrix later
(10 unique station pairs) without touching the world loop or the trainer.
"""

from __future__ import annotations

import random

# Flat placeholder duration, in seconds, for every action.
DEFAULT_ACTION_SECONDS = 5.0


class FlatCost:
    """Every action takes the same time. jitter is a fraction of `seconds`
    to vary by (0.0 = deterministic)."""

    def __init__(self, seconds: float = DEFAULT_ACTION_SECONDS,
                 jitter: float = 0.0, rng: random.Random | None = None):
        self.seconds = float(seconds)
        self.jitter = float(jitter)
        self._rng = rng or random.Random()

    def duration(self, agent, action, from_loc, to_loc) -> float:
        # from_loc/to_loc unused here; they're in the signature so a matrix
        # model can drop in without changing callers.
        base = self.seconds
        if self.jitter:
            base *= 1.0 + self._rng.uniform(-self.jitter, self.jitter)
        return max(0.0, base * getattr(agent.spec, "speed", 1.0))
