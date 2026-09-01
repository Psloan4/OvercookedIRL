"""Search for good actions.WEIGHTS values by playing simulated games.

Random search to explore, then hill-climbing to refine. Candidates are scored on
a fixed set of training seeds and the winner is re-checked on unseen holdout
seeds, so overfitting to a lucky set of games shows up.

Run `python -m sim.optimize` from GameFramework/.
"""

from __future__ import annotations

import json
import os
import random

import actions as A
from sim.world import play

# START actions never reach the sim (every tag already sits on the table), so
# this weight cannot affect fitness -- leave it alone rather than tune noise.
UNTUNABLE = {"start"}

# (low, high) search range per weight.
BOUNDS = {
    "deliver_ordered":   (0.0, 200.0),
    "deliver_unordered": (-100.0, 50.0),
    "combine_ready":     (0.0, 200.0),
    "combine_waiting":   (-50.0, 150.0),
    "progress_ordered":  (0.0, 200.0),
    "progress_idle":     (-50.0, 150.0),
    "trash":             (-50.0, 100.0),
    "wait_holding":      (0.0, 200.0),
    "wait_resetting":    (0.0, 200.0),
    "wait_start_work":   (0.0, 200.0),
}

TRAIN_SEEDS = tuple(range(14))
HOLDOUT_SEEDS = tuple(range(1000, 1014))


def evaluate(weights: dict, seeds) -> float:
    """Average points over `seeds` with `weights` applied."""
    saved = dict(A.WEIGHTS)
    A.WEIGHTS.update(weights)
    try:
        return sum(play(seed=s)["points"] for s in seeds) / len(seeds)
    finally:
        A.WEIGHTS.clear()
        A.WEIGHTS.update(saved)


def _sample(rng) -> dict:
    return {k: round(rng.uniform(lo, hi), 1) for k, (lo, hi) in BOUNDS.items()}


def _perturb(weights: dict, rng, sigma: float) -> dict:
    out = {}
    for k, (lo, hi) in BOUNDS.items():
        v = weights[k] + rng.gauss(0.0, sigma * (hi - lo))
        out[k] = round(min(hi, max(lo, v)), 1)
    return out


def search(random_iters=150, refine_iters=100, seed=0, verbose=True):
    rng = random.Random(seed)          # not the global RNG; games reseed that
    default = {k: A.WEIGHTS[k] for k in BOUNDS}
    best_score = evaluate(default, TRAIN_SEEDS)
    best = dict(default)
    if verbose:
        print(f"  baseline (hand-set weights): {best_score:.1f}\n  exploring...")

    for i in range(random_iters):
        cand = _sample(rng)
        score = evaluate(cand, TRAIN_SEEDS)
        if score > best_score:
            best_score, best = score, cand
            if verbose:
                print(f"    [random {i+1:3d}] {score:.1f}")

    if verbose:
        print("  refining...")
    sigma = 0.15
    for i in range(refine_iters):
        cand = _perturb(best, rng, sigma)
        score = evaluate(cand, TRAIN_SEEDS)
        if score > best_score:
            best_score, best = score, cand
            if verbose:
                print(f"    [refine {i+1:3d}] {score:.1f}")
        if (i + 1) % 25 == 0:
            sigma *= 0.6
    return best, best_score, default


def main():
    best, train_score, default = search()

    base_train = evaluate(default, TRAIN_SEEDS)
    base_hold = evaluate(default, HOLDOUT_SEEDS)
    best_hold = evaluate(best, HOLDOUT_SEEDS)

    print("\n  weight                 default -> tuned")
    for k in BOUNDS:
        print(f"    {k:<20} {default[k]:7.1f} -> {best[k]:7.1f}")
    for k in sorted(UNTUNABLE):
        print(f"    {k:<20} {A.WEIGHTS[k]:7.1f}    (not tuned: no effect in sim)")

    print(f"\n  train seeds   {base_train:6.1f} -> {train_score:6.1f}")
    print(f"  holdout seeds {base_hold:6.1f} -> {best_hold:6.1f}"
          f"   <- the number that matters")
    if best_hold <= base_hold:
        print("  WARNING: no gain on unseen games; likely overfit to train seeds.")

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_weights.json")
    with open(path, "w") as f:
        json.dump({"weights": best, "train": train_score, "holdout": best_hold}, f, indent=2)
    print(f"\n  saved -> {path}")


if __name__ == "__main__":
    main()
