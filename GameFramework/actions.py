"""
Action catalog + live "available actions" for OvercookedIRL.

Every action in the game is one of two things:

    MOVE(tag -> station)   pick a tagged item up and place it at a station
    WAIT(station)          stay present at a gated station so its scan continues

Stations transform items automatically once a tagged item sits in their region
long enough (and, for gated stations, a player is standing there), so the only
choices an agent -- human today, robot later -- ever makes are *where to put
each item next* and *which scanning station to stand at*.

Notes baked into this model (all grounded in config.py / station.py /
final_station.py):
  * There is no separate ingredient source: the game reuses a fixed set of
    tagged pieces, so "starting a dish" is just the first MOVE of a raw item to
    its station. No FETCH action.
  * The Delivery station doubles as the trash: any non-base item placed there is
    removed from play (see final_station.py). It scores points only if it
    matches an open order; otherwise it's just discarded. So "trash a burnt
    patty" is MOVE(-> DELIVER), tagged as a discard.
  * There is no neutral "holding" spot: an item you don't move simply stays
    where it is, and scans continue in place.

This module is pure data/logic (no Qt, no game loop): actions_window.py renders
what these functions return, and the robot information-handler can score the
same list.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from config import (
    STATION_DEFS,
    COMBINATIONS,
    COMPLETE_STATES,
    ICE_CREAM_FLAVORS,
    RECIPIES,
)

# --- Destinations (the real stations; DELIVER doubles as the trash) ---------
COOK = "COOK"          # station 1  (ungated)
SLICE = "SLICE"        # station 2a (gated)
ASSEMBLE = "ASSEMBLE"  # station 2b (gated)
PLATE = "PLATE"        # station 3  (gated)
DELIVER = "DELIVER"    # station 4  (ungated) -- also discards non-base items

# Config station name -> destination label.
_NAME_TO_DEST = {
    "Cooking": COOK,
    "Slicing": SLICE,
    "Assembling": ASSEMBLE,
    "Plating": PLATE,
}

# Station status key (station.type) -> destination label + friendly name.
_STYPE_TO_DEST = {"1": COOK, "2a": SLICE, "2b": ASSEMBLE, "3": PLATE, "4": DELIVER}
_DEST_NAME = {
    COOK: "Cooking",
    SLICE: "Slicing",
    ASSEMBLE: "Assembling",
    PLATE: "Plating",
    DELIVER: "Delivery",
}

# Tokens that appear in a station's `type` tuple but aren't real item states.
_SENTINELS = {"1", "2a", "2b", "3", "4"}

# States that score at Delivery. Ice-cream flavors satisfy the "ice_cream" order
# (see final_station.py's flavor -> "ice_cream" remap).
DELIVERABLE = {s for s in COMPLETE_STATES if s != "ice_cream"} | set(ICE_CREAM_FLAVORS)

# Failed items -- worthless, only good for discarding at Delivery.
DEAD_STATES = [
    "burnt_patty",
    "burnt_fries",
    "vanilla_melted",
    "chocolate_melted",
    "strawberry_melted",
    "trash",
]


def order_key(state: str) -> str:
    """The order type a deliverable state satisfies."""
    return "ice_cream" if state in ICE_CREAM_FLAVORS else state


def is_dead(state: str | None) -> bool:
    """Burnt / melted / trash -- discard at Delivery for no points."""
    if not state:
        return False
    return (
        state.startswith("burnt_")
        or state.endswith("_melted")
        or state == "trash"
    )


def _build_productive_index() -> dict[str, set[str]]:
    """state -> set of destinations that ADVANCE it (burn paths excluded)."""
    index: dict[str, set[str]] = {}
    for d in STATION_DEFS:
        dest = _NAME_TO_DEST.get(d["name"])
        if dest is None:
            continue
        advancing = set(d["type"]) - set(d.get("burn_type", ())) - _SENTINELS
        for state in advancing:
            index.setdefault(state, set()).add(dest)
    for state in DELIVERABLE:
        index.setdefault(state, set()).add(DELIVER)
    return index


def _build_partners() -> dict[str, set[str]]:
    """state -> states it combines with at ASSEMBLE."""
    partners: dict[str, set[str]] = {}
    for c in COMBINATIONS:
        partners.setdefault(c["base"], set()).add(c["add_on"])
        partners.setdefault(c["add_on"], set()).add(c["base"])
    return partners


PRODUCTIVE_DEST = _build_productive_index()
COMBINE_PARTNERS = _build_partners()

# --- Order -> "what to start" ------------------------------------------------
# The base ingredients you physically put down to BEGIN each ordered dish. Most
# of a recipe is derivable from config, but which base ingredients a dish needs
# isn't cleanly recoverable (burger/pizza braid a main line with a cheese line
# through COMBINATIONS), so it's spelled out here -- update it when adding a dish.
# The order.type keys come straight from COMPLETE_STATES.
ORDER_INGREDIENTS = {
    "complete_burger":       ["raw_patty", "cheese_block"],
    "complete_fries":        ["raw_potato"],
    "complete_cheese_fries": ["raw_potato", "cheese_block"],
    "complete_pizza":        ["dough", "cheese_block"],
    "ice_cream":             ["cone"],
}

# Friendly dish names for the START notes.
_DISH_NAME = {
    "complete_burger": "burger",
    "complete_fries": "fries",
    "complete_cheese_fries": "cheese fries",
    "complete_pizza": "pizza",
    "ice_cream": "ice cream",
}


def _build_transition_graph() -> dict[str, set[str]]:
    """state -> states it can become, from RECIPIES (linear) + COMBINATIONS.

    Used only to trace what's "already in progress" for an order. Burnt stages
    are dropped so they never count as progress toward a dish.
    """
    edges: dict[str, set[str]] = {}

    def add(a: str, b: str):
        if a and b and a != b:
            edges.setdefault(a, set()).add(b)

    def clean(stage):
        return [s for s in stage if not s.startswith("burnt_")]

    for stages in RECIPIES.values():
        cleaned = [c for c in (clean(st) for st in stages) if c]
        for i in range(len(cleaned) - 1):
            for a in cleaned[i]:
                for b in cleaned[i + 1]:
                    add(a, b)

    for c in COMBINATIONS:
        states = [s for s in (clean(st) for st in c["states"]) if s]
        if not states:
            continue
        for first in states[0]:
            add(c["base"], first)
            add(c["add_on"], first)
        for i in range(len(states) - 1):
            for a in states[i]:
                for b in states[i + 1]:
                    add(a, b)
    return edges


_GRAPH = _build_transition_graph()


def _ancestors(target: str) -> set[str]:
    """Every state that can eventually become `target`."""
    rev: dict[str, set[str]] = {}
    for a, tos in _GRAPH.items():
        for b in tos:
            rev.setdefault(b, set()).add(a)
    seen: set[str] = set()
    stack = [target]
    while stack:
        node = stack.pop()
        for parent in rev.get(node, ()):
            if parent not in seen:
                seen.add(parent)
                stack.append(parent)
    return seen


# The shared "cheese" add-on line -- excluded from a dish's covering set so a
# lone sliced_cheese doesn't read as "this specific dish is under way".
_ADD_ON_LINE: set[str] = set()
for _c in COMBINATIONS:
    _ADD_ON_LINE.add(_c["add_on"])
    _ADD_ON_LINE |= _ancestors(_c["add_on"])


def _build_covering() -> dict[str, set[str]]:
    """order type -> states that count as one unit of it already in progress
    (its own main line + the finished dish, minus the shared cheese line)."""
    covering: dict[str, set[str]] = {}
    for otype in ORDER_INGREDIENTS:
        targets = list(ICE_CREAM_FLAVORS) if otype == "ice_cream" else [otype]
        cov: set[str] = set(targets)
        for t in targets:
            cov |= _ancestors(t)
        cov -= _ADD_ON_LINE
        cov |= set(targets)  # keep the finished dish even if cheese-adjacent
        covering[otype] = cov
    return covering


COVERING = _build_covering()

# Lower priority sorts higher in the list (more worth doing). Kept for the
# coarse kind-bucket sort used by catalog()/reference views; the live ranking in
# available_actions() uses the richer WEIGHTS score below.
_KIND_PRIORITY = {
    "deliver": 0,
    "combine": 1,
    "progress": 1,
    "start": 2,
    "trash": 3,
    "wait": 4,
}

# --- Scoring weights ---------------------------------------------------------
# The live "best action" ranking is a plain linear score: each action's total is
# the sum of a handful of named components drawn from these weights. Nothing is
# hidden -- every point an action earns comes from one line here, and the window
# shows the same breakdown. Tune freely; higher total = more worth doing.
WEIGHTS = {
    "deliver_ordered":  100.0,  # hand in a dish an order is waiting for
    "deliver_unordered": -20.0,  # deliverable, but nothing ordered it -> a discard
    "combine_ready":     70.0,  # partner is on the table, combine now
    "combine_waiting":   25.0,  # on the combine line but no partner yet
    "progress_ordered":  55.0,  # advance an item an open order needs
    "progress_idle":     35.0,  # advance something with no order behind it
    "start":             40.0,  # put down a base ingredient to begin an open order
    "trash":             10.0,  # clear dead clutter off the table
    "wait_holding":      30.0,  # stay put to keep a running scan alive
    "wait_resetting":    45.0,  # nobody attending -> urgent, progress is draining
}

# Max bonus added to a delivery for how long its matching order has waited
# (FIFO fairness). Orders don't expire in this build, so this stays a light nudge
# rather than a hard deadline pressure.
URGENCY_MAX = 15.0


@dataclass(frozen=True)
class Action:
    verb: str                       # "MOVE" | "WAIT"
    kind: str                       # progress|combine|deliver|trash|wait
    dest: str | None = None         # destination / station label
    tag: int | None = None          # concrete tag (live) or None (template/global)
    item_state: str | None = None   # the item's state this action applies to
    note: str = ""                  # human-readable annotation
    wanted: bool = False            # deliver: is it ordered right now?
    ready: bool = True              # combine: partner present? / wait: player present?
    score: float = 0.0              # live ranking total (set by available_actions)
    weights: tuple = ()             # ((component_name, points), ...) that sum to score

    @property
    def priority(self) -> int:
        p = _KIND_PRIORITY.get(self.kind, 5)
        if self.kind == "deliver" and not self.wanted:
            p = 3  # deliverable, but nothing has ordered it -> discard-ish
        if self.kind == "combine" and not self.ready:
            p = 2  # can't combine until a partner shows up
        if self.kind == "wait" and not self.ready:
            p = 2  # nobody attending a scanning gated station -> it's resetting
        return p

    def label(self) -> str:
        if self.verb == "WAIT":
            where = _DEST_NAME.get(self.dest, self.dest)
            base = f"WAIT at {where}"
            return f"{base} -- {self.note}" if self.note else base
        if self.kind == "start":
            where = _DEST_NAME.get(self.dest, self.dest)
            base = f"START {self.item_state} at {where}"
            return f"{base}  ({self.note})" if self.note else base
        who = f"tag {self.tag} " if self.tag is not None else ""
        extra = f"  ({self.note})" if self.note else ""
        return f"MOVE {who}{self.item_state} -> {self.dest}{extra}"


def _wait_actions(present_items, station_status) -> list[Action]:
    """One WAIT per gated station that currently has a scan or a pending combine.

    Presence matters only at gated stations: leave one mid-scan and station.py
    wipes the progress after the grace period, so "stay here" is a real action.
    Cooking and Delivery are ungated -- they scan with nobody there, so they get
    no WAIT (the relevant move at Cooking is retrieving before it burns).
    """
    if not station_status:
        return []
    tag_state = {it.get("id"): it.get("state") for it in present_items}
    waits: list[Action] = []
    for stype, st in station_status.items():
        if not st or not st.get("gated"):
            continue
        scans = st.get("scans", {}) or {}
        combine_ready = st.get("combine_ready", {}) or {}
        if not scans and not combine_ready:
            continue
        present = st.get("player_present", True)
        parts = []
        for tag, progress in scans.items():
            name = tag_state.get(tag) or f"tag {tag}"
            parts.append(f"{name} {int(round(progress * 100))}%")
        for tag in combine_ready:
            name = tag_state.get(tag) or f"tag {tag}"
            parts.append(f"{name} ready to combine")
        note = ", ".join(parts)
        if not present:
            note += " -- nobody there, scan resetting!"
        waits.append(
            Action("WAIT", "wait", dest=_STYPE_TO_DEST.get(stype, stype),
                   item_state=None, note=note, ready=present)
        )
    return waits


def _start_actions(items, orders) -> list[Action]:
    """For each open order with nothing yet heading toward it, the base
    ingredient(s) to put down to begin that dish.

    "Heading toward it" = a present item on the dish's own main line (COVERING).
    Shortfall = open orders of that type minus items already in flight for it, so
    a second identical order still asks you to start another. Ingredients come
    from ORDER_INGREDIENTS; each one's first station is its productive dest.

    Known limits (fine for a start-advisor, worth knowing): coverage keys on the
    dish's MAIN line only, so it won't separately remind you of a forgotten
    cheese half; and a shared intermediate (e.g. cooked_fries) counts toward both
    the fries and cheese-fries orders.
    """
    if not orders:
        return []
    order_counts: dict[str, int] = {}
    for o in orders:
        order_counts[o.type] = order_counts.get(o.type, 0) + 1

    present_states = [it["state"] for it in items]
    starts: list[Action] = []
    for otype, count in order_counts.items():
        ingredients = ORDER_INGREDIENTS.get(otype)
        if not ingredients:
            continue
        cov = COVERING.get(otype, set())
        in_flight = sum(1 for s in present_states if s in cov)
        shortfall = count - in_flight
        if shortfall <= 0:
            continue
        dish = _DISH_NAME.get(otype, otype)
        note = f"for {dish}" + (f" (x{shortfall})" if shortfall > 1 else "")
        for ing in ingredients:
            dest = sorted(PRODUCTIVE_DEST.get(ing, [""]))[0]
            starts.append(Action("MOVE", "start", dest, None, ing, note=note))
    return starts


def _urgency_index(orders) -> "callable[[str], float]":
    """Build a fn: order_type -> urgency bonus in [0, URGENCY_MAX].

    Older orders (created earlier -> smaller `.time`) score higher, scaled across
    the spread of currently-open order ages. Returns 0 everywhere if orders carry
    no `.time` or all share one age.
    """
    times_by_type: dict[str, list[float]] = {}
    for o in orders:
        t = getattr(o, "time", None)
        if t is not None:
            times_by_type.setdefault(o.type, []).append(t)
    all_times = [t for ts in times_by_type.values() for t in ts]
    if not all_times:
        return lambda otype: 0.0
    tmin, tmax = min(all_times), max(all_times)
    span = tmax - tmin
    if span <= 0:
        return lambda otype: 0.0

    def bonus(otype: str) -> float:
        ts = times_by_type.get(otype)
        if not ts:
            return 0.0
        oldest = min(ts)  # earliest-created open order of this type = waited longest
        return round(URGENCY_MAX * (tmax - oldest) / span, 1)

    return bonus


def _score_action(a: Action, ordered_cover: set[str], urgency) -> Action:
    """Attach a linear score + its component breakdown to an action.

    Every component is one line from WEIGHTS (plus an optional urgency nudge on
    deliveries), so the total is fully explained by `a.weights`.
    """
    w: list[tuple[str, float]] = []
    if a.kind == "deliver":
        if a.wanted:
            w.append(("deliver ordered", WEIGHTS["deliver_ordered"]))
            u = urgency(order_key(a.item_state))
            if u:
                w.append(("order waiting", u))
        else:
            w.append(("no open order", WEIGHTS["deliver_unordered"]))
    elif a.kind == "combine":
        if a.ready:
            w.append(("partner ready", WEIGHTS["combine_ready"]))
        else:
            w.append(("waiting on partner", WEIGHTS["combine_waiting"]))
    elif a.kind == "progress":
        if a.item_state in ordered_cover:
            w.append(("advances an order", WEIGHTS["progress_ordered"]))
        else:
            w.append(("advances (no order)", WEIGHTS["progress_idle"]))
    elif a.kind == "start":
        w.append(("start ordered dish", WEIGHTS["start"]))
    elif a.kind == "trash":
        w.append(("clear clutter", WEIGHTS["trash"]))
    elif a.kind == "wait":
        if a.ready:
            w.append(("holding a scan", WEIGHTS["wait_holding"]))
        else:
            w.append(("scan resetting", WEIGHTS["wait_resetting"]))

    total = sum(v for _, v in w)
    return replace(a, score=total, weights=tuple(w))


def available_actions(present_items, orders=(), station_status=None) -> list[Action]:
    """Enumerate every legal action given the current table + orders.

    present_items:  iterable of dicts each with at least 'id' and 'state'
                    (main.py's render_list rows work directly).
    orders:         iterable of objects with a `.type` attr (OrderHandler.orders).
    station_status: optional {station.type: status_dict} from Station._tick(),
                    used to surface WAIT actions. Omit it and WAIT is skipped.

    Returns a flat list[Action], most-worth-doing first.
    """
    items = [it for it in present_items if it.get("state") and it["state"] != "inert"]
    present_states = {it["state"] for it in items}

    order_counts: dict[str, int] = {}
    for o in orders:
        order_counts[o.type] = order_counts.get(o.type, 0) + 1

    # dest -> tags already making PRODUCTIVE progress toward it right now. Built
    # from live scan data, so it's position-aware without any coordinates: a
    # station only scans a tag whose center is inside its region. A MOVE to a
    # dest an item is already progressing at is stale (the WAIT covers it), so
    # we suppress it. Burn scans are excluded -- there you DO want to move it.
    progressing_at: dict[str, set] = {}
    if station_status:
        for stype, st in station_status.items():
            if not st:
                continue
            dest = _STYPE_TO_DEST.get(stype)
            if dest is None:
                continue
            burning = st.get("burning", {}) or {}
            for tag in (st.get("scans", {}) or {}):
                if not burning.get(tag):
                    progressing_at.setdefault(dest, set()).add(tag)
            for tag in (st.get("combine_ready", {}) or {}):
                progressing_at.setdefault(dest, set()).add(tag)

    actions: list[Action] = []
    for it in items:
        tag = it.get("id")
        state = it["state"]

        if is_dead(state):
            actions.append(
                Action("MOVE", "trash", DELIVER, tag, state, note="discard, no points")
            )
            continue

        for dest in sorted(PRODUCTIVE_DEST.get(state, ())):
            if tag in progressing_at.get(dest, ()):
                continue  # already progressing here; the WAIT action covers it
            if dest == DELIVER:
                wanted = order_counts.get(order_key(state), 0) > 0
                note = "ordered now" if wanted else "no open order"
                actions.append(
                    Action("MOVE", "deliver", DELIVER, tag, state, note=note, wanted=wanted)
                )
            elif dest == ASSEMBLE and state in COMBINE_PARTNERS:
                partners = COMBINE_PARTNERS[state]
                ready_partners = sorted(p for p in partners if p in present_states)
                if ready_partners:
                    note = "combine with " + ", ".join(ready_partners)
                else:
                    note = "needs " + " or ".join(sorted(partners))
                actions.append(
                    Action("MOVE", "combine", ASSEMBLE, tag, state,
                           note=note, ready=bool(ready_partners))
                )
            else:
                actions.append(Action("MOVE", "progress", dest, tag, state))

    actions.extend(_start_actions(items, orders))
    actions.extend(_wait_actions(items, station_status))

    # Score every action so the list is a real ranking, not just kind buckets.
    # `ordered_cover` = states that advance some open order (its main line), used
    # to tell "advances an order" progress from idle progress.
    ordered_cover: set[str] = set()
    for otype in order_counts:
        ordered_cover |= COVERING.get(otype, set())
    urgency = _urgency_index(orders)
    actions = [_score_action(a, ordered_cover, urgency) for a in actions]

    # Best first; tie-break by tag so identical scores stay in a stable order.
    actions.sort(key=lambda a: (-a.score, a.tag if a.tag is not None else 1_000_000))
    return actions


def catalog() -> list[Action]:
    """Every action TEMPLATE, independent of live state -- the full action space
    a planner enumerates over, and a handy reference sheet. Run `python
    actions.py` to print it.
    """
    rows: list[Action] = []
    for state in sorted(PRODUCTIVE_DEST):
        for dest in sorted(PRODUCTIVE_DEST[state]):
            if dest == DELIVER:
                kind = "deliver"
            elif dest == ASSEMBLE and state in COMBINE_PARTNERS:
                kind = "combine"
            else:
                kind = "progress"
            note = ""
            if kind == "combine":
                note = "needs " + " or ".join(sorted(COMBINE_PARTNERS[state]))
            rows.append(Action("MOVE", kind, dest, None, state, note=note))
    for state in DEAD_STATES:
        rows.append(Action("MOVE", "trash", DELIVER, None, state, note="discard, no points"))
    # START templates: what to put down to begin each ordered dish.
    for otype, ingredients in ORDER_INGREDIENTS.items():
        dish = _DISH_NAME.get(otype, otype)
        for ing in ingredients:
            dest = sorted(PRODUCTIVE_DEST.get(ing, [""]))[0]
            rows.append(Action("MOVE", "start", dest, None, ing, note=f"for {dish}"))
    # WAIT is only meaningful at gated stations (Slicing / Assembling / Plating).
    for dest in (SLICE, ASSEMBLE, PLATE):
        rows.append(Action("WAIT", "wait", dest=dest, note="while a scan is running"))
    return rows


if __name__ == "__main__":
    rows = catalog()
    print("OvercookedIRL -- full action catalog\n")
    print("  MOVE (item state -> station):")
    by_state: dict[str, list[Action]] = {}
    starts: list[Action] = []
    waits: list[Action] = []
    for a in rows:
        if a.kind == "start":
            starts.append(a)
        elif a.verb == "WAIT":
            waits.append(a)
        else:
            by_state.setdefault(a.item_state, []).append(a)
    for state, acts in by_state.items():
        dests = "   ".join(
            f"{a.dest}{' [' + a.note + ']' if a.note else ''}" for a in acts
        )
        print(f"    {state:<22} {dests}")
    print("\n  START (per open order -- what to put down):")
    for a in starts:
        print(f"    {a.label()}")
    print("\n  WAIT (stay present at a scanning gated station):")
    for a in waits:
        print(f"    {a.label()}")
    print(f"\n  {len(rows)} action templates.")
