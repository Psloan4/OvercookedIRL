"""Headless OvercookedIRL -- real rules, no camera or Qt.

Reuses the real Station, ItemHandler, OrderHandler and actions.py, driven by a
fake clock, so game rules can't drift out of sync. This file owns only what the
physical world normally provides: item locations, agent positions, action times.
Delivery is reimplemented here because final_station.py needs the camera.

Run `python -m sim.world` from GameFramework/.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from config import (
    STATION_DEFS,
    GAME_SECONDS,
    BASE_STATES,
    ICE_CREAM_FLAVORS,
    IDS,
)
from item import ItemHandler
from order import OrderHandler
from station import Station

import actions as A
from sim.cost_model import FlatCost

# Where a loose item sits when it isn't at any station -- the shared counter.
TABLE = "TABLE"

_NAME_TO_DEST = A._NAME_TO_DEST
_NAME_TO_STYPE = A.NAME_TO_STYPE

# Tags that are real food. Player head tags and the camera's phantom tag 17
# never become items.
FOOD_TAGS = tuple(
    t for t, kind in IDS.items() if kind not in ("PLAYER", "THE GHOST")
)


@dataclass
class AgentSpec:
    """One body's parameters. Identical agents share values; human + robot
    differ by speed / can_attend."""
    name: str
    speed: float = 1.0                    # multiplier on every action duration
    can_attend: tuple | None = None       # None = may use every station


class SimAgent:
    """One body: where it stands, what it's doing, and when it'll be free."""

    def __init__(self, spec: AgentSpec):
        self.spec = spec
        self.location: str | None = None   # dest label, or None while in transit
        self.busy_until: float = 0.0
        self.action = None                 # the Action being carried out
        self.carrying: int | None = None   # tag held mid-move

    @property
    def name(self) -> str:
        return self.spec.name

    def may_use(self, dest: str | None) -> bool:
        allowed = self.spec.can_attend
        return allowed is None or dest is None or dest in allowed


class World:
    """One simulated game."""

    def __init__(self, agents=None, cost=None, seed=None,
                 game_seconds: float = GAME_SECONDS, dt: float = 0.1,
                 debug: bool = False):
        self.rng = random.Random(seed)
        # item.py picks states with the global `random`, so seed it too if we
        # were given a seed -- otherwise games aren't reproducible.
        if seed is not None:
            random.seed(seed)

        self.dt = float(dt)
        self.game_seconds = float(game_seconds)
        self.debug = debug
        self.cost = cost or FlatCost()

        # --- the fake clock every reused module reads -----------------------
        self.now = 0.0
        clock = lambda: self.now

        self.items = ItemHandler()
        self.orders = OrderHandler(clock=clock)

        self.stations: list[Station] = []
        self._dest: dict[int, str] = {}    # id(station) -> COOK/SLICE/...
        self._stype: dict[int, str] = {}   # id(station) -> "1"/"2a"/"2b"/"3"
        for d in STATION_DEFS:
            st = Station(
                name=d["name"],
                x=d["x"], y=d["y"], w=d["w"], h=d["h"],
                scan_time=d["scan_time"],
                burn_time=d.get("burn_time", d["scan_time"]),
                type=d["type"],
                burn_type=d["burn_type"],
                combinable=d["combinable"],
                item_handler=self.items,
                player_zone=d.get("player_zone"),
                cook_one=d.get("cook_one"),
                combine_both=d.get("combine_both"),
                debug=False,
                clock=clock,
            )
            self.stations.append(st)
            self._dest[id(st)] = _NAME_TO_DEST[d["name"]]
            self._stype[id(st)] = _NAME_TO_STYPE[d["name"]]

        if agents is None:
            agents = [AgentSpec("agent1"), AgentSpec("agent2")]
        self.agents = [SimAgent(s) for s in agents]

        # Every food tag starts as a fresh base ingredient on the table.
        self.location: dict[int, str] = {}
        for tag in FOOD_TAGS:
            self.items.create_item(tag)
            self.location[tag] = TABLE

        self.points = 0
        self.delivered: list[str] = []
        self.orders.start_game()

    # --- views the reused modules need --------------------------------------

    def _present_items(self) -> list[dict]:
        """The 'what's on the table' list actions.available_actions() expects.

        Items being carried are excluded -- they're already committed.
        """
        return [
            {"id": tag, "state": self.items.item_state(tag)}
            for tag in self.location
            if self.items.has_item(tag)
        ]

    def _tags_at(self, dest: str) -> list[int]:
        return [t for t, loc in self.location.items() if loc == dest]

    def _tick_stations(self) -> dict[str, dict]:
        """Advance every station one dt, exactly as main.py's loop does."""
        statuses: dict[str, dict] = {}
        for st in self.stations:
            dest = self._dest[id(st)]
            # Gated stations only progress while somebody stands there.
            if st.player_zone is None:
                present = True
            else:
                present = any(a.location == dest for a in self.agents)
            # _tick mutates the list it's given (cook_one filtering), so hand it
            # a fresh one each time.
            status = st._tick(list(self._tags_at(dest)), present)
            statuses[self._stype[id(st)]] = status
        return statuses

    # --- delivery (mirrors final_station.py, minus the camera) --------------

    def _resolve_delivery(self, tag: int):
        state = self.items.item_state(tag)
        if state is None or state in BASE_STATES:
            return  # raw ingredients do nothing at delivery; they just sit there
        key = "ice_cream" if state in ICE_CREAM_FLAVORS else state
        # final_station removes the item whether or not it scored -- delivery
        # doubles as the bin.
        self.items.remove_item(tag)
        if self.orders.complete_order(key):
            self.points += 10
            self.delivered.append(state)
            if self.debug:
                print(f"  [{self.now:6.1f}s] DELIVERED {state} (+10)")
        elif self.debug:
            print(f"  [{self.now:6.1f}s] binned {state} (no order)")
        # The physical tag goes back in the pool as a fresh ingredient.
        self.items.create_item(tag)
        self.location[tag] = TABLE

    # --- agent decisions -----------------------------------------------------

    def _claimed_tags(self) -> set[int]:
        return {a.carrying for a in self.agents if a.carrying is not None}

    def _choose(self, agent: SimAgent, ranked) -> "A.Action | None":
        """Best action this agent may actually take right now.

        `ranked` is already sorted best-first by the WEIGHTS score, so this is
        just the first entry that passes the sim's physical constraints.
        """
        claimed = self._claimed_tags()
        occupied = {
            a.location for a in self.agents
            if a is not agent and a.location is not None
        }
        for act in ranked:
            if act.tag is not None and act.tag in claimed:
                continue                      # another agent already has it
            if act.tag is not None and self.location.get(act.tag) == act.dest:
                # Already sitting at that station -- carrying it there again is a
                # no-op that would snatch it back before the scan can start.
                # actions.py can't see this (it only infers position from live
                # scans), so the sim has to rule it out.
                continue
            if not agent.may_use(act.dest):
                continue                      # this body can't work that station
            if act.kind == "start":
                continue                      # advisory only; no concrete tag
            if act.verb == "WAIT" and act.dest in occupied:
                continue                      # presence is binary; one body is enough
            return act
        return None

    def _begin(self, agent: SimAgent, act):
        agent.action = act
        dur = self.cost.duration(agent, act, agent.location, act.dest)

        if act.verb == "WAIT":
            if agent.location == act.dest:
                # Already standing there: holding position is continuously
                # revisable, so only commit for one tick.
                agent.busy_until = self.now + self.dt
            else:
                agent.busy_until = self.now + dur   # walk over
            return

        # MOVE: pick the item up now so nobody else claims it, and so the
        # station it was sitting at stops scanning it.
        agent.carrying = act.tag
        self.location.pop(act.tag, None)
        agent.location = None                      # in transit, attending nothing
        agent.busy_until = self.now + dur

    def _finish(self, agent: SimAgent):
        act = agent.action
        agent.action = None
        if act is None:
            return
        agent.location = act.dest
        if act.verb == "WAIT":
            return
        tag = agent.carrying
        agent.carrying = None
        if tag is None:
            return
        self.location[tag] = act.dest
        if act.dest == A.DELIVER:
            self._resolve_delivery(tag)

    # --- the loop ------------------------------------------------------------

    def step(self):
        # 1. finish anything that's come due
        for agent in self.agents:
            if agent.action is not None and self.now >= agent.busy_until:
                self._finish(agent)

        # 2. free agents pick their next action off the ranked list
        idle = [a for a in self.agents if a.action is None]
        if idle:
            ranked = A.available_actions(
                self._present_items(), self.orders.orders, self._last_statuses
            )
            for agent in idle:
                act = self._choose(agent, ranked)
                if act is not None:
                    self._begin(agent, act)

        # 3. stations advance, 4. orders arrive
        self._last_statuses = self._tick_stations()
        self.orders._tick()

        self.now += self.dt

    def run(self) -> dict:
        self._last_statuses = {}
        while self.now < self.game_seconds:
            self.step()
        return {
            "points": self.points,
            "delivered": list(self.delivered),
            "deliveries": len(self.delivered),
            "orders_left": len(self.orders.orders),
        }


def play(seed=None, agents=None, cost=None, debug=False) -> dict:
    """Play one game and return its result. This is the fitness function the
    weight optimizer will call, averaged over many seeds."""
    return World(agents=agents, cost=cost, seed=seed, debug=debug).run()


if __name__ == "__main__":
    print("OvercookedIRL -- headless simulation\n")
    print(f"  {len(FOOD_TAGS)} food tags, {GAME_SECONDS:.0f}s game, "
          f"flat {FlatCost().seconds:.0f}s per action\n")
    results = [play(seed=s) for s in range(10)]
    for s, r in enumerate(results):
        got = ", ".join(r["delivered"]) or "-"
        print(f"  seed {s}: {r['points']:3d} pts  ({r['deliveries']} delivered: {got})")
    avg = sum(r["points"] for r in results) / len(results)
    print(f"\n  average over {len(results)} games: {avg:.1f} points")
