import time
import itertools
from item import Item, ItemHandler
from config import COMBINATIONS


class Station:
    READY = "ready"
    SCANNING = "scanning"

    GRACE_SECONDS = 2.0

    # Print per-event diagnostics ([SCAN START/FINISH], [TARGET DROP]).
    DEBUG = True

    def __init__(
        self,
        x, y, w, h,
        scan_time,
        type,
        burn_type,
        combinable,
        item_handler,
        player_zone=None,
    ):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.type = type
        self.burn_type = burn_type
        self.combinable = combinable
        self.item_handler: ItemHandler = item_handler
        self.scan_time = float(scan_time)

        # Key into PLAYER_ZONES/PLAYER_CAMS, or None if this station never
        # requires a player to be standing beside it.
        self.player_zone = player_zone
        self.player_grace_frames = 0

        # State machine
        self.state = self.READY

        #   tag -> {"accum": float, "last_seen": float, "last_tick": float}
        self.scans: dict[int, dict] = {}

        # tags ready for combining (uses a dictionary to track grace frames on combinable tags)
        self.combine_ready: dict[int, int] = {}

    def contains(self, px: float, py: float) -> bool:
        """Is the point (full-frame pixel coords) inside this station's region?"""
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def reset(self):
        """Clear all in-progress scans (used when a new round starts)."""
        self.state = self.READY
        self.scans.clear()

    def _ensure_item(self, tag: int) -> Item | None:
        # This is the "nonexistent tag" guard.
        if not self.item_handler.has_item(tag):
            self.item_handler.create_item(tag)
        if not self.item_handler.has_item(tag):
            return None
        return self.item_handler.get_item(tag)

    def _matches(self, tag: int) -> bool:
        """Does this tag's item currently need THIS station (state == type)?"""
        item = self._ensure_item(tag)
        return item is not None and item.state in self.type

    def _progress(self, accum: float) -> float:
        return 1.0 if self.scan_time <= 0 else min(accum / self.scan_time, 1.0)
    
    def _combine(self, ids, states_list):
        """sets ids to the given state list and removes them from the ready to combine list"""
        for id in ids:
            self.item_handler.change_states(id, states_list.copy())
            del self.combine_ready[id]


    def _tick(self, ids: list[int], player_present: bool = True):
        """
        ids: the tag ids whose centers fall inside this station's region this
        frame, detected once on the full frame by the caller.

        player_present: whether a player is standing beside this station this
        frame. Stations that don't require a player leave it True.

        Returns per-tag progress + the tags that finished a scan
        this tick.
        """
        now = time.time()

        # No player beside the station -> wipe all progress. Scans restart from
        # zero when the player returns and the item is still present.
        if not player_present:
            self.player_grace_frames += 1
            if self.player_grace_frames >= 16: #Max number of scans player can be missing before reset
                wiped = bool(self.scans)
                if self.DEBUG and wiped:
                    print(f"[SCAN RESET] Station={self.type} player left zone "
                        f"(dropped {len(self.scans)} scan(s))")
                self.scans.clear()
                self.state = self.READY
                return {
                    "state": self.state,
                    "scans": {},
                    "completed": [],
                    "ids": ids,
                    "gated": self.player_zone is not None,
                    "player_present": False,
                    # True only on the tick a scan was actually lost to the player
                    # leaving -- the UI uses this to flash the zone red.
                    "reset_by_player": wiped,
                }
        else:
            self.player_grace_frames = 0
        present_ids = set(ids)

        # Start a scan for any present, matching item we aren't already tracking.
        for tag in ids:
            if tag in self.scans:
                continue
            if self._matches(tag):
                self.scans[tag] = {"accum": 0.0, "last_seen": now, "last_tick": now}
                if self.DEBUG and (not tag in self.combine_ready):
                    print(f"[SCAN START] Station={self.type} Tag={tag} scan_time={self.scan_time}")

        completed: list[int] = []

        seen_combine_ready = []

        for tag in list(self.scans.keys()):
            sc = self.scans[tag]

            # Item advanced (or is waiting to combine) -> this scan is finished/irrelevant.
            if (not self._matches(tag)):
                del self.scans[tag]
                continue

            if (tag in self.combine_ready):
                self.combine_ready[tag] = 0 #resets grace period if we see the tag
                seen_combine_ready.append(tag)
                del self.scans[tag]
                continue

            present = tag in present_ids

            if present:
                sc["accum"] += now - sc["last_tick"]
                sc["last_seen"] = now
                sc["last_tick"] = now
            else:
                # Paused: freeze last_tick so the gap isn't counted on resume.
                sc["last_tick"] = now
                if (now - sc["last_seen"]) > self.GRACE_SECONDS:
                    if self.DEBUG:
                        print(
                            f"[SCAN DROP] Station={self.type} Tag={tag} gone>{self.GRACE_SECONDS}s "
                            f"(lost {sc['accum']:.1f}/{self.scan_time}s)"
                        )
                    del self.scans[tag]
                continue

            #process completed tags
            if self._progress(sc["accum"]) >= 1.0:
                if self.DEBUG:
                    print(f"[SCAN FINISH] Station={self.type} Tag={tag} seen_time={sc['accum']:.3f}")
                if self.item_handler.get_item(tag).state in self.burn_type:
                    self.item_handler.burn_item(tag)
                elif self.item_handler.get_item(tag).state in self.combinable:
                    self.combine_ready[tag] = 0
                    seen_combine_ready.append(tag)
                else:
                    self.item_handler.advance_item(tag)
                completed.append(tag)
                del self.scans[tag]

        #process combinations
        if len(self.combine_ready) > 1:
            for i, j in itertools.combinations(self.combine_ready, 2):
                i_state = self.item_handler.get_item(i).state
                j_state = self.item_handler.get_item(j).state
                for combination in COMBINATIONS.keys():
                    if (i_state in combination) and (j_state in combination):
                        if self.DEBUG:
                            print(f"[COMBINING] Station={self.type} Tags={i},{j} Combination={combination}")
                        self._combine([i,j], COMBINATIONS[combination])

        deleted_tags = []
        for tag in self.combine_ready:
            if not tag in seen_combine_ready:
                self.combine_ready[tag] += 1
                if self.combine_ready[tag] > 16:
                    deleted_tags.append(tag)
        for tag in deleted_tags:
            if self.DEBUG:
                print(f"[FORGETTING] Station={self.type} Tag={tag}")
            del self.combine_ready[tag]

        scans_progress = {tag: self._progress(sc["accum"]) for tag, sc in self.scans.items()}
        self.state = self.SCANNING if self.scans else self.READY

        return {
            "state": self.state,
            "scans": scans_progress,   # tag -> 0..1 for every active scan
            "completed": completed,    # tags that finished a scan this tick
            "ids": ids,
            "gated": self.player_zone is not None,
            "player_present": True,    # reached here only when present
            "reset_by_player": False,
        }
