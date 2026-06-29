import time
import itertools
from item import Item, ItemHandler
from config import COMBINATIONS


class Station:
    READY = "ready"
    SCANNING = "scanning"

    GRACE_SECONDS = 2.0
    DEBUG = True

    def __init__(
        self,
        name,
        x, y, w, h,
        scan_time,
        burn_time,
        type,
        burn_type,
        combinable,
        item_handler,
        player_zone=None,
        cook_one=False,
    ):
        self.name = str(name),
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.type = type
        self.burn_type = burn_type
        self.combinable = combinable
        self.item_handler: ItemHandler = item_handler
        self.scan_time = float(scan_time)
        self.burn_time = float(burn_time)

        # PLAYER_ZONES/PLAYER_CAMS key, or None if this station isn't player-gated.
        self.player_zone = player_zone
        self.player_grace_frames = 0

        self.cook_one = cook_one
        self.target = None
        self.target_grace_frames = 0

        self.state = self.READY

        # tag -> {"accum", "last_seen", "last_tick"}
        self.scans: dict[int, dict] = {}

        # tag -> missed-frame count, for items waiting to combine
        self.combine_ready: dict[int, int] = {}

    def contains(self, px: float, py: float) -> bool:
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def reset(self):
        self.state = self.READY
        self.scans.clear()
        self.combine_ready.clear()

    def _ensure_item(self, tag: int) -> Item | None:
        if not self.item_handler.has_item(tag):
            self.item_handler.create_item(tag)
        if not self.item_handler.has_item(tag):
            return None
        return self.item_handler.get_item(tag)

    def _matches(self, tag: int) -> bool:
        """Does this tag's item currently need THIS station?"""
        item = self._ensure_item(tag)
        return item is not None and item.state in self.type

    def _is_burning(self, tag: int) -> bool:
        item = self.item_handler.get_item(tag) if self.item_handler.has_item(tag) else None
        return item is not None and item.state in self.burn_type

    def _is_combining(self, tag: int) -> bool:
        item = self.item_handler.get_item(tag) if self.item_handler.has_item(tag) else None
        return item is not None and item.state in self.combinable

    def _progress(self, tag: int, accum: float) -> float:
        # Burns run on burn_time; everything else on scan_time.
        duration = self.burn_time if self._is_burning(tag) else self.scan_time
        return 1.0 if duration <= 0 else min(accum / duration, 1.0)

    def _combine(self, ids, states_list):
        """Set each id to states_list and drop it from combine_ready."""
        for id in ids:
            self.item_handler.change_states(id, states_list.copy())
            del self.combine_ready[id]

    def _filter_burn_scans(self):
        """removes all tags from scans except those that are burning, returns True if any tags are deleted"""
        to_delete = []
        for tag in self.scans:
            if not self._is_burning(tag):
                to_delete.append(tag)
        for tag in to_delete:
            del self.scans[tag]
        return bool(len(to_delete))


    def _tick(self, ids: list[int], player_present: bool = True):
        """Advance one frame. ids are the tags inside this station's region;
        player_present gates scanning. Returns per-tag progress + events."""
        now = time.time()

    
        reset = False

        # No player beside the station -> wipe all progress. Scans restart from
        # zero when the player returns and the item is still present.
        if not player_present:
            self.player_grace_frames += 1
            if self.player_grace_frames >= 16: #Max number of scans player can be missing before reset
                # self.scans.clear()
                wiped = self._filter_burn_scans()
                if self.DEBUG and wiped:
                    print(f"[SCAN RESET] Station={self.name} player left zone "
                        f"(dropped {len(self.scans)} scan(s))")
                self.state = self.READY
                self.target = None
                reset = wiped
                # return {
                #     "state": self.state,
                #     "scans": self.scans,
                #     "completed": [],
                #     "ids": ids,
                #     "gated": self.player_zone is not None,
                #     "player_present": False,
                #     # True only on the tick a scan was actually lost to the player
                #     # leaving -- the UI uses this to flash the zone red.
                #     "reset_by_player": wiped,
                # }
        else:
            self.player_grace_frames = 0
        present_ids = set(ids)

        #When in cook_one mode, filter out all scans that aren't the target, ready to combine, or burnable
        if self.cook_one and player_present:
            #Case we have no target and need to pick a new one
            if self.target is None:
                for tag in ids:
                    if self._matches(tag) and not (tag in self.combine_ready) and not (self.item_handler.item_state(tag) in self.burn_type):
                        self.target = tag
                        break
            if not self.target is None:
                if not self.target in ids:
                    self.target_grace_frames += 1
                    if self.target_grace_frames > 16:
                        self.target = None
                        self.scans.clear()
                for tag in ids:
                    if (tag == self.target):
                        self.target_grace_frames = 0
                    if (tag != self.target):
                        if (tag in self.combine_ready) or (self._is_burning(tag)):
                            continue
                        else:
                            ids.remove(tag)

        # Start scans for matching items we aren't tracking yet.
        if self.cook_one:
            for tag in ids:
                if tag in self.scans:
                    continue
                if (tag == self.target) and (player_present):
                    self.scans[tag] = {"accum": 0.0, "last_seen": now, "last_tick": now}
                    if self.DEBUG:
                        print(f"[SCAN START] Station={self.name} Tag={tag} scan_time={self.scan_time}")
                if (self._is_burning(tag)) and (self._matches(tag)):
                    self.scans[tag] = {"accum": 0.0, "last_seen": now, "last_tick": now}
                    if self.DEBUG:
                        print(f"[BURN SCAN START] Station={self.name} Tag={tag} scan_time={self.scan_time}")
        else:
            for tag in ids:
                if tag in self.scans:
                    continue
                if self._matches(tag) and (player_present):
                    self.scans[tag] = {"accum": 0.0, "last_seen": now, "last_tick": now}
                    if self.DEBUG:
                        print(f"[SCAN START] Station={self.name} Tag={tag} scan_time={self.scan_time}")
                if (self._is_burning(tag)) and (self._matches(tag)):
                    self.scans[tag] = {"accum": 0.0, "last_seen": now, "last_tick": now}
                    if self.DEBUG:
                        print(f"[BURN SCAN START] Station={self.name} Tag={tag} scan_time={self.scan_time}")

        completed: list[int] = []

        seen_combine_ready = []

        for tag in list(self.scans.keys()):
            sc = self.scans[tag]

            # Item advanced (or is waiting to combine) -> this scan is finished/irrelevant.
            # if (not self._matches(tag)):
            #     del self.scans[tag]
            #     continue

            # Waiting to combine -> readiness is maintained below, by presence.
            if (tag in self.combine_ready):
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
                            f"[SCAN DROP] Station={self.name} Tag={tag} gone>{self.GRACE_SECONDS}s "
                            f"(lost {sc['accum']:.1f}/{self.scan_time}s)"
                        )
                    del self.scans[tag]
                continue

            if self._progress(tag, sc["accum"]) >= 1.0:
                self.target = None
                if self.DEBUG:
                    print(f"[SCAN FINISH] Station={self.name} Tag={tag} seen_time={sc['accum']:.3f}")
                if self.item_handler.get_item(tag).state in self.burn_type:
                    self.item_handler.burn_item(tag)
                elif self.item_handler.get_item(tag).state in self.combinable:
                    self.combine_ready[tag] = 0
                    break
                else:
                    self.target = None
                    self.item_handler.advance_item(tag)
                completed.append(tag)
                del self.scans[tag]

        # Maintain combine_ready by actual presence (not self.scans): items
        # carried away age out, and a player stepping away can't drop them.
        present_ready = []
        for tag in list(self.combine_ready):
            if tag in present_ids:
                self.combine_ready[tag] = 0
                present_ready.append(tag)
            else:
                self.combine_ready[tag] += 1
                if self.combine_ready[tag] > 16:
                    if self.DEBUG:
                        print(f"[FORGETTING] Station={self.name} Tag={tag}")
                    del self.combine_ready[tag]

        # Combine only items ready AND present here now (no combining from afar).
        if len(present_ready) > 1:
            for i, j in itertools.combinations(present_ready, 2):
                if i not in self.combine_ready or j not in self.combine_ready:
                    continue
                i_state = self.item_handler.get_item(i).state
                j_state = self.item_handler.get_item(j).state
                for combination in COMBINATIONS.keys():
                    if (i_state in combination) and (j_state in combination) and (i_state != j_state):
                        if self.DEBUG:
                            print(f"[COMBINING] Station={self.name} Tags={i},{j} Combination={combination}")
                        self._combine([i,j], COMBINATIONS[combination])
                        completed.append(i)
                        completed.append(j)
                        self.scans.pop(i, None)
                        self.scans.pop(j, None)
                        break

        scans_progress = {tag: self._progress(tag, sc["accum"]) for tag, sc in self.scans.items()}
        burning = {tag: self._is_burning(tag) for tag in self.scans}
        combining = {tag: self._is_combining(tag) for tag in self.scans}
        self.state = self.SCANNING if self.scans else self.READY

        return {
            "state": self.state,
            "scans": scans_progress,
            "burning": burning,
            "combining": combining,
            "combine_ready": list(self.combine_ready.keys()),
            "completed": completed,
            "ids": ids,
            "gated": self.player_zone is not None,
            "player_present": player_present,
            "reset_by_player": reset,
        }
