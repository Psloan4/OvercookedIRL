import time
from item import Item, ItemHandler


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
        item_handler,
    ):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.type = type
        self.item_handler: ItemHandler = item_handler
        self.scan_time = float(scan_time)

        # State machine
        self.state = self.READY

        #   tag -> {"accum": float, "last_seen": float, "last_tick": float}
        self.scans: dict[int, dict] = {}

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
        return item is not None and item.state == self.type

    def _progress(self, accum: float) -> float:
        return 1.0 if self.scan_time <= 0 else min(accum / self.scan_time, 1.0)

    def _tick(self, ids: list[int]):
        """
        ids: the tag ids whose centers fall inside this station's region this
        frame, detected once on the full frame by the caller.

        Every item whose stage matches this station scans concurrently and
        independently. Returns per-tag progress + the tags that finished a scan
        this tick.
        """
        now = time.time()
        present_ids = set(ids)

        # Start a scan for any present, matching item we aren't already tracking.
        for tag in ids:
            if tag in self.scans:
                continue
            if self._matches(tag):
                self.scans[tag] = {"accum": 0.0, "last_seen": now, "last_tick": now}
                if self.DEBUG:
                    print(f"[SCAN START] Station={self.type} Tag={tag} scan_time={self.scan_time}")

        completed: list[int] = []

        for tag in list(self.scans.keys()):
            sc = self.scans[tag]

            # Item advanced (or went invalid) -> this scan is finished/irrelevant.
            if not self._matches(tag):
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

            if self._progress(sc["accum"]) >= 1.0:
                if self.DEBUG:
                    print(f"[SCAN FINISH] Station={self.type} Tag={tag} seen_time={sc['accum']:.3f}")
                self.item_handler.advance_item(tag)
                completed.append(tag)
                del self.scans[tag]

        scans_progress = {tag: self._progress(sc["accum"]) for tag, sc in self.scans.items()}
        self.state = self.SCANNING if self.scans else self.READY

        return {
            "state": self.state,
            "scans": scans_progress,   # tag -> 0..1 for every active scan
            "completed": completed,    # tags that finished a scan this tick
            "ids": ids,
        }
