import time
from item import Item, ItemHandler


class Station:
    READY = "ready"
    SCANNING = "scanning"

    # How long (in seconds) the target can go undetected before we drop it AND
    # discard progress. While it's missing but under this window, scanning just
    # PAUSES (progress is preserved), so a brief dropout doesn't reset the scan.
    # Generous on purpose: real cameras lose a tag for a moment all the time.
    GRACE_SECONDS = 4.0

    # Print per-event diagnostics ([SCAN START/FINISH], [TARGET DROP]).
    DEBUG = True

    def __init__(
        self,
        x, y, w, h,
        scan_time,
        type,
        item_handler,
        covered=None
    ):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.type = type
        self.item_handler: ItemHandler = item_handler
        self.scan_time = float(scan_time)

        self.covered = covered

        # State machine
        self.state = self.READY

        # Target tracking
        self.target_tag: int | None = None
        self.miss_count = 0
        self.last_seen_time: float | None = None    # wall time target was last detected

        # Scan timing: accumulate "seen time" so flicker doesn't reset scanning
        self.scan_start_time: float | None = None   # when we entered SCANNING
        self.scan_accum: float = 0.0                # seconds of confirmed visibility
        self.last_tick_time: float | None = None    # to compute dt between ticks

        # Optional debug bookkeeping
        self.last_seen_ids: set[int] = set()

    def contains(self, px: float, py: float) -> bool:
        """Is the point (full-frame pixel coords) inside this station's region?"""
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def _full_reset(self):
        self.state = self.READY
        self.target_tag = None
        self.miss_count = 0
        self.last_seen_time = None
        self.scan_start_time = None
        self.scan_accum = 0.0
        self.last_tick_time = None
        self.last_seen_ids.clear()

    def _ensure_item(self, tag: int) -> Item | None:
        # create_item is a no-op for tags that aren't real items (not in IDS),
        # so it can still be missing afterwards -> return None instead of
        # crashing on get_item. This is the "nonexistent tag" guard.
        if not self.item_handler.has_item(tag):
            self.item_handler.create_item(tag)
        if not self.item_handler.has_item(tag):
            return None
        return self.item_handler.get_item(tag)

    def _compute_valid_tags(self, ids: list[int]) -> list[int]:
        """
        Valid tags among the *currently detected* ids.
        (Still useful for choosing the next target.)
        """
        valid = []
        for tag in ids:
            if self.covered == tag:
                continue

            item = self._ensure_item(tag)
            if item is not None and item.state == self.type:
                valid.append(tag)
        return valid

    def _target_should_scan(self, ids: list[int]) -> bool:
        """
        Whether the CURRENT target is eligible to be scanned based on game logic.
        IMPORTANT: This must NOT depend on whether the tag was detected this frame,
        otherwise flicker will interrupt scanning.
        """
        if self.target_tag is None:
            return False

        # Covered logic: if "covered" tag is visible, block scanning
        tag_covered_ok = (self.covered is None) or (self.covered not in ids)
        if not tag_covered_ok:
            return False

        item = self._ensure_item(self.target_tag)
        return item is not None and item.state == self.type

    def _reset_scan_timer(self):
        self.scan_start_time = None
        self.scan_accum = 0.0
        self.last_tick_time = None

    def _tick(self, ids: list[int]):
        """
        ids: the tag ids whose centers fall inside this station's region this
        frame, detected once on the full frame by the caller. The station no
        longer detects on its own ROI crop -- detection is shared.
        """
        now = time.time()

        valid_tags = self._compute_valid_tags(ids)
        self.last_seen_ids = set(ids)

        # -----------------------------
        # TARGET SELECTION POLICY
        # -----------------------------

        # If we have no target, choose one from what's currently seen
        if self.target_tag is None:
            candidates = [t for t in ids if t != self.covered]
            if not candidates:
                self.state = self.READY
                self.miss_count = 0
                self._reset_scan_timer()
                return {"state": self.READY, "progress": None, "target": None, "ids": ids}

            # Prefer a valid tag if present; else any seen tag
            preferred = [t for t in candidates if t in valid_tags]
            self.target_tag = preferred[0] if preferred else candidates[0]
            self.miss_count = 0
            self.last_seen_time = now
            self._reset_scan_timer()

        # We have a target: prioritize it and count misses
        target_present = (self.target_tag in ids)

        if target_present:
            self.miss_count = 0
            self.last_seen_time = now
        else:
            self.miss_count += 1
            if self.last_seen_time is None:
                self.last_seen_time = now

            # Drop the target only after it's been gone for GRACE_SECONDS of
            # real time (frame-rate independent), not a fixed frame count.
            if (now - self.last_seen_time) > self.GRACE_SECONDS:
                # Drop target after grace timeout and immediately try to pick a new one this frame
                if self.DEBUG:
                    print(
                        f"[TARGET DROP] Station={self.type} "
                        f"Tag={self.target_tag} gone>{self.GRACE_SECONDS}s "
                        f"(lost {self.scan_accum:.1f}/{self.scan_time}s progress)"
                    )
                self.target_tag = None
                self.miss_count = 0
                self.last_seen_time = None
                self._reset_scan_timer()

                candidates = [t for t in ids if t != self.covered]
                if not candidates:
                    self.state = self.READY
                    return {"state": self.READY, "progress": None, "target": None, "ids": ids}

                preferred = [t for t in candidates if t in valid_tags]
                self.target_tag = preferred[0] if preferred else candidates[0]
                self.last_seen_time = now
                self.state = self.READY
                return {"state": self.READY, "progress": None, "target": self.target_tag, "ids": ids}

        # -----------------------------
        # SCANNING LOGIC (flicker-safe)
        # -----------------------------

        # If game logic says we shouldn't scan this target, stay READY but keep
        # the target (grace still applies). PAUSE progress instead of wiping it:
        # a one-frame glitch (covered tag flickers in, item state hiccups) must
        # not throw away seconds of accumulated scanning. We freeze last_tick_time
        # so the paused interval isn't counted when scanning resumes.
        if not self._target_should_scan(ids):
            self.state = self.READY
            self.last_tick_time = now
            return {"state": self.READY, "progress": None, "target": self.target_tag, "ids": ids}

        # Valid target: scanning. Do NOT reset scan timer on brief misses.
        self.state = self.SCANNING
        if self.scan_start_time is None:
            self.scan_start_time = now
            self.scan_accum = 0.0
            self.last_tick_time = now
            print(
                f"[SCAN START] Station={self.type} "
                f"Tag={self.target_tag} "
                f"t={self.scan_start_time:.3f} "
                f"scan_time={self.scan_time}"
            )

        dt = 0.0 if self.last_tick_time is None else (now - self.last_tick_time)
        self.last_tick_time = now

        # Accumulate scan time ONLY while the tag is actually present.
        # While it's briefly missing (within GRACE_SECONDS) we simply don't add
        # time -> progress PAUSES and is preserved, instead of filling on thin
        # air and then snapping to zero when grace expires.
        if target_present:
            self.scan_accum += dt

        progress = 1.0 if self.scan_time <= 0 else min(self.scan_accum / self.scan_time, 1.0)

        # Only finish if we've accumulated enough "seen time" AND it's present right now
        if progress >= 1.0 and target_present:
            print(
                f"[SCAN FINISH] Station={self.type} "
                f"Tag={self.target_tag} "
                f"seen_time={self.scan_accum:.3f}"
            )

            self.item_handler.advance_item(self.target_tag)

            finished_tag = self.target_tag
            self.state = self.READY
            self.target_tag = None
            self.miss_count = 0
            self._reset_scan_timer()

            return {
                "state": self.READY,
                "progress": None,
                "target": None,
                "completed": finished_tag,
                "ids": ids
            }

        return {"state": self.SCANNING, "progress": progress, "target": self.target_tag, "ids": ids}
