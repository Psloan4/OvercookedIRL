import cv2
import time
import numpy as np
from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay
from item import Item, ItemHandler


class Station:
    READY = "ready"
    SCANNING = "scanning"

    # How many consecutive "missed" frames before we drop the target
    GRACE_FRAMES = 20

    def __init__(
        self,
        x, y, w, h,
        feed_relay,
        scan_time,
        type,
        item_handler,
        show_window: bool = False,
        covered=None
    ):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.type = type
        self.feed_relay: FeedRelay = feed_relay
        self.item_handler: ItemHandler = item_handler
        self.tag_det = ArucoTagDetector()
        self.scan_time = float(scan_time)

        self.show_window = show_window
        self.window_name = f"Station {type}"

        self.covered = covered

        # State machine
        self.state = self.READY

        # Target tracking
        self.target_tag: int | None = None
        self.miss_count = 0

        # Scan timing: accumulate "seen time" so flicker doesn't reset scanning
        self.scan_start_time: float | None = None   # when we entered SCANNING
        self.scan_accum: float = 0.0                # seconds of confirmed visibility
        self.last_tick_time: float | None = None    # to compute dt between ticks

        # Optional debug bookkeeping
        self.last_seen_ids: set[int] = set()

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def _draw_detection_overlay(self, frame_bgr, corners, ids_list, target_tag=None, offset_xy=(0, 0)):
        out = frame_bgr.copy()
        ox, oy = offset_xy

        if ids_list:
            shifted_corners = []
            for c in corners:
                c2 = c.copy()
                c2[:, :, 0] += ox
                c2[:, :, 1] += oy
                shifted_corners.append(c2)

            ids_np = np.array(ids_list, dtype=np.int32).reshape(-1, 1)
            cv2.aruco.drawDetectedMarkers(out, shifted_corners, ids_np)

        cv2.rectangle(out, (self.x, self.y), (self.x + self.w, self.y + self.h), (0, 255, 0), 2)

        cv2.putText(
            out,
            f"Station: {self.type}  State: {self.state}  miss={self.miss_count}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if target_tag is not None:
            cv2.putText(
                out,
                f"TARGET: {target_tag}",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return out

    def _full_reset(self):
        self.state = self.READY
        self.target_tag = None
        self.miss_count = 0
        self.scan_start_time = None
        self.scan_accum = 0.0
        self.last_tick_time = None
        self.last_seen_ids.clear()

    def _ensure_item(self, tag: int) -> Item:
        if not self.item_handler.has_item(tag):
            self.item_handler.create_item(tag)
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
            if item.state == self.type:
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
        return item.state == self.type

    def _reset_scan_timer(self):
        self.scan_start_time = None
        self.scan_accum = 0.0
        self.last_tick_time = None

    def _tick(self):
        now = time.time()

        # ROI for detection
        sub_sec = self.feed_relay.get_sub_section(self.x, self.y, self.w, self.h)

        # Detection pass on ROI
        corners, ids = self.tag_det.detect(sub_sec)  # ids: list[int]
        valid_tags = self._compute_valid_tags(ids)
        self.last_seen_ids = set(ids)

        # Show annotated window
        if self.show_window:
            if self.feed_relay.frame is None:
                raise RuntimeError("FeedRelay has no frame. Call feed_relay.update_image() before _tick().")

            annotated = self._draw_detection_overlay(
                self.feed_relay.frame,
                corners,
                ids,
                target_tag=self.target_tag,
                offset_xy=(self.x, self.y),
            )
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

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
                return {"state": self.READY, "progress": None, "target": None}

            # Prefer a valid tag if present; else any seen tag
            preferred = [t for t in candidates if t in valid_tags]
            self.target_tag = preferred[0] if preferred else candidates[0]
            self.miss_count = 0
            self._reset_scan_timer()

        # We have a target: prioritize it and count misses
        target_present = (self.target_tag in ids)

        if target_present:
            self.miss_count = 0
        else:
            self.miss_count += 1
            if self.miss_count > self.GRACE_FRAMES:
                # Drop target after grace timeout and immediately try to pick a new one this frame
                self.target_tag = None
                self.miss_count = 0
                self._reset_scan_timer()

                candidates = [t for t in ids if t != self.covered]
                if not candidates:
                    self.state = self.READY
                    return {"state": self.READY, "progress": None, "target": None}

                preferred = [t for t in candidates if t in valid_tags]
                self.target_tag = preferred[0] if preferred else candidates[0]
                self.state = self.READY
                return {"state": self.READY, "progress": None, "target": self.target_tag}

        # -----------------------------
        # SCANNING LOGIC (flicker-safe)
        # -----------------------------

        # If game logic says we shouldn't scan this target, stay READY but keep the target (grace still applies)
        if not self._target_should_scan(ids):
            self.state = self.READY
            self._reset_scan_timer()
            return {"state": self.READY, "progress": None, "target": self.target_tag}

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

        # Accumulate scan time ONLY while the tag is actually present
        #if target_present:
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
            }

        return {"state": self.SCANNING, "progress": progress, "target": self.target_tag}
