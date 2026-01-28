import cv2
import time
import numpy as np
from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay
from item import Item, ItemHandler


class Station:
    READY = "ready"
    SCANNING = "scanning"

    GRACE_FRAMES = 10


    #add an optional param covered_tag which defaults to none, it will also require that covered_tag is set to none or not found in order to scan
    def __init__(
        self,
        x, y, w, h,
        feed_relay,
        scan_time,
        type,
        item_handler,
        show_window: bool = False,
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

        # State machine
        self.state = self.READY
        self.scan_start_time: float | None = None

        # Target tracking
        self.target_tag: int | None = None
        self.miss_count = 0

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def _draw_detection_overlay(self, frame_bgr, corners, ids_list, target_tag=None):
        """Draw detected markers + IDs; highlight target tag if present."""
        out = frame_bgr.copy()
        if ids_list:
            ids_np = np.array(ids_list, dtype=np.int32).reshape(-1, 1)
            cv2.aruco.drawDetectedMarkers(out, corners, ids_np)

        if target_tag is not None:
            cv2.putText(
                out,
                f"TARGET: {target_tag}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return out

    def _compute_valid_tags(self, ids: list[int]) -> list[int]:
        """
        A tag is 'valid' for this station if its corresponding item exists (create if needed)
        and the item.state == this station's type.
        """
        valid = []
        for tag in ids:
            if not self.item_handler.has_item(tag):
                self.item_handler.create_item(tag)

            item: Item = self.item_handler.get_item(tag)
            if item.state == self.type:
                valid.append(tag)
        return valid

    def get_status(self):
        now = time.time()

        sub_sec = self.feed_relay.get_sub_section(self.x, self.y, self.w, self.h)

        # Single detection pass
        corners, ids = self.tag_det.detect(sub_sec)
        valid_tags = self._compute_valid_tags(ids)

        # Show annotated window (same detection results)
        if self.show_window:
            annotated = self._draw_detection_overlay(sub_sec, corners, ids, self.target_tag)
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

        # ---------------- State machine ----------------

        # READY: acquire a target if any valid tag is present
        if self.state == self.READY:
            if valid_tags:
                # Choose a deterministic target (first in list, or you can use min(valid_tags))
                self.target_tag = valid_tags[0]
                self.state = self.SCANNING
                self.scan_start_time = now
                self.miss_count = 0
                return {"state": self.SCANNING, "progress": 0.0, "target": self.target_tag}

            self.target_tag = None
            return {"state": self.READY, "progress": None, "target": None}

        # SCANNING: only track the locked target_tag
        if self.target_tag is None:
            # Safety fallback: if somehow scanning without a target, reset.
            self.state = self.READY
            self.scan_start_time = None
            self.miss_count = 0
            return {"state": self.READY, "progress": None, "target": None}

        target_present = self.target_tag in valid_tags

        if target_present:
            self.miss_count = 0
        else:
            self.miss_count += 1
            if self.miss_count > self.GRACE_FRAMES:
                # Target lost beyond grace -> abandon scan and await new target
                self.state = self.READY
                self.scan_start_time = None
                self.target_tag = None
                self.miss_count = 0
                return {"state": self.READY, "progress": None, "target": None}

        # Progress based on time since scan started
        elapsed = 0.0 if self.scan_start_time is None else (now - self.scan_start_time)
        progress = 1.0 if self.scan_time <= 0 else min(elapsed / self.scan_time, 1.0)

        # If scan completes AND target is present now, advance and reset to READY
        if progress >= 1.0 and target_present:
            self.item_handler.advance_item(self.target_tag)

            finished_tag = self.target_tag
            self.state = self.READY
            self.scan_start_time = None
            self.target_tag = None
            self.miss_count = 0

            # You can return READY immediately, or return SCANNING with 1.0 for one frame.
            return {"state": self.READY, "progress": None, "target": None, "completed": finished_tag}

        return {"state": self.SCANNING, "progress": progress, "target": self.target_tag}
