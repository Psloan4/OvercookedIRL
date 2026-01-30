import cv2
import time
import numpy as np
from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay
from item import Item, ItemHandler


class Station:
    READY = "ready"
    SCANNING = "scanning"

    GRACE_FRAMES = 20

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

        # NEW: track what we saw last frame (to detect "new" tags)
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
            f"Station: {self.type}  State: {self.state}",
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

    def _compute_valid_tags(self, ids: list[int]) -> list[int]:
        valid = []
        for tag in ids:
            if not self.item_handler.has_item(tag):
                self.item_handler.create_item(tag)

            item: Item = self.item_handler.get_item(tag)
            if item.state == self.type:
                valid.append(tag)
        return valid

    def _select_new_tag(self, ids: list[int], valid_tags: list[int]) -> int | None:
        """
        Select a tag ONLY when a new tag appears compared to last frame.
        - Prefer a newly-appeared valid tag
        - Else choose the first newly-appeared tag
        """
        if not ids:
            return None

        new_ids = [t for t in ids if t not in self.last_seen_ids]
        if not new_ids:
            return None

        # Prefer newly-appeared valid
        new_valid = [t for t in new_ids if t in valid_tags]
        return new_valid[0] if new_valid else new_ids[0]

    def get_status(self):
        now = time.time()

        # ROI for detection
        sub_sec = self.feed_relay.get_sub_section(self.x, self.y, self.w, self.h)

        # Detection pass on ROI
        corners, ids = self.tag_det.detect(sub_sec)  # ids: list[int]
        valid_tags = self._compute_valid_tags(ids)

        # Decide if there's a "new" tag this frame -> select it
        newly_selected = self._select_new_tag(ids, valid_tags)
        if newly_selected is not None:
            self.target_tag = newly_selected
            self.miss_count = 0
            self.scan_start_time = now  # restart timing whenever a new tag is selected

        # Update last seen set AFTER selection logic
        self.last_seen_ids = set(ids)

        # Show annotated window
        if self.show_window:
            if self.feed_relay.frame is None:
                raise RuntimeError("FeedRelay has no frame. Call feed_relay.update_image() before get_status().")

            annotated = self._draw_detection_overlay(
                self.feed_relay.frame,
                corners,
                ids,
                target_tag=self.target_tag,
                offset_xy=(self.x, self.y),
            )
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

        # If no target selected yet, we're just ready
        if self.target_tag is None:
            self.state = self.READY
            self.scan_start_time = None
            self.miss_count = 0
            return {"state": self.READY, "progress": None, "target": None}

        # Target presence uses raw detections (validity doesn't matter for selection/reporting)
        target_present = self.target_tag in ids

        if target_present:
            self.miss_count = 0
        else:
            self.miss_count += 1
            if self.miss_count > self.GRACE_FRAMES:
                # drop target after grace timeout
                self.state = self.READY
                self.scan_start_time = None
                self.target_tag = None
                self.miss_count = 0
                return {"state": self.READY, "progress": None, "target": None}

        # Determine validity of currently selected target
        target_valid = self.target_tag in valid_tags

        # Only SCAN valid targets
        if not target_valid:
            self.state = self.READY
            # We still return the selected tag even though we aren't scanning it
            return {"state": self.READY, "progress": None, "target": self.target_tag}

        # Valid target: scanning state and progress
        self.state = self.SCANNING

        elapsed = 0.0 if self.scan_start_time is None else (now - self.scan_start_time)
        progress = 1.0 if self.scan_time <= 0 else min(elapsed / self.scan_time, 1.0)

        if progress >= 1.0 and target_present:
            self.item_handler.advance_item(self.target_tag)

            finished_tag = self.target_tag
            self.state = self.READY
            self.scan_start_time = None
            #self.target_tag = None
            self.miss_count = 0

            return {
                "state": self.READY,
                "progress": None,
                "target": None,
                "completed": finished_tag,
            }

        return {"state": self.SCANNING, "progress": progress, "target": self.target_tag}
