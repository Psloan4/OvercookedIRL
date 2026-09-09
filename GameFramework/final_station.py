from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay
from item import Item, ItemHandler
from order import Order, OrderHandler
from config import BASE_STATES
from delivery import resolve_delivery, SCORED


class FinalStation:
    COMPLETE = "complete"
    UNCOMPLETE = "uncomplete"

    # Set True to log final-camera detections each tick (diagnostic only).
    DEBUG = False

    def __init__(self, feed_relay, item_handler, order_handler, station_def):
        self.feed_relay: FeedRelay = feed_relay
        self.item_handler: ItemHandler = item_handler
        self.order_handler: OrderHandler = order_handler
        self.x = station_def["x"]
        self.y = station_def["y"]
        self.w = station_def["w"]
        self.h = station_def["h"]
        self.type = station_def.get("type", "4")
        self.required_frames = station_def.get("required_frames", 2)
        self.tag_det: ArucoTagDetector = ArucoTagDetector("DICT_4X4_50")
        self.frames_seen: dict[int, int] = {}

    def contains(self, px: float, py: float) -> bool:
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def reset(self):
        self.frames_seen.clear()

    def _detect_tags(self, image) -> list[tuple[int, float, float]]:
        corners, ids = self.tag_det.detect(image)
        tags = []
        for c, tag_id in zip(corners, ids):
            pts = c.reshape(-1, 2)
            tags.append((tag_id, float(pts[:, 0].mean()), float(pts[:, 1].mean())))
        return tags

    def _tick(self):
        image = self.feed_relay.get_frame()
        if image is None:
            if self.DEBUG:
                print("[FINAL] no frame from final camera (cam/1)")
            return self.empty_status()
        return self.process(self._detect_tags(image))

    @staticmethod
    def empty_status() -> dict:
        return {"state": FinalStation.UNCOMPLETE, "delivered": [],
                "delivered_items": [], "ids": [], "scans": {}, "positions": {}}

    def process(self, tags: list[tuple[int, float, float]]) -> dict:
        """Run the delivery rule over already-located tags.

        Split out from _tick so callers that know where things are without a
        camera -- the Godot bridge, tests -- reuse this instead of copying it.
        tags are (tag_id, cx, cy) in full-frame pixel coords.
        """
        # tag -> position normalized (0..1) within the station region, for the UI.
        positions = {
            tag_id: ((cx - self.x) / self.w, (cy - self.y) / self.h)
            for tag_id, cx, cy in tags
            if self.contains(cx, cy)
        }
        ids_in_region = list(positions.keys())
        present_ids = set(ids_in_region)

        if self.DEBUG:
            all_ids = [t for t, _cx, _cy in tags]
            known = [t for t in ids_in_region if self.item_handler.has_item(t)]
            print(f"[FINAL] detected={all_ids} in_region={ids_in_region} known_items={known}")

        # Create tags if it sees IDs
        for tag in ids_in_region:
            self.item_handler.create_item(tag)

        for tag in list(self.frames_seen.keys()):
            if tag not in present_ids:
                del self.frames_seen[tag]

        delivered = []
        delivered_items = []
        for tag in ids_in_region:
            if not self.item_handler.has_item(tag):
                self.frames_seen.pop(tag, None)
                continue

            item: Item = self.item_handler.get_item(tag)
            if item.state in BASE_STATES:
                self.frames_seen.pop(tag, None)
                continue

            self.frames_seen[tag] = self.frames_seen.get(tag, 0) + 1
            if self.frames_seen[tag] >= self.required_frames:
                self.frames_seen.pop(tag, None)
                status, record = resolve_delivery(
                    self.item_handler, self.order_handler, tag
                )
                if status == SCORED:
                    delivered.append(tag)
                    # The record carries the item as it was before removal, so
                    # the delivery UI can show what was actually delivered.
                    delivered_items.append(
                        {"type": record["type"], "state": record["state"],
                         "points": record["points"]}
                    )

        scans = {
            tag: min(frames / self.required_frames, 1.0)
            for tag, frames in self.frames_seen.items()
        }
        state = self.COMPLETE if delivered else self.UNCOMPLETE
        return {
            "state": state,
            "delivered": delivered,
            "delivered_items": delivered_items,
            "ids": ids_in_region,
            "scans": scans,
            "positions": positions,
        }
