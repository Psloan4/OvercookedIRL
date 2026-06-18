from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay
from item import Item, ItemHandler
from config import BASE_STATES


class FinalStation:
    COMPLETE = "complete"
    UNCOMPLETE = "uncomplete"

    def __init__(self, feed_relay, item_handler, station_def):
        self.feed_relay: FeedRelay = feed_relay
        self.item_handler: ItemHandler = item_handler
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
            return {"state": self.UNCOMPLETE, "delivered": [], "ids": [], "scans": {}}

        tags = self._detect_tags(image)
        ids_in_region = [
            tag_id
            for tag_id, cx, cy in tags
            if self.contains(cx, cy)
        ]
        present_ids = set(ids_in_region)

        for tag in list(self.frames_seen.keys()):
            if tag not in present_ids:
                del self.frames_seen[tag]

        delivered = []
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
                self.item_handler.remove_item(tag)
                self.frames_seen.pop(tag, None)
                if item.state == "complete":
                    delivered.append(tag)

        scans = {
            tag: min(frames / self.required_frames, 1.0)
            for tag, frames in self.frames_seen.items()
        }
        state = self.COMPLETE if delivered else self.UNCOMPLETE
        return {
            "state": state,
            "delivered": delivered,
            "ids": ids_in_region,
            "scans": scans,
        }
