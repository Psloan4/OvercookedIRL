from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay
from item import Item, ItemHandler


class FinalStation:
    COMPLETE = "complete"
    UNCOMPLETE = "uncomplete"

    def __init__(self, feed_relay, item_handler):
        self.feed_relay: FeedRelay = feed_relay
        self.item_handler: ItemHandler = item_handler
        self.tag_det: ArucoTagDetector = ArucoTagDetector()

    def _tick(self):
        image = self.feed_relay.get_frame()

        corners, ids = self.tag_det.detect(image)

        for tag in ids:
            if self.item_handler.has_item(tag):
                item: Item = self.item_handler.get_item(tag)
                if item.state == "complete":
                    self.item_handler.remove_item(tag)
                    return {"state": self.COMPLETE}
        return {"state": self.UNCOMPLETE}