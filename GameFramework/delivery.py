"""What happens when a tagged item is placed at Delivery.

The camera-free core of the delivery rule, shared by the live game
(final_station.py), the headless sim (sim/world.py) and anything else that
needs to deliver a tag. Callers own presence detection (frame debouncing,
collision, whatever) and decide where the freed tag goes next.
"""

from config import BASE_STATES, ICE_CREAM_FLAVORS

# Outcomes.
NOTHING = "nothing"   # raw ingredient or unknown tag -- it just sits there
BINNED = "binned"     # removed from play, but matched no open order
SCORED = "scored"     # matched an order

# Points per delivered order. Make this a function of the record to add
# dynamic scoring; every caller reads it from here.
POINTS = 10


def resolve_delivery(item_handler, order_handler, tag):
    """Deliver or bin one tag. Returns (status, record or None).

    The item is removed whether or not it scored -- Delivery doubles as the
    bin. Base ingredients are left alone.
    """
    if not item_handler.has_item(tag):
        return NOTHING, None

    item = item_handler.get_item(tag)
    if item.state in BASE_STATES:
        return NOTHING, None

    state, item_type = item.state, item.type
    key = "ice_cream" if state in ICE_CREAM_FLAVORS else state

    item_handler.remove_item(tag)
    if not order_handler.complete_order(key):
        return BINNED, None

    return SCORED, {"tag": tag, "type": item_type, "state": state, "points": POINTS}
