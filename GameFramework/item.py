

import random


class Item:

    def __init__(self):
        self.state = "1"
        self.future_states = [
            ["2a", "2b"],
            ["3"],
            ["complete"]
        ]
    
    def advance_state(self):
        self.state = random.choice(self.future_states.pop(0))

class ItemHandler:

    def __init__(self, point_inc_func):
        self.items = {}
        self.point_inc_func = point_inc_func

    def has_item(self, tag_id):
        return tag_id in self.items
    
    def get_item(self, tag_id):
        return self.items[tag_id]

    def create_item(self, tag_id):
        self.items[tag_id] = Item()
    
    def advance_item(self, tag_id):
        item = self.items[tag_id]
        item.advance_state()
        if item.state == "complete":
            self.point_inc_func(10)