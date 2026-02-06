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

    def __init__(self):
        self.items = {}

    def has_item(self, tag_id) -> bool:
        return tag_id in self.items
    
    def get_item(self, tag_id) -> Item:
        return self.items[tag_id]

    def create_item(self, tag_id):
        if self.has_item(tag_id): return
        self.items[tag_id] = Item()
    
    def remove_item(self, tag_id):
        if not self.has_item(tag_id): return
        del self.items[tag_id]

    def advance_item(self, tag_id):
        if self.has_item(tag_id):
            item = self.get_item(tag_id)
            item.advance_state()