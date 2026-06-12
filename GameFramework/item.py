import random
from config import IDS, BURGER, FRIES

class Item:

    def __init__(self, tag_id):
        self.type = IDS[tag_id]
        if self.type == "PLAYER":
            #handle tag as a player
            pass
        elif self.type == "BURGER":
            states_list = BURGER.copy()
        elif self.type == "FRIES":
            states_list = FRIES.copy()
        self.state = random.choice(states_list.pop(0))
        self.future_states = states_list
    
    def advance_state(self):
        self.state = random.choice(self.future_states.pop(0))

class ItemHandler:

    def __init__(self):
        self.items = {}

    def clear(self):
        self.items.clear()

    def has_item(self, tag_id) -> bool:
        return tag_id in self.items
    
    def get_item(self, tag_id) -> Item:
        return self.items[tag_id]

    def create_item(self, tag_id):
        if not tag_id in IDS: return
        if self.has_item(tag_id): return
        self.items[tag_id] = Item(tag_id)
    
    def remove_item(self, tag_id):
        if not self.has_item(tag_id): return
        del self.items[tag_id]

    def advance_item(self, tag_id):
        if self.has_item(tag_id):
            item = self.get_item(tag_id)
            item.advance_state()