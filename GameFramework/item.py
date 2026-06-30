import random
import copy
from config import IDS, RECIPIES

class Item:

    def __init__(self, tag_id):
        self.type = IDS[tag_id]
        if self.type in RECIPIES:
            recipies = copy.deepcopy(RECIPIES)
            states_list = recipies[self.type]
        else:
            # Non-food tags (PLAYER, THE GHOST, ...) aren't cookable; give them
            # an inert state so constructing one can never throw.
            states_list = [["inert"]]
        self.state = random.choice(states_list.pop(0))
        self.future_states = states_list
    
    def advance_state(self):
        self.state = random.choice(self.future_states.pop(0))

    def burn(self):
        if self.type == "CONE":
            self.state = self.state + "_melted"
        else:
            self.state = self.future_states.pop(-1)[0]
        self.future_states = []

    def change_states(self, states_list):
        self.state = random.choice(states_list.pop(0))
        self.future_states = states_list

        
class ItemHandler:

    def __init__(self):
        self.items = {}

    def clear(self):
        self.items.clear()

    def has_item(self, tag_id) -> bool:
        return tag_id in self.items
    
    def get_item(self, tag_id) -> Item:
        return self.items[tag_id]
    
    def item_state(self, tag_id):
        if self.has_item(tag_id):
            return self.get_item(tag_id).state
        else:
            return None


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

    def burn_item(self, tag_id):
        if self.has_item(tag_id):
            item = self.get_item(tag_id)
            item.burn()

    def change_states(self, tag_id, states_list):
        if self.has_item(tag_id):
            item = self.get_item(tag_id)
            item.change_states(states_list)