import time
import random
from config import COMPLETE_STATES

class Order:
    def __init__(self, time):
        self.type = random.choice(COMPLETE_STATES)
        self.time = time

class OrderHandler:
    def __init__(self, DEBUG=False):
        self.DEBUG = DEBUG
        self.orders = []
        self.order_num = 0
        
    def create_order(self):
        order = Order(time.time())
        self.orders.append(order)
        if self.DEBUG:
            print(f"[ORDER CREATED]: Type = {order.type}, Time = {order.time:.3f}, Index = {self.order_num}")
        self.order_num += 1
    
    def complete_index(self, item_state):
        """Returns the lowest possible index of a completed order, or None if no orders exist"""
        i = 0
        while i < self.order_num:
            order = self.orders[i]
            if order.type == item_state:
                return i
            i += 1
        return None
    
    def complete_order(self, item_state) -> bool:
        """Returns true when an order is successfully completed and removed, False if otherwise"""
        i = self.complete_index(item_state)
        if i is None: return False
        if self.DEBUG:
            time_to_complete = time.time() - self.orders[i].time
            print(f"[ORDER COMPLETE]: Type = {item_state}, Time = {time_to_complete:.3f}, Index = {i}")
        del self.orders[i]
        self.order_num -= 1
        return True
    
    def _tick(self):
        """to be called every tick from main, this function handles the system for adding orders periodically"""
        #ensure at least 2 orders
        while self.order_num < 2:
            self.create_order()
        if self.order_num > 5:
            return
        recent_order_time = self.orders[self.order_num-1].time
        if time.time() - recent_order_time > 15:
            self.create_order()

    def clear(self):
        self.orders = []
        self.order_num = 0
    
    def start_game(self):
        self.create_order()
        self.create_order()
        self.create_order()

        
