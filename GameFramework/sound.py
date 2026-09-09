import os
import pygame
from config import SOUND_EFFECTS

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

class SoundEffectPlayer:

    def __init__(self, alternate_sounds=False):
        i = 0
        if alternate_sounds:
            i = -1 #gets the alternate sound if theres an extra on the list, does nothing otherwise
        self.increment_points = pygame.mixer.Sound(os.path.join(BASE_DIR, "assets", SOUND_EFFECTS["increment_points"][i]))
        self.order_created = pygame.mixer.Sound(os.path.join(BASE_DIR, "assets", SOUND_EFFECTS["order_created"][i]))
        self.item_progressed = pygame.mixer.Sound(os.path.join(BASE_DIR, "assets", SOUND_EFFECTS["item_progressed"][i]))
        self.item_burnt = pygame.mixer.Sound(os.path.join(BASE_DIR, "assets", SOUND_EFFECTS["item_burnt"][i]))


    def play_increment_points(self):
        self.increment_points.play()

    def play_order_created(self):
        self.order_created.play()
    
    def play_item_progressed(self):
        self.item_progressed.play()

    def play_item_burnt(self):
        self.item_burnt.play()
