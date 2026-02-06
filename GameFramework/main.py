import subprocess
import sys
import pygame
import os
from PySide6.QtWidgets import QApplication, QStackedWidget
from PySide6.QtCore import QTimer

from final_station import FinalStation
from feed_relay import FeedRelay
from item import ItemHandler
from station import Station

from style import APP_QSS
from config import STATION_CAMERA_DEV, FINAL_CAMERA_DEV, GAME_SECONDS, TICK_MS, STATION_DEFS, GRID_PLACEMENT
from ui_components import StartPage, GamePage, EndPage


pygame.mixer.init()
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
INC_POINTS_SOUND = pygame.mixer.Sound(
    os.path.join(BASE_DIR, "assets", "inc_points.mp3")
)

class OvercookedIRLApp:
    def __init__(self):
        self.stack = QStackedWidget()
        self.stack.setWindowTitle("OvercookedIRL")

        # game state
        self.points = 0
        self.time_left = GAME_SECONDS

        # services
        self.station_feed_relay = FeedRelay(STATION_CAMERA_DEV)
        self.final_feed_relay = FeedRelay(FINAL_CAMERA_DEV)
        self.item_handler = ItemHandler()

        # stations (pure logic objects)
        self.stations: list[Station] = []
        for d in STATION_DEFS:
            self.stations.append(
                Station(
                    d["x"], d["y"], d["w"], d["h"],
                    self.station_feed_relay,
                    d["scan_time"],
                    d["type"],
                    self.item_handler,
                    d["show_window"],
                    d["covered"],
                )
            )
        self.final_station = FinalStation(self.final_feed_relay, self.item_handler)

        # pages
        self.start_page = StartPage(self.start_game)
        self.game_page = GamePage([s.type for s in self.stations], GRID_PLACEMENT)
        self.end_page = EndPage(self.go_to_start)

        self.stack.addWidget(self.start_page)
        self.stack.addWidget(self.game_page)
        self.stack.addWidget(self.end_page)
        self.stack.setCurrentWidget(self.start_page)

        # timers
        self.tick_timer = QTimer(self.stack)
        self.tick_timer.timeout.connect(self._tick)

        self.countdown_timer = QTimer(self.stack)
        self.countdown_timer.timeout.connect(self._countdown_tick)

        # initial HUD render
        self.game_page.set_points(self.points)
        self.game_page.set_time_left(self.time_left)

    # ---- scoring ----
    def inc_points(self, inc: int):
        self.points += inc
        self.game_page.set_points(self.points)

        INC_POINTS_SOUND.play()

    # ---- flow ----
    def start_game(self):
        self.points = 0
        self.time_left = GAME_SECONDS
        self.game_page.set_points(self.points)
        self.game_page.set_time_left(self.time_left)
        self.game_page.reset_station_cards()

        self.stack.setCurrentWidget(self.game_page)
        self.tick_timer.start(TICK_MS)
        self.countdown_timer.start(1000)

    def end_game(self):
        self.tick_timer.stop()
        self.countdown_timer.stop()
        self.end_page.set_score(self.points)
        self.stack.setCurrentWidget(self.end_page)

    def go_to_start(self):
        self.tick_timer.stop()
        self.countdown_timer.stop()
        self.stack.setCurrentWidget(self.start_page)

    # ---- timers ----
    def _countdown_tick(self):
        self.time_left -= 1
        self.game_page.set_time_left(self.time_left)
        if self.time_left <= 0:
            self.end_game()

    def _tick(self):
        self.station_feed_relay.update_image()
        self.final_feed_relay.update_image()

        for station in self.stations:
            status = station._tick()
            self.game_page.cards_by_type[station.type].update_from_status(status, self.item_handler)
        
        fss = self.final_station._tick()
        if fss["state"] == self.final_station.COMPLETE:
            self.inc_points(10)

    def run(self):
        self.stack.setMinimumSize(520, 420)
        self.stack.resize(820, 560)
        self.stack.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_QSS)

    ui = OvercookedIRLApp()
    ui.run()

    sys.exit(app.exec())
