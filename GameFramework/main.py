import subprocess
import sys
import pygame
import os
import argparse
from PySide6.QtWidgets import QApplication, QStackedWidget
from PySide6.QtCore import QTimer

from final_station import FinalStation
from feed_relay import FeedRelay
from item import ItemHandler
from station import Station
from aruco_tag_detector import ArucoTagDetector

from style import APP_QSS
from config import CAMERA_HOST, CAMERA_PORT, STATION_CAMERA_DEV, FINAL_CAMERA_DEV, GAME_SECONDS, TICK_MS, STATION_DEFS, FINAL_STATION_DEF, TABLE_REGION
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
                    d["scan_time"],
                    d["type"],
                    self.item_handler,
                )
            )
        self.final_station = FinalStation(
            self.final_feed_relay,
            self.item_handler,
            FINAL_STATION_DEF,
        )


        self.detector = ArucoTagDetector("DICT_4X4_50")

        self.SCAN_EVERY = 3
        self._scan_tick = 0

        # pages
        self.start_page = StartPage(self.start_game)
        self.game_page = GamePage()
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

        # drop every item so tags start raw again
        self.item_handler.clear()
        for station in self.stations:
            station.reset()
        self.final_station.reset()
        self._scan_tick = 0

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
        try:
            self.final_feed_relay.update_image()
        except RuntimeError:
            pass

        # Throttles detection and it's station logic
        self._scan_tick = (self._scan_tick + 1) % self.SCAN_EVERY
        if self._scan_tick != 0:
            return

        frame = self.station_feed_relay.frame
        if frame is None:
            return

        tags = self._detect_tags(frame)  # [(tag_id, cx, cy), ...] full-frame px

        # Ensures items
        for tag_id, _cx, _cy in tags:
            self.item_handler.create_item(tag_id)

        # Hand each station the tags whose centers fall inside its region. Many
        # items scan at once; merge every station's per-tag progress.
        scan_progress: dict[int, float] = {}
        statuses: dict[str, dict] = {}
        for station in self.stations:
            ids = [tag_id for (tag_id, cx, cy) in tags if station.contains(cx, cy)]
            status = station._tick(ids)
            statuses[station.type] = status
            scan_progress.update(status.get("scans", {}))

        final_status = self.final_station._tick()
        for tag in final_status.get("delivered", []):
            self.inc_points(10)

        # Drive the station zones (pills + detected-tag info).
        self.game_page.update_stations(statuses, self.item_handler)

        # Same detection feeds the on-screen item positions.
        self.game_page.update_tags(self._build_render_list(tags, scan_progress))

    def _detect_tags(self, frame) -> list[tuple[int, float, float]]:
        """Detect once on the full frame; return (tag_id, center_x, center_y)."""
        corners, ids = self.detector.detect(frame)
        tags = []
        for c, tag_id in zip(corners, ids):
            pts = c.reshape(-1, 2)
            tags.append((tag_id, float(pts[:, 0].mean()), float(pts[:, 1].mean())))
        return tags

    def _build_render_list(self, tags, scan_progress: dict[int, float]) -> list[dict]:
        """Map detected tags to normalized table positions for the UI, with the
        item's type + stage so the picture reflects progression."""
        tx, ty, tw, th = TABLE_REGION

        render_list = []
        for tag_id, cx, cy in tags:
            nx = (cx - tx) / tw
            ny = (cy - ty) / th
            # Skip tags detected outside the table region (small slack for edges).
            if nx < -0.05 or nx > 1.05 or ny < -0.05 or ny > 1.05:
                continue

            item_type = None
            item_state = None
            if self.item_handler.has_item(tag_id):
                item = self.item_handler.get_item(tag_id)
                item_type = item.type
                item_state = item.state

            render_list.append({
                "id": tag_id,
                "nx": min(1.0, max(0.0, nx)),
                "ny": min(1.0, max(0.0, ny)),
                "progress": scan_progress.get(tag_id),
                "type": item_type,
                "state": item_state,
            })
        return render_list

    def run(self):
        self.stack.setMinimumSize(520, 420)
        self.stack.resize(820, 560)
        self.stack.showMaximized()


if __name__ == "__main__":

    app = QApplication()
    app.setStyleSheet(APP_QSS)
    ui = OvercookedIRLApp()
    ui.run()

    sys.exit(app.exec())
