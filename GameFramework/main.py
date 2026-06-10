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
from config import CAMERA_HOST, CAMERA_PORT, STATION_CAMERA_DEV, FINAL_CAMERA_DEV, GAME_SECONDS, TICK_MS, STATION_DEFS, GRID_PLACEMENT, TABLE_REGION
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

        # Independent full-frame detector used ONLY for rendering item positions.
        # Kept separate from the per-station logic (station.py) so we don't have
        # to touch that file: stations score on ROI crops, this maps the whole
        # frame -> screen.
        self.render_detector = ArucoTagDetector("DICT_4X4_50")
        # Full-frame detection is expensive; run it every Nth UI tick instead of
        # every frame. At TICK_MS=16 (~60fps), RENDER_EVERY=4 => ~15 position
        # updates/sec, which is plenty since tags barely move between frames.
        self.RENDER_EVERY = 4
        self._render_tick = 0

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

        # tag_id -> scan progress (0..1) for whichever tag each station is scanning
        scan_progress: dict[int, float] = {}
        for station in self.stations:
            status = station._tick()
            self.game_page.cards_by_type[station.type].update_from_status(status, self.item_handler)
            target = status.get("target")
            if status.get("state") == Station.SCANNING and target is not None:
                scan_progress[target] = status.get("progress")

        fss = self.final_station._tick()
        if fss["state"] == self.final_station.COMPLETE:
            self.inc_points(10)

        # Draw items where their tags physically are on the table. Throttled:
        # the full-frame detection only runs every RENDER_EVERY ticks so it
        # doesn't starve the UI event loop.
        self._render_tick = (self._render_tick + 1) % self.RENDER_EVERY
        if self._render_tick == 0:
            frame = self.station_feed_relay.frame
            if frame is not None:
                self.game_page.update_tags(self._build_render_list(frame, scan_progress))

    def _build_render_list(self, frame, scan_progress: dict[int, float]) -> list[dict]:
        """Map every tag in the full frame to a normalized table position."""
        corners, ids = self.render_detector.detect(frame)
        tx, ty, tw, th = TABLE_REGION

        render_list = []
        for c, tag_id in zip(corners, ids):
            pts = c.reshape(-1, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            nx = (cx - tx) / tw
            ny = (cy - ty) / th
            # Skip tags detected outside the table region (small slack for edges).
            if nx < -0.05 or nx > 1.05 or ny < -0.05 or ny > 1.05:
                continue

            render_list.append({
                "id": tag_id,
                "nx": min(1.0, max(0.0, nx)),
                "ny": min(1.0, max(0.0, ny)),
                "progress": scan_progress.get(tag_id),
            })
        return render_list

    def run(self):
        self.stack.setMinimumSize(520, 420)
        self.stack.resize(820, 560)
        # The UI (and the table view) is sized for a large table-top display;
        # open maximized so there's room for the table and the station cards.
        self.stack.showMaximized()


if __name__ == "__main__":

    # #Arguments
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--station",
        type = int,
        default = 4,
        help = "Index of camera used for station detection (Default = 4)"
    )
    ap.add_argument(
        "--host",
        type = str,
        help = "IP of host device streaming camera feeds"
    )
    ap.add_argument(
        "--port",
        type = int,
        help = "Port of host device streaming camera feeds"
    )
    ap.add_argument(
        "--local",
        action = "store_true",
        help = "Configures main to run using local cameras instead of streaming"
    )
    args = ap.parse_args()

    if not args.host is None:
        CAMERA_HOST = args.host
    if not args.port is None:
        CAMERA_PORT = args.port
    if args.local:
        print("Running main.py locally...")
        STATION_CAMERA_DEV = args.station
        FINAL_CAMERA_DEV = args.station
    else :
        STATION_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/{args.station}"
        FINAL_CAMERA_DEV = f"http://{CAMERA_HOST}:{CAMERA_PORT}/cam/{args.station}"

    app = QApplication()
    app.setStyleSheet(APP_QSS)
    ui = OvercookedIRLApp()
    ui.run()

    sys.exit(app.exec())
