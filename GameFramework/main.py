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
from order import OrderHandler
from station import Station
from aruco_tag_detector import ArucoTagDetector

from style import APP_QSS
from config import CAMERA_HOST, CAMERA_PORT, STATION_CAMERA_DEV, FINAL_CAMERA_DEV, GAME_SECONDS, TICK_MS, STATION_DEFS, FINAL_STATION_DEF, TABLE_REGION, PLAYER_CAMS, PLAYER_ZONES, PLAYER_TAG_IDS, STAGE_COLORS
from ui_components import StartPage, GamePage, EndPage
from final_window import FinalStationWindow


pygame.mixer.init()
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


# how many seconds of "get ready" countdown after Start is clicked
PREGAME_SECONDS = 5

class OvercookedIRLApp:
    def __init__(self, debug=False, show_final_window=False, playerless=False):
        self.debug = debug
        self.stack = QStackedWidget()
        self.stack.setWindowTitle("OvercookedIRL")

        self.points = 0
        self.time_left = GAME_SECONDS

        self.station_feed_relay = FeedRelay(STATION_CAMERA_DEV)
        self.final_feed_relay = FeedRelay(FINAL_CAMERA_DEV)
        self.item_handler = ItemHandler()
        self.order_handler = OrderHandler(DEBUG=self.debug)

        # One presence camera per gated station. If a feed won't open, that
        # zone falls back to "always present" so a bad camera can't brick play.
        self.player_feeds: dict[str, FeedRelay] = {}
        for zone_key, url in PLAYER_CAMS.items():
            if playerless:
                print("Configuring in playerless mode -- stations treat players as always present")
                break
            try:
                self.player_feeds[zone_key] = FeedRelay(url)
            except RuntimeError as e:
                print(f"\033[31m[WARN]\033[0m Player camera for zone '{zone_key}' unavailable "
                      f"({e}); treating it as always-present.")

        self.stations: list[Station] = []
        for d in STATION_DEFS:
            station_name = d["name"]
            if self.debug: print(f"Creating {station_name}...")
            self.stations.append(
                Station(
                    name=station_name,
                    x=d["x"], y=d["y"], w=d["w"], h=d["h"],
                    scan_time=d["scan_time"],
                    burn_time=d.get("burn_time", d["scan_time"]),
                    type=d["type"],
                    burn_type=d["burn_type"],
                    combinable=d["combinable"],
                    item_handler=self.item_handler,
                    player_zone=d.get("player_zone"),
                    cook_one=d.get("cook_one"),
                    combine_both=d.get("combine_both"),
                    debug=self.debug
                )
            )
        self.final_station = FinalStation(
            self.final_feed_relay,
            self.item_handler,
            self.order_handler,
            FINAL_STATION_DEF,
        )

        # Optional second window mirroring the delivery station in real time.
        # Enabled by the --final-station CLI flag, or by show_window in config.
        self.final_window = (
            FinalStationWindow(FINAL_STATION_DEF)
            if (show_final_window or FINAL_STATION_DEF.get("show_window")) else None
        )

        self.detector = ArucoTagDetector("DICT_4X4_50")

        self.SCAN_EVERY = 3
        self._scan_tick = 0

        self.start_page = StartPage(self.start_game)
        self.game_page = GamePage()
        self.end_page = EndPage(self.go_to_start)

        self.stack.addWidget(self.start_page)
        self.stack.addWidget(self.game_page)
        self.stack.addWidget(self.end_page)
        self.stack.setCurrentWidget(self.start_page)

        self.tick_timer = QTimer(self.stack)
        self.tick_timer.timeout.connect(self._tick)

        self.countdown_timer = QTimer(self.stack)
        self.countdown_timer.timeout.connect(self._countdown_tick)

        self.pregame_timer = QTimer(self.stack)
        self.pregame_timer.timeout.connect(self._pregame_tick)
        self.pregame_left = 0

        self.game_page.set_points(self.points)
        self.game_page.set_time_left(self.time_left)

    def inc_points(self, inc: int):
        self.points += inc
        self.game_page.set_points(self.points)

        INC_POINTS_SOUND.play()

    def start_game(self):
        self.points = 0
        self.time_left = GAME_SECONDS

        self.item_handler.clear()
        self.order_handler.clear()
        for station in self.stations:
            station.reset()
        self.final_station.reset()
        if self.final_window:
            self.final_window.reset()
        self._scan_tick = 0

        self.game_page.set_points(self.points)
        self.game_page.set_time_left(self.time_left)
        self.game_page.reset_station_cards()

        self.stack.setCurrentWidget(self.game_page)

        # Frozen "get ready" countdown
        self.pregame_left = PREGAME_SECONDS
        self.game_page.show_countdown(self.pregame_left)
        self.pregame_timer.start(1000)

    def _pregame_tick(self):
        self.pregame_left -= 1
        if self.pregame_left > 0:
            self.game_page.show_countdown(self.pregame_left)
        else:
            self.pregame_timer.stop()
            self.game_page.hide_countdown()
            self._begin_round()

    def _begin_round(self):
        if self.debug:
            print("[GAME STARTED]")
        self.tick_timer.start(TICK_MS)
        self.countdown_timer.start(1000)
        self.order_handler.start_game()

    def end_game(self):
        self.tick_timer.stop()
        self.countdown_timer.stop()
        self.end_page.set_score(self.points)
        self.stack.setCurrentWidget(self.end_page)

    def go_to_start(self):
        self.tick_timer.stop()
        self.countdown_timer.stop()
        self.pregame_timer.stop()
        self.game_page.hide_countdown()
        self.stack.setCurrentWidget(self.start_page)

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
        for feed in self.player_feeds.values():
            try:
                feed.update_image()
            except RuntimeError:
                pass

        # Only run detection every SCAN_EVERY ticks.
        self._scan_tick = (self._scan_tick + 1) % self.SCAN_EVERY
        if self._scan_tick != 0:
            return

        frame = self.station_feed_relay.frame
        if frame is None:
            return

        tags = self._detect_tags(frame)

        for tag_id, _cx, _cy in tags:
            self.item_handler.create_item(tag_id)

        player_present = self._detect_player_presence()

        scan_progress: dict[int, float] = {}
        burning_map: dict[int, bool] = {}
        combining_map: dict[int, bool] = {}
        ready_set: set[int] = set()
        statuses: dict[str, dict] = {}
        for station in self.stations:
            ids = [tag_id for (tag_id, cx, cy) in tags if station.contains(cx, cy)]
            present = player_present.get(station.player_zone, True)
            status = station._tick(ids, present)
            statuses[station.type] = status
            scan_progress.update(status.get("scans", {}))
            burning_map.update(status.get("burning", {}))
            combining_map.update(status.get("combining", {}))
            ready_set.update(status.get("combine_ready", {}))

        final_status = self.final_station._tick()
        delivered = final_status.get("delivered", [])
        for tag in delivered:
            self.inc_points(10)

        if self.final_window:
            self.final_window.update_view(final_status, self.item_handler)


        self.game_page.update_stations(statuses, self.item_handler)
        self.game_page.update_tags(self._build_render_list(tags, scan_progress, burning_map, combining_map, ready_set))

        self.order_handler._tick()
        self.game_page.update_orders(self.order_handler.orders)

    def _detect_tags(self, frame) -> list[tuple[int, float, float]]:
        """Return (tag_id, center_x, center_y) for every tag in the frame."""
        corners, ids = self.detector.detect(frame)
        tags = []
        for c, tag_id in zip(corners, ids):
            pts = c.reshape(-1, 2)
            tags.append((tag_id, float(pts[:, 0].mean()), float(pts[:, 1].mean())))
        return tags

    @staticmethod
    def _in_zone(px: float, py: float, zone: dict) -> bool:
        return (zone["x"] <= px < zone["x"] + zone["w"]
                and zone["y"] <= py < zone["y"] + zone["h"])

    def _detect_player_presence(self) -> dict[str, bool]:
        """Per gated zone, is a player head-tag inside it? Fail-open if no frame yet."""
        present: dict[str, bool] = {}
        for zone_key, feed in self.player_feeds.items():
            frame = feed.frame
            zone = PLAYER_ZONES.get(zone_key)
            if frame is None or zone is None:
                present[zone_key] = True
                continue
            tags = self._detect_tags(frame)
            present[zone_key] = any(
                tag_id in PLAYER_TAG_IDS and self._in_zone(cx, cy, zone)
                for tag_id, cx, cy in tags
            )
        return present

    def _build_render_list(self, tags, scan_progress: dict[int, float],
                           burning_map: dict[int, bool],
                           combining_map: dict[int, bool],
                           ready_set: set[int]) -> list[dict]:
        """Map detected tags to normalized table positions (+ type/stage) for the UI."""
        tx, ty, tw, th = TABLE_REGION

        render_list = []
        for tag_id, cx, cy in tags:
            nx = (cx - tx) / tw
            ny = (cy - ty) / th
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
                "color": STAGE_COLORS.get(item_state),
                "burning": burning_map.get(tag_id, False),
                "combining": combining_map.get(tag_id, False),
                "ready": tag_id in ready_set,
            })
        return render_list

    def run(self):
        self.stack.setMinimumSize(520, 420)
        self.stack.resize(820, 560)
        self.stack.showMaximized()
        if self.final_window:
            self.final_window.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="OvercookedIRL")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print debug statements to terminal while the game runs"
    )
    parser.add_argument(
        "--final-station",
        action="store_true",
        help="Open the delivery-station window (overrides config's show_window).",
    )
    parser.add_argument(
        "--playerless",
        action="store_true",
        help="All stations work regardless of player presence, useful for testing purposes"
    )
    parser.add_argument(
        "--miku",
        action="store_true",
        help="Replaces certain game sounds with Hatsune Miku"
    )
    args = parser.parse_args()

    if args.miku:
        INC_POINTS_SOUND = pygame.mixer.Sound(os.path.join(BASE_DIR, "assets", "miku-miku-beam-made-with-Voicemod.mp3"))
    else:
        INC_POINTS_SOUND = pygame.mixer.Sound(os.path.join(BASE_DIR, "assets", "inc_points.mp3"))

    app = QApplication()
    app.setStyleSheet(APP_QSS)
    ui = OvercookedIRLApp(debug=args.debug, show_final_window=args.final_station, playerless=args.playerless)
    ui.run()

    sys.exit(app.exec())
