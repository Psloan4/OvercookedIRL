from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QProgressBar,
    QPushButton, QStackedWidget, QHBoxLayout, QGridLayout, QSizePolicy
)
from PySide6.QtCore import QTimer, Qt
import sys

from feed_relay import FeedRelay
from item import ItemHandler
from station import Station


APP_QSS = """
QWidget {
    background: #e9edf5;
    color: #1f2937;
    font-family: Inter, Segoe UI, Arial;
    font-size: 16px;
}

QLabel#Title {
    font-size: 42px;
    font-weight: 900;
}

QLabel#Subtitle {
    color: #4b5563;
    font-size: 18px;
}

QPushButton {
    background: #2563eb;
    color: white;
    border: none;
    padding: 16px 20px;
    border-radius: 14px;
    font-size: 18px;
    font-weight: 800;
}
QPushButton:hover { background: #1d4ed8; }
QPushButton:pressed { background: #1e40af; }

QWidget#Card {
    background: #ffffff;
    border: 1px solid #dbe0ea;
    border-radius: 18px;
}

QLabel#HudLabel {
    color: #6b7280;
    font-size: 16px;
    font-weight: 700;
}

QLabel#HudValue {
    font-size: 36px;
    font-weight: 900;
}

QLabel#BadgeReady {
    background: #fdecea;
    color: #dc2626;
    border: 1px solid #f5c2c7;
    padding: 6px 14px;
    border-radius: 999px;
    font-size: 14px;
    font-weight: 900;
}

QLabel#BadgeScan {
    background: #e7f8ef;
    color: #059669;
    border: 1px solid #a7f3d0;
    padding: 6px 14px;
    border-radius: 999px;
    font-size: 14px;
    font-weight: 900;
}

QProgressBar {
    background: #eef1f7;
    border: 1px solid #dbe0ea;
    border-radius: 10px;
    height: 20px;
    text-align: center;
    color: #1f2937;
    font-size: 14px;
    font-weight: 700;
}

QProgressBar::chunk {
    background: #2563eb;
    border-radius: 9px;
}

QGroupBox {
    border: none;
}
"""


class OvercookedIRL:
    GAME_SECONDS = 120  # 2 minutes

    def __init__(self):
        self.stack = QStackedWidget()
        self.stack.setWindowTitle("OvercookedIRL")

        self.points = 0

        self.feed_relay = FeedRelay("/dev/video6")
        self.item_handler = ItemHandler(self.inc_points)

        # Stations
        self.stations = [
            Station(7, 79, 201, 331, self.feed_relay, 2, "1", self.item_handler, True),
            Station(7 + 201, 79, 263, 178, self.feed_relay, 2, "2a", self.item_handler, False),
            Station(7 + 201, 79 + 178, 263, 153, self.feed_relay, 2, "2b", self.item_handler, False),
            Station(7 + 201 + 263, 79, 164, 333, self.feed_relay, 2, "3", self.item_handler, True),
        ]

        self._build_start_page()
        self._build_game_page()
        self._build_end_page()

        self.frame = 0
        self.time_left = self.GAME_SECONDS

        self.timer = QTimer(self.stack)
        self.timer.timeout.connect(self._tick)

        self.countdown_timer = QTimer(self.stack)
        self.countdown_timer.timeout.connect(self._countdown_tick)

        self.stack.setCurrentWidget(self.start_page)

    # -------------------- Helpers --------------------

    def _update_points_label(self):
        if hasattr(self, "points_value"):
            self.points_value.setText(str(self.points))

    def inc_points(self, inc):
        self.points += inc
        self._update_points_label()

    # -------------------- UI pages --------------------

    def _build_start_page(self):
        self.start_page = QWidget()
        outer = QVBoxLayout(self.start_page)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setSpacing(18)

        title = QLabel("OvercookedIRL")
        title.setObjectName("Title")
        outer.addWidget(title)

        subtitle = QLabel("Press Start to begin a 2-minute round.")
        subtitle.setObjectName("Subtitle")
        outer.addWidget(subtitle)

        outer.addStretch(1)

        self.start_button = QPushButton("Start Round")
        self.start_button.clicked.connect(self.start_game)
        self.start_button.setFixedHeight(44)
        outer.addWidget(self.start_button)

        self.stack.addWidget(self.start_page)

    def _build_game_page(self):
        self.game_page = QWidget()
        root = QVBoxLayout(self.game_page)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(14)

        # --- HUD row (white card) ---
        hud = QWidget()
        hud.setObjectName("Card")
        hud.setAttribute(Qt.WA_StyledBackground, True)
        hud.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        hud_layout = QHBoxLayout(hud)
        hud_layout.setContentsMargins(20, 18, 20, 18)
        hud_layout.setSpacing(32)

        # Points block
        points_block = QWidget()
        points_layout = QVBoxLayout(points_block)
        points_layout.setContentsMargins(0, 0, 0, 0)
        points_layout.setSpacing(2)

        points_label = QLabel("POINTS")
        points_label.setObjectName("HudLabel")
        self.points_value = QLabel("0")
        self.points_value.setObjectName("HudValue")
        points_layout.addWidget(points_label)
        points_layout.addWidget(self.points_value)

        # Time block
        time_block = QWidget()
        time_layout = QVBoxLayout(time_block)
        time_layout.setContentsMargins(0, 0, 0, 0)
        time_layout.setSpacing(2)

        time_label = QLabel("TIME LEFT")
        time_label.setObjectName("HudLabel")
        self.time_value = QLabel("2:00")
        self.time_value.setObjectName("HudValue")
        time_layout.addWidget(time_label)
        time_layout.addWidget(self.time_value)

        hud_layout.addWidget(points_block)
        hud_layout.addStretch(1)
        hud_layout.addWidget(time_block)

        root.addWidget(hud)

        # --- Station cards layout ---
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(18)

        # 3 columns: left (1), middle (2a/2b), right (3)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)

        # 2 rows
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)

        self.station_badges = []
        self.station_progress_bars = []

        for station in self.stations:
            card = QWidget()
            card.setObjectName("Card")
            card.setAttribute(Qt.WA_StyledBackground, True)
            card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(20, 20, 20, 20)
            card_layout.setSpacing(18)

            top_row = QHBoxLayout()
            top_row.setContentsMargins(0, 0, 0, 0)

            title = QLabel(f"Station {station.type}")
            title.setStyleSheet("font-size: 20px; font-weight: 900;")

            badge = QLabel("READY")
            badge.setAlignment(Qt.AlignCenter)
            badge.setObjectName("BadgeReady")

            top_row.addWidget(title)
            top_row.addStretch(1)
            top_row.addWidget(badge)

            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(0)
            bar.setTextVisible(True)
            bar.setFormat("SCANNING %p%")
            bar.hide()

            card_layout.addLayout(top_row)
            card_layout.addWidget(bar)
            card_layout.addStretch(1)  # helps fill tall cards nicely

            self.station_badges.append(badge)
            self.station_progress_bars.append(bar)

            # Place based on station.type
            if station.type == "1":
                grid.addWidget(card, 0, 0, 2, 1)   # spans rows
            elif station.type == "2a":
                grid.addWidget(card, 0, 1)
            elif station.type == "2b":
                grid.addWidget(card, 1, 1)
            elif station.type == "3":
                grid.addWidget(card, 0, 2, 2, 1)   # spans rows

        root.addLayout(grid, 1)  # stretch: grid grows with window

        self.stack.addWidget(self.game_page)

    def _build_end_page(self):
        self.end_page = QWidget()
        outer = QVBoxLayout(self.end_page)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setSpacing(18)

        title = QLabel("Time’s up!")
        title.setObjectName("Title")
        outer.addWidget(title)

        score_card = QWidget()
        score_card.setObjectName("Card")
        score_card.setAttribute(Qt.WA_StyledBackground, True)

        score_layout = QVBoxLayout(score_card)
        score_layout.setContentsMargins(18, 16, 18, 16)
        score_layout.setSpacing(6)

        score_label = QLabel("FINAL SCORE")
        score_label.setObjectName("HudLabel")

        self.final_score_value = QLabel("0")
        self.final_score_value.setObjectName("HudValue")
        self.final_score_value.setStyleSheet("font-size: 56px; font-weight: 900;")

        score_layout.addWidget(score_label)
        score_layout.addWidget(self.final_score_value)

        outer.addWidget(score_card)
        outer.addStretch(1)

        self.restart_button = QPushButton("Play Again")
        self.restart_button.clicked.connect(self.go_to_start)
        self.restart_button.setFixedHeight(44)
        outer.addWidget(self.restart_button)

        self.stack.addWidget(self.end_page)

    # -------------------- game flow --------------------

    def start_game(self):
        self.frame = 0
        self.points = 0
        self._update_points_label()

        self.time_left = self.GAME_SECONDS
        self._update_time_label()

        # reset station UI
        for badge, bar in zip(self.station_badges, self.station_progress_bars):
            badge.setText("READY")
            badge.setObjectName("BadgeReady")
            badge.style().unpolish(badge)
            badge.style().polish(badge)
            bar.hide()
            bar.setValue(0)

        self.stack.setCurrentWidget(self.game_page)
        self.timer.start(16)
        self.countdown_timer.start(1000)

    def end_game(self):
        self.timer.stop()
        self.countdown_timer.stop()

        self.final_score_value.setText(str(self.points))
        self.stack.setCurrentWidget(self.end_page)

    def go_to_start(self):
        self.timer.stop()
        self.countdown_timer.stop()
        self.stack.setCurrentWidget(self.start_page)

    # -------------------- timers --------------------

    def _countdown_tick(self):
        self.time_left -= 1
        self._update_time_label()
        if self.time_left <= 0:
            self.end_game()

    def _update_time_label(self):
        m = max(0, self.time_left) // 60
        s = max(0, self.time_left) % 60
        if hasattr(self, "time_value"):
            self.time_value.setText(f"{m}:{s:02d}")

    def _tick(self):
        self.frame += 1
        self.feed_relay.update_image()

        for i, station in enumerate(self.stations):
            status = station.get_status()
            state = status.get("state", "unknown")
            progress = status.get("progress", None)

            badge = self.station_badges[i]
            bar = self.station_progress_bars[i]

            if state == Station.SCANNING:
                badge.setText("SCANNING")
                badge.setObjectName("BadgeScan")
                bar.show()
                bar.setValue(int(max(0.0, min(1.0, float(progress))) * 100) if progress is not None else 0)
            elif state == Station.READY:
                badge.setText("READY")
                badge.setObjectName("BadgeReady")
                bar.hide()
                bar.setValue(0)
            else:
                badge.setText("UNKNOWN")
                badge.setStyleSheet("color: #a9b1c3;")
                bar.hide()
                bar.setValue(0)

            badge.style().unpolish(badge)
            badge.style().polish(badge)

    def run(self):
        self.stack.setMinimumSize(520, 420)
        self.stack.resize(820, 560)  # starting size; user can resize freely
        self.stack.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_QSS)

    game_sess = OvercookedIRL()
    game_sess.run()
    sys.exit(app.exec())
