from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QGroupBox
)
from PySide6.QtCore import QTimer
import sys

from feed_relay import FeedRelay
from station import Station

'''
    dict will map tag to its current state, the station can trigger advance state for a tag, probably should add a state machine to the stations
    if tag is not recognized, then add it to orders
'''


class OvercookedIRL:
    def __init__(self):
        # --- UI setup ---
        self.window = QWidget()
        self.window.setWindowTitle("OvercookedIRL")

        self.root_layout = QVBoxLayout(self.window)

        self.frame_label = QLabel("Frame: 0")
        self.root_layout.addWidget(self.frame_label)

        self.feed_relay = FeedRelay("/dev/video6")

        # --- stations (replace coords as needed) ---
        self.stations = [
            Station(75, 120, 158, 260, self.feed_relay, 5),
            Station(75 + 158, 120, 203, 135, self.feed_relay, 5),
            Station(75 + 158, 120 + 135, 203, 125, self.feed_relay, 5),
            Station(75 + 158 + 203, 120, 121, 260, self.feed_relay, 5),
        ]

        self.active_tags = {}

        # --- UI subsections for each station ---
        self.station_status_labels = []
        for i in range(4):
            box = QGroupBox(f"Station {i + 1}")
            box_layout = QVBoxLayout(box)

            status_label = QLabel("Status: (unknown)")
            box_layout.addWidget(status_label)

            self.station_status_labels.append(status_label)
            self.root_layout.addWidget(box)

        # --- loop state ---
        self.frame = 0

        # --- timer loop ---
        self.timer = QTimer(self.window)
        self.timer.timeout.connect(self._tick)

    def _tick(self):
        self.frame += 1
        self.frame_label.setText(f"Frame: {self.frame}")

        self.feed_relay.update_image()

        # Update each station's section
        for i, station in enumerate(self.stations):
            status = station.get_status()

            label = self.station_status_labels[i]
            label.setText(f"Status: {status}")

            if status:
                label.setStyleSheet("color: green;")
            else:
                label.setStyleSheet("color: red;")


    def enter_game_loop(self):
        self.window.show()
        self.timer.start(16)  # ~60 FPS


if __name__ == "__main__":
    app = QApplication(sys.argv)
    game_sess = OvercookedIRL()
    game_sess.enter_game_loop()
    sys.exit(app.exec())