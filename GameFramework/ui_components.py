from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QPushButton, QSizePolicy
)
from PySide6.QtCore import Qt

from station import Station


def fmt_mmss(seconds: int) -> str:
    seconds = max(0, int(seconds))
    m = seconds // 60
    s = seconds % 60
    return f"{m}:{s:02d}"


class StartPage(QWidget):
    def __init__(self, on_start_clicked):
        super().__init__()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 24, 24, 24)
        outer.setSpacing(18)

        title = QLabel("OvercookedIRL")
        title.setObjectName("Title")
        outer.addWidget(title)

        subtitle = QLabel("Press Start to begin a 2-minute round.")
        subtitle.setObjectName("Subtitle")
        outer.addWidget(subtitle)

        outer.addStretch(1)

        btn = QPushButton("Start Round")
        btn.clicked.connect(on_start_clicked)
        btn.setFixedHeight(44)
        outer.addWidget(btn)


class EndPage(QWidget):
    def __init__(self, on_restart_clicked):
        super().__init__()
        outer = QVBoxLayout(self)
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

        btn = QPushButton("Play Again")
        btn.clicked.connect(on_restart_clicked)
        btn.setFixedHeight(44)
        outer.addWidget(btn)

    def set_score(self, points: int):
        self.final_score_value.setText(str(points))


class StationCard(QWidget):
    def __init__(self, station_type: str):
        super().__init__()
        self.setObjectName("Card")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(18)

        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)

        title = QLabel(f"Station {station_type}")
        title.setStyleSheet("font-size: 20px; font-weight: 900;")

        self.badge = QLabel("READY")
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.setObjectName("BadgeReady")

        top_row.addWidget(title)
        top_row.addStretch(1)
        top_row.addWidget(self.badge)

        self.tag_label = QLabel("")
        self.tag_label.setObjectName("HudLabel")
        self.tag_label.setStyleSheet("font-size: 16px; font-weight: 900;")

        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.bar.setValue(0)
        self.bar.setTextVisible(True)
        self.bar.setFormat("SCANNING %p%")
        self.bar.hide()

        layout.addLayout(top_row)
        layout.addWidget(self.tag_label)
        layout.addWidget(self.bar)
        layout.addStretch(1)

    def reset_ui(self):
        self.badge.setText("READY")
        self.badge.setObjectName("BadgeReady")
        self.badge.style().unpolish(self.badge)
        self.badge.style().polish(self.badge)
        self.bar.hide()
        self.bar.setValue(0)
        self.tag_label.setText("")

    def update_from_status(self, status: dict, item_handler):
        state = status.get("state", "unknown")
        progress = status.get("progress", None)
        target = status.get("target", None)

        # Show the selected tag's ITEM STATE (stage), not the tag id.
        if target is not None and item_handler.has_item(target):
            item = item_handler.get_item(target)
            self.tag_label.setText(f"Item detected: stage {item.state}")
        else:
            self.tag_label.setText("")

        if state == Station.SCANNING:
            self.badge.setText("SCANNING")
            self.badge.setObjectName("BadgeScan")
            self.bar.show()
            p = 0 if progress is None else int(max(0.0, min(1.0, float(progress))) * 100)
            self.bar.setValue(p)
        elif state == Station.READY:
            self.badge.setText("READY")
            self.badge.setObjectName("BadgeReady")
            self.bar.hide()
            self.bar.setValue(0)
        else:
            self.badge.setText("UNKNOWN")
            self.badge.setStyleSheet("color: #a9b1c3;")
            self.bar.hide()
            self.bar.setValue(0)

        self.badge.style().unpolish(self.badge)
        self.badge.style().polish(self.badge)



class GamePage(QWidget):
    def __init__(self, station_types: list[str], placement_map: dict[str, tuple[int, int, int, int]]):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(14)

        # HUD
        hud = QWidget()
        hud.setObjectName("Card")
        hud.setAttribute(Qt.WA_StyledBackground, True)
        hud.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        hud_layout = QHBoxLayout(hud)
        hud_layout.setContentsMargins(20, 18, 20, 18)
        hud_layout.setSpacing(32)

        # points
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

        # time
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

        # station grid
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(18)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)

        self.cards_by_type: dict[str, StationCard] = {}
        for stype in station_types:
            card = StationCard(stype)
            self.cards_by_type[stype] = card
            r, c, rs, cs = placement_map[stype]
            grid.addWidget(card, r, c, rs, cs)

        root.addLayout(grid, 1)

    def set_points(self, points: int):
        self.points_value.setText(str(points))

    def set_time_left(self, seconds: int):
        self.time_value.setText(fmt_mmss(seconds))

    def reset_station_cards(self):
        for card in self.cards_by_type.values():
            card.reset_ui()
