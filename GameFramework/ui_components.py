import os

from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QPushButton, QSizePolicy
)
from PySide6.QtCore import Qt, QRect
from PySide6.QtGui import QPainter, QPixmap, QColor, QFont, QBrush, QPen

from station import Station
from config import ASSET_MAP, TABLE_CM

_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def fmt_mmss(seconds: int) -> str:
    seconds = max(0, int(seconds))
    m = seconds // 60
    s = seconds % 60
    return f"{m}:{s:02d}"


class StartPage(QWidget):
    def __init__(self, on_start_clicked):
        super().__init__()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(32, 32, 32, 32)
        outer.setSpacing(28)

        title = QLabel("OvercookedIRL")
        title.setObjectName("Title")
        title.setStyleSheet("font-size: 140px; font-weight: 900;")
        outer.addWidget(title)

        subtitle = QLabel("Press Start to begin a 2-minute round.")
        subtitle.setObjectName("Subtitle")
        subtitle.setStyleSheet("font-size: 56px; font-weight: 700;")
        outer.addWidget(subtitle)

        outer.addStretch(1)

        btn = QPushButton("START ROUND")
        btn.clicked.connect(on_start_clicked)
        btn.setStyleSheet("font-size: 52px; font-weight: 900; padding: 22px 32px;")
        btn.setFixedHeight(140)
        outer.addWidget(btn)


class EndPage(QWidget):
    def __init__(self, on_restart_clicked):
        super().__init__()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(32, 32, 32, 32)
        outer.setSpacing(28)

        title = QLabel("TIME’S UP!")
        title.setObjectName("Title")
        title.setStyleSheet("font-size: 140px; font-weight: 900;")
        outer.addWidget(title)

        score_card = QWidget()
        score_card.setObjectName("Card")
        score_card.setAttribute(Qt.WA_StyledBackground, True)

        score_layout = QVBoxLayout(score_card)
        score_layout.setContentsMargins(28, 28, 28, 28)
        score_layout.setSpacing(10)

        score_label = QLabel("FINAL SCORE")
        score_label.setObjectName("HudLabel")
        score_label.setStyleSheet(
            "font-size: 44px; font-weight: 900; letter-spacing: 2px;"
        )

        self.final_score_value = QLabel("0")
        self.final_score_value.setObjectName("HudValue")
        self.final_score_value.setStyleSheet("font-size: 180px; font-weight: 900;")

        score_layout.addWidget(score_label)
        score_layout.addWidget(self.final_score_value)

        outer.addWidget(score_card)
        outer.addStretch(1)

        btn = QPushButton("PLAY AGAIN")
        btn.clicked.connect(on_restart_clicked)
        btn.setStyleSheet("font-size: 52px; font-weight: 900; padding: 22px 32px;")
        btn.setFixedHeight(140)
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
        layout.setContentsMargins(28, 28, 28, 28)
        layout.setSpacing(24)

        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)

        title = QLabel(f"Station {station_type}")
        title.setStyleSheet("font-size: 32px; font-weight: 900;")

        self.badge = QLabel("READY")
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.setObjectName("BadgeReady")
        self.badge.setStyleSheet(
            "font-size: 40px; font-weight: 900; padding: 16px 28px;"
        )

        top_row.addWidget(title)
        top_row.addStretch(1)
        top_row.addWidget(self.badge)

        self.tag_label = QLabel("")
        self.tag_label.setObjectName("HudLabel")
        self.tag_label.setStyleSheet("font-size: 44px; font-weight: 900;")

        self.listed_tags = QLabel("Detected tags: ")
        self.listed_tags.setObjectName("Listed_Tags")
        self.listed_tags.setStyleSheet("font-size: 44px; font-weight: 900;")


        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.bar.setValue(0)
        self.bar.setTextVisible(True)
        self.bar.setFormat("SCANNING %p%")
        self.bar.setStyleSheet(
            "font-size: 36px; font-weight: 900; height: 48px;"
        )
        self.bar.hide()

        layout.addLayout(top_row)
        layout.addWidget(self.tag_label)
        layout.addWidget(self.bar)
        layout.addWidget(self.listed_tags)
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

        if target is not None and item_handler.has_item(target):
            item = item_handler.get_item(target)
            self.tag_label.setText(f"ITEM STAGE: {item.state}")
        else:
            self.tag_label.setText("")

        if state == Station.SCANNING:
            self.badge.setText("SCANNING")
            self.badge.setObjectName("BadgeScan")
            self.bar.show()
            p = 0 if progress is None else int(
                max(0.0, min(1.0, float(progress))) * 100
            )
            self.bar.setValue(p)
        elif state == Station.READY:
            self.badge.setText("READY")
            self.badge.setObjectName("BadgeReady")
            self.bar.hide()
            self.bar.setValue(0)
        else:
            self.badge.setText("UNKNOWN")
            self.badge.setStyleSheet(
                "font-size: 40px; font-weight: 900; color: #a9b1c3; padding: 16px 28px;"
            )
            self.bar.hide()
            self.bar.setValue(0)

        self.badge.style().unpolish(self.badge)
        self.badge.style().polish(self.badge)

        self.listed_tags.setText(f"Detected tags: {status['ids']}")
        self.listed_tags.show()


# ---------------------------------------------------------------------------
# Spatial table view: items drawn where their tags physically are
# ---------------------------------------------------------------------------
_base_pixmap_cache: dict[int, QPixmap | None] = {}


def _load_base_pixmap(tag_id: int) -> QPixmap | None:
    """Load (and cache) the full-res image for a tag id, or None if missing."""
    if tag_id in _base_pixmap_cache:
        return _base_pixmap_cache[tag_id]

    pixmap = None
    name = ASSET_MAP.get(tag_id)
    if name:
        path = os.path.join(_ASSETS_DIR, name)
        if os.path.exists(path):
            loaded = QPixmap(path)
            if not loaded.isNull():
                pixmap = loaded

    _base_pixmap_cache[tag_id] = pixmap  # cache the miss too
    return pixmap


def _make_placeholder(tag_id: int, size: int) -> QPixmap:
    """A drawn stand-in (warm circle with the id) used when art is missing."""
    px = QPixmap(size, size)
    px.fill(Qt.transparent)
    p = QPainter(px)
    p.setRenderHint(QPainter.Antialiasing)
    p.setBrush(QBrush(QColor("#e0a458")))
    p.setPen(QPen(QColor("#7a4f12"), 2))
    p.drawEllipse(2, 2, size - 4, size - 4)
    p.setPen(QColor("#2a1c08"))
    f = QFont()
    f.setBold(True)
    f.setPointSize(max(8, size // 4))
    p.setFont(f)
    p.drawText(px.rect(), Qt.AlignCenter, str(tag_id))
    p.end()
    return px


class TagIcon(QWidget):
    """A single item on the table: its image, with a thin scan bar underneath."""

    def __init__(self, tag_id: int, parent=None):
        super().__init__(parent)
        self.tag_id = tag_id

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)

        self.img = QLabel()
        self.img.setAlignment(Qt.AlignCenter)

        self.bar = QProgressBar()
        self.bar.setObjectName("ScanBar")
        self.bar.setRange(0, 100)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(8)
        self.bar.hide()

        lay.addWidget(self.img)
        lay.addWidget(self.bar)

        self.img_size = 0
        # normalized position on the table (0..1), set by TableView
        self.nx = 0.5
        self.ny = 0.5

    def set_size(self, img_size: int):
        if img_size == self.img_size:
            return
        self.img_size = img_size
        self.img.setFixedSize(img_size, img_size)
        self.bar.setFixedWidth(img_size)

        base = _load_base_pixmap(self.tag_id)
        if base is None:
            pm = _make_placeholder(self.tag_id, img_size)
        else:
            pm = base.scaled(
                img_size, img_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        self.img.setPixmap(pm)
        self.adjustSize()

    def set_progress(self, progress):
        if progress is None:
            self.bar.hide()
        else:
            self.bar.show()
            self.bar.setValue(int(max(0.0, min(1.0, float(progress))) * 100))


class TableView(QWidget):
    """
    Draws the physical table to scale and places item icons where their tags
    actually are. Positions arrive as normalized (0..1) table coordinates, so
    this widget never needs to know about cameras or pixels.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("TableView")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._aspect = TABLE_CM[0] / TABLE_CM[1]
        self._icons: dict[int, TagIcon] = {}
        self._table_rect = QRect()

    def _compute_table_rect(self) -> QRect:
        """Largest rectangle matching the real table's aspect, centered."""
        margin = 18
        avail_w = max(1, self.width() - 2 * margin)
        avail_h = max(1, self.height() - 2 * margin)

        if avail_w / avail_h > self._aspect:
            th = avail_h
            tw = int(th * self._aspect)
        else:
            tw = avail_w
            th = int(tw / self._aspect)

        tx = (self.width() - tw) // 2
        ty = (self.height() - th) // 2
        return QRect(tx, ty, tw, th)

    def paintEvent(self, event):
        self._table_rect = self._compute_table_rect()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(QColor("#e8dcc6")))   # warm countertop
        p.setPen(QPen(QColor("#c9b79a"), 3))
        p.drawRoundedRect(self._table_rect, 14, 14)
        p.end()
        self._reposition()

    def update_tags(self, render_list: list[dict]):
        """
        render_list: [{"id": int, "nx": float, "ny": float, "progress": float|None}]
        Creates/updates/removes icons to match exactly what's on the table.
        """
        self._table_rect = self._compute_table_rect()
        icon_size = max(28, int(self._table_rect.width() * 0.09))

        seen = set()
        for entry in render_list:
            tid = entry["id"]
            seen.add(tid)

            icon = self._icons.get(tid)
            if icon is None:
                icon = TagIcon(tid, self)
                self._icons[tid] = icon
                icon.show()

            icon.set_size(icon_size)
            icon.set_progress(entry.get("progress"))
            icon.nx = entry["nx"]
            icon.ny = entry["ny"]

        for tid in list(self._icons.keys()):
            if tid not in seen:
                self._icons[tid].deleteLater()
                del self._icons[tid]

        self._reposition()

    def _reposition(self):
        r = self._table_rect
        for icon in self._icons.values():
            cx = r.x() + icon.nx * r.width()
            cy = r.y() + icon.ny * r.height()
            x = int(cx - icon.width() / 2)
            y = int(cy - icon.img_size / 2)  # center the image on the point
            icon.move(x, y)

    def reset(self):
        for icon in self._icons.values():
            icon.deleteLater()
        self._icons.clear()
        self.update()


class GamePage(QWidget):
    def __init__(self, station_types: list[str], placement_map: dict[str, tuple[int, int, int, int]]):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(20)

        # HUD
        hud = QWidget()
        hud.setObjectName("Card")
        hud.setAttribute(Qt.WA_StyledBackground, True)
        hud.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        hud_layout = QHBoxLayout(hud)
        hud_layout.setContentsMargins(28, 24, 28, 24)
        hud_layout.setSpacing(64)

        # points
        points_block = QWidget()
        points_layout = QVBoxLayout(points_block)
        points_layout.setSpacing(6)

        points_label = QLabel("POINTS")
        points_label.setStyleSheet(
            "font-size: 44px; font-weight: 900; letter-spacing: 2px;"
        )

        self.points_value = QLabel("0")
        self.points_value.setStyleSheet("font-size: 120px; font-weight: 900;")

        points_layout.addWidget(points_label)
        points_layout.addWidget(self.points_value)

        # time
        time_block = QWidget()
        time_layout = QVBoxLayout(time_block)
        time_layout.setSpacing(6)

        time_label = QLabel("TIME LEFT")
        time_label.setStyleSheet(
            "font-size: 44px; font-weight: 900; letter-spacing: 2px;"
        )

        self.time_value = QLabel("2:00")
        self.time_value.setStyleSheet("font-size: 120px; font-weight: 900;")

        time_layout.addWidget(time_label)
        time_layout.addWidget(self.time_value)

        hud_layout.addWidget(points_block)
        hud_layout.addStretch(1)
        hud_layout.addWidget(time_block)

        root.addWidget(hud)

        # spatial table view (main play area) — items drawn where their tags are
        self.table_view = TableView()
        self.table_view.setMinimumHeight(260)  # never let the cards squeeze it to nothing
        root.addWidget(self.table_view, 3)

        # station grid (status strip beneath the table)
        grid = QGridLayout()
        grid.setHorizontalSpacing(28)
        grid.setVerticalSpacing(28)

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
        self.table_view.reset()

    def update_tags(self, render_list: list[dict]):
        self.table_view.update_tags(render_list)
