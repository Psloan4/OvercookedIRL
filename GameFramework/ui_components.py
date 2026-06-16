import os

from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QProgressBar, QPushButton, QSizePolicy
)
from PySide6.QtCore import Qt, QRect
from PySide6.QtGui import QPainter, QPixmap, QColor, QFont, QBrush, QPen

from station import Station
from config import ASSET_MAP, TABLE_CM, TABLE_REGION, STATION_DEFS

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
_base_pixmap_cache: dict[tuple, QPixmap | None] = {}


def _load_base_pixmap(item_type, state) -> QPixmap | None:
    """Load (and cache) the image for an item's (type, stage), or None if missing."""
    key = (item_type, state)
    if key in _base_pixmap_cache:
        return _base_pixmap_cache[key]

    pixmap = None
    name = ASSET_MAP.get(item_type, {}).get(state) if item_type else None
    if name:
        path = os.path.join(_ASSETS_DIR, name)
        if os.path.exists(path):
            loaded = QPixmap(path)
            if not loaded.isNull():
                pixmap = loaded

    _base_pixmap_cache[key] = pixmap  # cache the miss too
    return pixmap


def _make_placeholder(tag_id: int, size: int) -> QPixmap:
    """A drawn stand-in (warm circle with the id) used when art is missing."""
    px = QPixmap(size, size)
    px.fill(Qt.transparent)
    p = QPainter(px)
    p.setRenderHint(QPainter.Antialiasing)
    p.setBrush(QBrush(QColor("#dbe4f0")))
    p.setPen(QPen(QColor("#2563eb"), 2))
    p.drawEllipse(2, 2, size - 4, size - 4)
    p.setPen(QColor("#1d4ed8"))
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
        self.img.setStyleSheet("background-color: rgba(255, 0, 0, 0)")
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
        self._appearance = None  # (item_type, state, size) currently drawn
        # normalized position on the table (0..1), set by TableView
        self.nx = 0.5
        self.ny = 0.5

    def set_appearance(self, item_type, state, img_size: int):
        """Pick the image from the item's (type, stage); redraw only on change."""
        key = (item_type, state, img_size)
        if key == self._appearance:
            return
        self._appearance = key
        self.img_size = img_size
        self.img.setFixedSize(img_size, img_size)
        self.bar.setFixedWidth(img_size)

        base = _load_base_pixmap(item_type, state)
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


class StationZone(QWidget):
    """
    A station drawn as a region on the table (not a separate card). Shows the
    station name, a READY/SCANNING pill, and a corner line with the detected
    tags + current item stage.
    """

    def __init__(self, station_type: str, parent=None):
        super().__init__(parent)
        self.station_type = station_type
        self.setObjectName("StationZone")
        self.setAttribute(Qt.WA_StyledBackground, True)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(4)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)

        self.name = QLabel(f"Station {station_type}")
        self.name.setObjectName("ZoneName")

        self.pill = QLabel("READY")
        self.pill.setObjectName("PillReady")
        self.pill.setAlignment(Qt.AlignCenter)

        top.addWidget(self.name)
        top.addStretch(1)
        top.addWidget(self.pill)

        # corner info (detected tags + item stage) pinned to the bottom-left
        self.info = QLabel("")
        self.info.setObjectName("ZoneInfo")
        self.info.setWordWrap(True)
        self.info.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        lay.addLayout(top)
        lay.addStretch(1)
        lay.addWidget(self.info)

        self.normalized_rect = (0.0, 0.0, 1.0, 1.0)  # set by TableView

    def _set_scanning(self, scanning: bool):
        new_obj = "PillScan" if scanning else "PillReady"
        if self.pill.objectName() != new_obj:
            self.pill.setObjectName(new_obj)
            self.pill.setText("SCANNING" if scanning else "READY")
            self.pill.style().unpolish(self.pill)
            self.pill.style().polish(self.pill)
        if self.property("scan") != scanning:
            self.setProperty("scan", scanning)
            self.style().unpolish(self)
            self.style().polish(self)

    def update_status(self, status: dict, item_handler):
        scans = status.get("scans") or {}
        self._set_scanning(len(scans) > 0)

        lines = []
        if scans:
            lines.append(f"Scanning: {len(scans)}")
        ids = status.get("ids")
        if ids:
            lines.append(f"Tags: {list(ids)}")
        self.info.setText("\n".join(lines))

    def reset(self):
        self._set_scanning(False)
        self.info.setText("")


class TableView(QWidget):
    """
    Draws the physical table to scale, splits it into the station zones (from
    STATION_DEFS), and places item icons where their tags actually are.
    Positions arrive as normalized (0..1) table coordinates, so this widget
    never needs to know about cameras or pixels.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("TableView")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._aspect = TABLE_CM[0] / TABLE_CM[1]
        self._icons: dict[int, TagIcon] = {}
        self._table_rect = QRect()

        # Station zones, positioned from STATION_DEFS mapped through TABLE_REGION
        # into normalized table coordinates (same space the tags map into).
        tx, ty, tw, th = TABLE_REGION
        self._zones: dict[str, StationZone] = {}
        for d in STATION_DEFS:
            zone = StationZone(d["type"], self)
            zone.normalized_rect = (
                (d["x"] - tx) / tw,
                (d["y"] - ty) / th,
                d["w"] / tw,
                d["h"] / th,
            )
            self._zones[d["type"]] = zone
            zone.show()

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
        p.setBrush(QBrush(QColor("#eef2f8")))   # light blue-gray tabletop
        p.setPen(QPen(QColor("#c4cdda"), 3))
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

            icon.set_appearance(entry.get("type"), entry.get("state"), icon_size)
            icon.set_progress(entry.get("progress"))
            icon.nx = entry["nx"]
            icon.ny = entry["ny"]
            icon.raise_()  # keep items above the station zones

        for tid in list(self._icons.keys()):
            if tid not in seen:
                self._icons[tid].deleteLater()
                del self._icons[tid]

        self._reposition()

    def update_stations(self, statuses: dict[str, dict], item_handler):
        """statuses: {station_type: status_dict} — drives each zone's pill/info."""
        for stype, zone in self._zones.items():
            if stype in statuses:
                zone.update_status(statuses[stype], item_handler)

    def _reposition(self):
        r = self._table_rect

        gap = 3
        for zone in self._zones.values():
            nx, ny, nw, nh = zone.normalized_rect
            zone.setGeometry(
                int(r.x() + nx * r.width()) + gap,
                int(r.y() + ny * r.height()) + gap,
                max(1, int(nw * r.width()) - 2 * gap),
                max(1, int(nh * r.height()) - 2 * gap),
            )

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
        for zone in self._zones.values():
            zone.reset()
        self.update()


class GamePage(QWidget):
    """One big table view: stations are zones on the table, items sit where
    their tags are, and points/time float in the corners."""

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)

        self.table_view = TableView()
        root.addWidget(self.table_view)

        # HUD: children of the page so they float over the table corners.
        self.points_block, self.points_value = self._make_hud_block("POINTS", "0")
        self.time_block, self.time_value = self._make_hud_block("TIME LEFT", "2:00")
        self.points_block.raise_()
        self.time_block.raise_()

    def _make_hud_block(self, label_text: str, value_text: str):
        block = QWidget(self)
        block.setObjectName("Card")
        block.setAttribute(Qt.WA_StyledBackground, True)

        # Horizontal layout => a wide, short card (label beside the value).
        lay = QHBoxLayout(block)
        lay.setContentsMargins(34, 12, 34, 12)
        lay.setSpacing(24)

        lbl = QLabel(label_text)
        lbl.setObjectName("HudLabel")
        lbl.setStyleSheet("font-size: 40px; font-weight: 900; letter-spacing: 2px;")

        val = QLabel(value_text)
        val.setStyleSheet("font-size: 76px; font-weight: 900;")

        lay.addWidget(lbl)
        lay.addWidget(val, 0, Qt.AlignVCenter)
        return block, val

    def resizeEvent(self, event):
        super().resizeEvent(event)
        m = 28
        self.points_block.adjustSize()
        self.time_block.adjustSize()
        # points bottom-left, time bottom-right
        self.points_block.move(m, self.height() - self.points_block.height() - m)
        self.time_block.move(
            self.width() - self.time_block.width() - m,
            self.height() - self.time_block.height() - m,
        )

    def set_points(self, points: int):
        self.points_value.setText(str(points))

    def set_time_left(self, seconds: int):
        self.time_value.setText(fmt_mmss(seconds))

    def update_tags(self, render_list: list[dict]):
        self.table_view.update_tags(render_list)

    def update_stations(self, statuses: dict[str, dict], item_handler):
        self.table_view.update_stations(statuses, item_handler)

    def reset_station_cards(self):
        self.table_view.reset()
