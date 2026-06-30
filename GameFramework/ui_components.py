import os

from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QProgressBar, QPushButton, QSizePolicy, QGraphicsOpacityEffect
)
from PySide6.QtCore import Qt, QRect, QTimer, QVariantAnimation, QEasingCurve
from PySide6.QtGui import QPainter, QPixmap, QColor, QFont, QBrush, QPen, QConicalGradient

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


class _ItemImage(QLabel):
    """Item picture that paints a colored 'destination' ring on top of itself.

    ring_color is None (no ring), a "#rrggbb" hex string (solid ring), or the
    sentinel "rainbow" (conical-gradient ring, used for the delivery station).
    """

    _RAINBOW = ["#ff0000", "#ff9900", "#ffee00", "#33cc33",
                "#3399ff", "#9933ff", "#ff0000"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ring_color = None
        self._ready = False
        self._pulse_alpha = 1.0
        self._pulse = QVariantAnimation(self)
        self._pulse.setKeyValueAt(0.0, 0.3)
        self._pulse.setKeyValueAt(0.5, 1.0)
        self._pulse.setKeyValueAt(1.0, 0.3)
        self._pulse.setDuration(900)
        self._pulse.setLoopCount(-1)
        self._pulse.setEasingCurve(QEasingCurve.InOutSine)
        self._pulse.valueChanged.connect(self._on_pulse)

    def _on_pulse(self, v):
        self._pulse_alpha = float(v)
        self.update()

    def set_ready(self, ready):
        if ready == self._ready:
            return
        self._ready = ready
        if ready:
            self._pulse.start()
        else:
            self._pulse.stop()
            self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if self._ready:
            w = 5
            inset = w // 2 + 1
            rect = self.rect().adjusted(inset, inset, -inset, -inset)
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            c = QColor("#22c55e")
            c.setAlphaF(self._pulse_alpha)
            p.setPen(QPen(c, w))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(rect)
            p.end()
            return
        if not self.ring_color:
            return
        w = 4
        inset = w // 2 + 1
        rect = self.rect().adjusted(inset, inset, -inset, -inset)

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self.ring_color == "rainbow":
            grad = QConicalGradient(rect.center(), 0)
            n = len(self._RAINBOW)
            for i, c in enumerate(self._RAINBOW):
                grad.setColorAt(i / (n - 1), QColor(c))
            p.setPen(QPen(QBrush(grad), w))
        else:
            p.setPen(QPen(QColor(self.ring_color), w))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(rect)
        p.end()


class TagIcon(QWidget):
    """A single item on the table: its image, with a thin scan bar underneath."""

    def __init__(self, tag_id: int, parent=None):
        super().__init__(parent)
        self.tag_id = tag_id

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)

        self.img = _ItemImage()
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
        self._appearance = None  # (item_type, state, size, color) currently drawn
        # normalized position on the table (0..1), set by TableView
        self.nx = 0.5
        self.ny = 0.5

    def set_appearance(self, item_type, state, img_size: int, color=None):
        """Pick the image from the item's (type, stage); redraw only on change."""
        key = (item_type, state, img_size, color)
        if key == self._appearance:
            return
        self._appearance = key
        self.img_size = img_size
        self.img.setFixedSize(img_size, img_size)
        self.bar.setFixedWidth(img_size)

        # Colored ring = the station this item should be taken to next.
        self.img.ring_color = color

        base = _load_base_pixmap(item_type, state)
        if base is None:
            pm = _make_placeholder(self.tag_id, img_size)
        else:
            pm = base.scaled(
                img_size, img_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        self.img.setPixmap(pm)
        self.adjustSize()

    def set_ready(self, ready):
        self.img.set_ready(ready)

    def set_progress(self, progress, burning=False, combining=False):
        if progress is None:
            self.bar.hide()
            return
        self.bar.show()
        p = max(0.0, min(1.0, float(progress)))
        self.bar.setValue(int((1.0 - p) * 100) if burning else int(p * 100))
        if self.bar.property("burning") != burning or self.bar.property("combining") != combining:
            self.bar.setProperty("burning", burning)
            self.bar.setProperty("combining", combining)
            self.bar.style().unpolish(self.bar)
            self.bar.style().polish(self.bar)


class StationZone(QWidget):
    """
    A station drawn as a region on the table (not a separate card). Shows the
    station name, a READY/SCANNING pill, and a corner line with the detected
    tags + current item stage.
    """

    def __init__(self, station_name: str, parent=None):
        super().__init__(parent)
        self.station_name = station_name
        self.setObjectName("StationZone")
        self.setAttribute(Qt.WA_StyledBackground, True)

        self.zone_color = None      # signature station colour (set by TableView)
        self._scanning = False
        self._reset_flash = False

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(4)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)

        self.name = QLabel(f"{station_name}")
        self.name.setObjectName("ZoneName")

        # Player-presence glyph for gated stations (2a/2b). Hidden for stations
        # that don't require a player; dimmed when the player has stepped away.
        self.player_glyph = QLabel("\N{STANDING PERSON}")
        self.player_glyph.setObjectName("PlayerGlyph")
        self.player_glyph.setAlignment(Qt.AlignCenter)
        self._glyph_opacity = QGraphicsOpacityEffect(self.player_glyph)
        self.player_glyph.setGraphicsEffect(self._glyph_opacity)
        self.player_glyph.hide()

        self.pill = QLabel("READY")
        self.pill.setObjectName("PillReady")
        self.pill.setAlignment(Qt.AlignCenter)

        top.addWidget(self.name)
        top.addStretch(1)
        top.addWidget(self.player_glyph)
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
        self._restyle()

    @staticmethod
    def _rgba(hex_color: str, alpha: float) -> str:
        h = hex_color.lstrip("#")
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return f"rgba({r}, {g}, {b}, {alpha})"

    def set_zone_color(self, color):
        self.zone_color = color
        self._restyle()

    def _restyle(self):
        """Tint the zone with its station colour, emphasised by scan/reset state."""
        # Zones only ever take a solid hex; fall back to neutral for anything else.
        color = self.zone_color if str(self.zone_color or "").startswith("#") else "#c4cdda"
        if self._reset_flash:
            border, bg = "2px solid #dc2626", self._rgba("#dc2626", 0.18)
        elif self._scanning:
            border, bg = f"2px solid {color}", self._rgba(color, 0.30)
        else:
            border, bg = f"1px solid {color}", self._rgba(color, 0.12)
        self.setStyleSheet(
            f"QWidget#StationZone {{ background: {bg}; border: {border}; border-radius: 12px; }}"
        )

    def _set_pill(self, scanning: bool, gated: bool, present: bool):
        # "STEP IN" takes priority: a gated station with no player can't scan.
        if gated and not present:
            new_obj, text = "PillNoPlayer", "STEP IN"
        elif scanning:
            new_obj, text = "PillScan", "SCANNING"
        else:
            new_obj, text = "PillReady", "READY"

        if self.pill.objectName() != new_obj:
            self.pill.setObjectName(new_obj)
            self.pill.setText(text)
            self.pill.style().unpolish(self.pill)
            self.pill.style().polish(self.pill)

        if self._scanning != scanning:
            self._scanning = scanning
            self._restyle()

    def _set_player_glyph(self, gated: bool, present: bool):
        if not gated:
            self.player_glyph.hide()
            return
        self.player_glyph.show()
        # Solid when a player is standing here; faded when they've stepped away.
        self._glyph_opacity.setOpacity(1.0 if present else 0.25)

    def _flash_reset(self):
        """Briefly outline the zone red to show a scan was lost to leaving."""
        self._reset_flash = True
        self._restyle()
        QTimer.singleShot(600, self._clear_reset_flash)

    def _clear_reset_flash(self):
        self._reset_flash = False
        self._restyle()

    def update_status(self, status: dict, item_handler):
        scans = status.get("scans") or {}
        gated = status.get("gated", False)
        present = status.get("player_present", True)

        self._set_pill(len(scans) > 0, gated, present)
        self._set_player_glyph(gated, present)

        if status.get("reset_by_player"):
            self._flash_reset()

        lines = []
        if scans:
            lines.append(f"Scanning: {len(scans)}")
        ids = status.get("ids")
        if ids:
            lines.append(f"Tags: {list(ids)}")
    #For Debugging purposes, will likely be removed later
        combine_ready = status.get("combine_ready")
        if combine_ready:
            lines.append(f"Ready to combine: {list(combine_ready.keys())}")
    #-----
        self.info.setText("\n".join(lines))

    def reset(self):
        self._set_pill(False, False, True)
        self._clear_reset_flash()
        self.player_glyph.hide()
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
            zone = StationZone(d["name"], self)
            zone.set_zone_color(d.get("color"))
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

            icon.set_appearance(entry.get("type"), entry.get("state"), icon_size, entry.get("color"))
            icon.set_progress(entry.get("progress"), entry.get("burning", False), entry.get("combining", False))
            icon.set_ready(entry.get("ready", False))
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
