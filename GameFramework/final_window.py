"""
Second, standalone window that mirrors the delivery (final) station in real
time.

Toggle it with FINAL_STATION_DEF["show_window"]
in config.py.
"""

from PySide6.QtWidgets import QWidget, QLabel, QGraphicsOpacityEffect
from PySide6.QtCore import Qt, QRect, QVariantAnimation, QEasingCurve
from PySide6.QtGui import QPainter, QColor, QBrush, QPen

from ui_components import TagIcon, _load_base_pixmap


class _DeliveryFlash(QLabel):
    """A big centered icon that pops in and fades out when an item is delivered."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)

        self._effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._effect)

        self._anim = QVariantAnimation(self)
        self._anim.setStartValue(1.0)
        self._anim.setEndValue(0.0)
        self._anim.setDuration(1100)
        self._anim.setEasingCurve(QEasingCurve.InQuad)
        self._anim.valueChanged.connect(lambda v: self._effect.setOpacity(float(v)))
        self._anim.finished.connect(self.hide)
        self.hide()

    def play(self, pixmap):
        if pixmap is None:
            return
        self.setPixmap(pixmap)
        self.show()
        self.raise_()
        self._anim.stop()
        self._anim.start()


class FinalStationWindow(QWidget):
    def __init__(self, station_def, parent=None, embedded=False):
        super().__init__(parent)
        self.embedded = embedded
        self.setWindowTitle("OvercookedIRL - Delivery")
        self.setObjectName("FinalStationWindow")

        if embedded:
            # Blend into the main window: transparent (the page shows through),
            # no counter box, and the counter runs vertically (rotated 90° CCW).
            self.setAttribute(Qt.WA_StyledBackground, True)
            self.setStyleSheet(
                "QWidget#FinalStationWindow { background: transparent; }"
            )
            self._aspect = station_def["h"] / station_def["w"]  # tall, not wide
            title_css = "color: #334155; font-size: 18px; font-weight: 900; letter-spacing: 3px;"
        else:
            # Standalone pop-out window keeps its dark theme + horizontal counter.
            self.setStyleSheet(
                "QWidget#FinalStationWindow { background-color: #0f172a; }"
            )
            self.resize(760, 320)
            self.setMinimumSize(360, 180)
            self._aspect = station_def["w"] / station_def["h"]
            title_css = "color: #e2e8f0; font-size: 26px; font-weight: 900; letter-spacing: 3px;"

        self._icons: dict[int, TagIcon] = {}
        self._counter_rect = QRect()

        self._title = QLabel("DELIVERY STATION", self)
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setStyleSheet(title_css)

        self._flash = _DeliveryFlash(self)

    def _compute_counter_rect(self) -> QRect:
        """Largest rect matching the station's aspect, below the title, centered."""
        margin = 24
        top = 56  # room for the title
        avail_w = max(1, self.width() - 2 * margin)
        avail_h = max(1, self.height() - top - margin)

        if avail_w / avail_h > self._aspect:
            h = avail_h
            w = int(h * self._aspect)
        else:
            w = avail_w
            h = int(w / self._aspect)

        x = (self.width() - w) // 2
        y = top + (avail_h - h) // 2
        return QRect(x, y, w, h)

    def paintEvent(self, event):
        self._counter_rect = self._compute_counter_rect()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self.embedded:
            # No backdrop or counter box -- items sit on the page surface, set
            # off from the table only by a faint divider line down the right.
            p.setPen(QPen(QColor("#cfd6e2"), 2))
            x = self.width() - 2
            p.drawLine(x, 24, x, self.height() - 24)
        else:
            p.setBrush(QBrush(QColor("#eef2f8")))   # light counter on the dark window
            p.setPen(QPen(QColor("#c4cdda"), 3))
            p.drawRoundedRect(self._counter_rect, 14, 14)
        p.end()

        self._title.setGeometry(0, 16, self.width(), 32)
        self._reposition()

    def update_view(self, final_status: dict, item_handler):
        """Called once per scan-tick with FinalStation._tick()'s output."""
        positions = final_status.get("positions", {})
        scans = final_status.get("scans", {})

        self._counter_rect = self._compute_counter_rect()
        # Size off the short side so the vertical (embedded) counter doesn't
        # produce enormous icons.
        short = min(self._counter_rect.width(), self._counter_rect.height())
        icon_size = max(28, int(short * 0.5))

        seen = set()
        for tag, (nx, ny) in positions.items():
            # A just-delivered tag is still in positions but already gone from
            # the handler; skip it (the flash covers it instead).
            if not item_handler.has_item(tag):
                continue
            item = item_handler.get_item(tag)
            seen.add(tag)

            icon = self._icons.get(tag)
            if icon is None:
                icon = TagIcon(tag, self)
                self._icons[tag] = icon
                icon.show()

            icon.set_appearance(item.type, item.state, icon_size, None)
            icon.set_progress(scans.get(tag))
            icon.nx = nx
            icon.ny = ny
            icon.raise_()

        for tag in list(self._icons.keys()):
            if tag not in seen:
                self._icons[tag].deleteLater()
                del self._icons[tag]

        for d in final_status.get("delivered_items", []):
            pm = _load_base_pixmap(d.get("type"), d.get("state"))
            if pm is not None:
                size = int(short * 0.9)
                pm = pm.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self._flash.play(pm)

        self._reposition()

    def _orient(self, nx: float, ny: float) -> tuple[float, float]:
        """Station-space (0..1) coords -> display coords. The embedded view is
        rotated 90° counter-clockwise so the counter runs vertically."""
        if self.embedded:
            return ny, 1.0 - nx
        return nx, ny

    def _reposition(self):
        r = self._counter_rect
        for icon in self._icons.values():
            nx, ny = self._orient(icon.nx, icon.ny)
            cx = r.x() + nx * r.width()
            cy = r.y() + ny * r.height()
            icon.move(int(cx - icon.width() / 2), int(cy - icon.img_size / 2))
        self._flash.setGeometry(r)

    def reset(self):
        for icon in self._icons.values():
            icon.deleteLater()
        self._icons.clear()
        self.update()
