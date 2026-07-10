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
    def __init__(self, station_def, parent=None):
        super().__init__(parent)
        self.setWindowTitle("OvercookedIRL - Delivery")
        self.setObjectName("FinalStationWindow")
        self.setStyleSheet(
            "QWidget#FinalStationWindow { background-color: #0f172a; }"
        )
        self.resize(760, 320)
        self.setMinimumSize(360, 180)

        # Draw the counter to the real station's aspect ratio.
        self._aspect = station_def["w"] / station_def["h"]
        self._icons: dict[int, TagIcon] = {}
        self._counter_rect = QRect()

        self._title = QLabel("DELIVERY STATION", self)
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setStyleSheet(
            "color: #e2e8f0; font-size: 26px; font-weight: 900; letter-spacing: 3px;"
        )

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
        p.setBrush(QBrush(QColor("#eef2f8")))     # light counter, matches the table
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
        icon_size = max(28, int(self._counter_rect.height() * 0.5))

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
                size = int(self._counter_rect.height() * 0.9)
                pm = pm.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self._flash.play(pm)

        self._reposition()

    def _reposition(self):
        r = self._counter_rect
        for icon in self._icons.values():
            cx = r.x() + icon.nx * r.width()
            cy = r.y() + icon.ny * r.height()
            icon.move(int(cx - icon.width() / 2), int(cy - icon.img_size / 2))
        self._flash.setGeometry(r)

    def reset(self):
        for icon in self._icons.values():
            icon.deleteLater()
        self._icons.clear()
        self.update()
