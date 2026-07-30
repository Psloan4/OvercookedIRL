"""
Standalone window listing the actions available right now -- the live output of
actions.available_actions(). Two humans use it to eyeball the best move; later
the robot's information-handler will score the same list.

Opt in with the --actions CLI flag. Pure view: it just re-renders whatever the
actions module returns each detection tick.
"""

from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QScrollArea
from PySide6.QtCore import Qt

import actions as actions_mod

# Dot colour per action kind, so the list scans quickly by eye.
_KIND_COLOR = {
    "deliver": "#22c55e",
    "combine": "#f59e0b",
    "progress": "#38bdf8",
    "start": "#e879f9",
    "trash": "#ef4444",
    "wait": "#64748b",
}


class ActionsWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("OvercookedIRL - Actions")
        self.setObjectName("ActionsWindow")
        self.setStyleSheet("QWidget#ActionsWindow { background-color: #0f172a; }")
        self.resize(440, 640)
        self.setMinimumSize(320, 320)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        title = QLabel("AVAILABLE ACTIONS", self)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "color:#e2e8f0; font-size:20px; font-weight:900;"
            " letter-spacing:2px; padding:12px;"
        )
        root.addWidget(title)

        self._scroll = QScrollArea(self)
        self._scroll.setWidgetResizable(True)
        self._scroll.setStyleSheet(
            "QScrollArea { border:none; background:transparent; }"
        )
        self._body = QLabel(self)
        self._body.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self._body.setTextFormat(Qt.RichText)
        self._body.setWordWrap(True)
        self._body.setStyleSheet("color:#e2e8f0; font-size:14px; padding:8px 14px;")
        self._scroll.setWidget(self._body)
        root.addWidget(self._scroll, 1)

        self.reset()

    def reset(self):
        self._body.setText(self._render([]))

    def update_view(self, present_items, orders=(), station_status=None):
        """Fed each detection tick with main.py's render_list, current orders,
        and the per-station status dict (for WAIT actions)."""
        acts = actions_mod.available_actions(present_items, orders, station_status)
        self._body.setText(self._render(acts))

    def _chip(self, a) -> str:
        color = _KIND_COLOR.get(a.kind, "#e2e8f0")
        star = " &#9733;" if (a.kind == "deliver" and a.wanted) else ""
        return (
            f"&nbsp;&nbsp;<span style='color:{color}'>&#9679;</span> "
            f"<span style='color:#e2e8f0'>{a.label()}</span>"
            f"<span style='color:#22c55e; font-weight:700'>{star}</span>"
        )

    def _render(self, acts) -> str:
        if not acts:
            return (
                "<span style='color:#64748b'>No items on the table yet. "
                "Place an ingredient, or FETCH one.</span>"
            )

        item_acts = [a for a in acts if a.tag is not None]
        start_acts = [a for a in acts if a.tag is None and a.kind == "start"]
        wait_acts = [a for a in acts if a.tag is None and a.kind == "wait"]

        by_tag: dict[tuple, list] = {}
        for a in item_acts:
            by_tag.setdefault((a.tag, a.item_state), []).append(a)

        rows = []
        for (tag, state), group in sorted(by_tag.items()):
            group.sort(key=lambda a: a.priority)
            chips = "<br>".join(self._chip(a) for a in group)
            rows.append(
                "<div style='margin:6px 0'>"
                f"<span style='color:#f8fafc; font-weight:700'>tag {tag}</span> "
                f"<span style='color:#94a3b8'>&middot; {state}</span>"
                f"<br>{chips}</div>"
            )

        rows.append(self._section("START -- open orders", start_acts))
        rows.append(self._section("WAIT", wait_acts))
        return "".join(r for r in rows if r)

    def _section(self, heading: str, acts) -> str:
        if not acts:
            return ""
        chips = "<br>".join(self._chip(a) for a in acts)
        return (
            "<div style='margin:12px 0 4px'>"
            "<span style='color:#94a3b8; font-weight:700; letter-spacing:1px'>"
            f"{heading}</span><br>{chips}</div>"
        )
