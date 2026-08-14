"""
Standalone window ranking the actions available right now -- the live output of
actions.available_actions(), best move first. Each row shows the action, its
total score, and the weighted components that sum to that score (the same
breakdown the robot's information-handler reads).

Opt in with the --actions CLI flag. Pure view: it just re-renders whatever the
actions module returns each detection tick; all scoring lives in actions.py.
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

        title = QLabel("ACTION RANKING", self)
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

    def _render(self, acts) -> str:
        if not acts:
            return (
                "<span style='color:#64748b'>No items on the table yet. "
                "Place an ingredient, or FETCH one.</span>"
            )
        # acts arrives already sorted best-first by actions.available_actions().
        return "".join(self._row(a, i) for i, a in enumerate(acts))

    def _row(self, a, rank: int) -> str:
        """One ranked action: kind dot, label, total score, weight breakdown."""
        color = _KIND_COLOR.get(a.kind, "#e2e8f0")
        if a.score > 0:
            score_color = "#22c55e"
        elif a.score < 0:
            score_color = "#ef4444"
        else:
            score_color = "#94a3b8"

        breakdown = " &middot; ".join(
            f"{name} {value:+.0f}" for name, value in a.weights
        ) or "&mdash;"

        badge = ""
        if rank == 0:
            badge = (
                "<div style='color:#fbbf24; font-weight:800; letter-spacing:2px;"
                " font-size:12px; margin:8px 0 2px'>&#9733; BEST ACTION</div>"
            )

        return (
            f"{badge}"
            "<div style='margin:6px 0'>"
            f"<span style='color:{color}'>&#9679;</span> "
            f"<span style='color:#e2e8f0'>{a.label()}</span> "
            f"<span style='color:{score_color}; font-weight:800'>"
            f"&nbsp;{a.score:.0f}</span>"
            "<br><span style='color:#64748b; font-size:12px'>"
            f"&nbsp;&nbsp;&nbsp;{breakdown}</span>"
            "</div>"
        )
