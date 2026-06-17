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

QWidget#TableView {
    background: #e9edf5;
}

/* Station drawn as a translucent panel on the table surface. */
QWidget#StationZone {
    background: rgba(255, 255, 255, 0.75);
    border: 1px solid #c4cdda;
    border-radius: 12px;
}
QWidget#StationZone[scan="true"] {
    background: rgba(37, 99, 235, 0.14);
    border: 1px solid #93c5fd;
}
/* Brief red flash when a scan is wiped because the player left the zone. */
QWidget#StationZone[reset_flash="true"] {
    background: rgba(220, 38, 38, 0.18);
    border: 2px solid #dc2626;
}
QLabel#ZoneName {
    font-size: 44px;
    font-weight: 900;
    color: #1f2937;
    background: transparent;
    border: none;
}
QLabel#ZoneInfo {
    font-size: 34px;
    font-weight: 800;
    color: #4b5563;
    background: transparent;
    border: none;
}
QLabel#PillReady {
    background: #eef1f7;
    color: #374151;
    border: 1px solid #cbd5e1;
    padding: 8px 22px;
    border-radius: 999px;
    font-size: 30px;
    font-weight: 900;
}
QLabel#PillScan {
    background: #dbeafe;
    color: #1d4ed8;
    border: 1px solid #93c5fd;
    padding: 8px 22px;
    border-radius: 999px;
    font-size: 30px;
    font-weight: 900;
}
/* Gated station (2a/2b) with no player standing beside it. */
QLabel#PillNoPlayer {
    background: #fef3c7;
    color: #b45309;
    border: 1px solid #fcd34d;
    padding: 8px 22px;
    border-radius: 999px;
    font-size: 30px;
    font-weight: 900;
}
QLabel#PlayerGlyph {
    font-size: 40px;
    background: transparent;
    border: none;
}

/* Thin per-item scan bar that sits under each patty on the table. */
QProgressBar#ScanBar {
    background: rgba(0, 0, 0, 0.18);
    border: none;
    border-radius: 4px;
    height: 8px;
}
QProgressBar#ScanBar::chunk {
    background: #2563eb;
    border-radius: 4px;
}
"""
