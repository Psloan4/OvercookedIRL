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
"""
