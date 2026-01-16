import cv2
from feed_relay import FeedRelay

def main():
    relay = FeedRelay("/dev/video6")

    # Register stations
    relay.register_station("stat_1", 75, 120, 158, 260)
    relay.register_station("stat_2a", 75 + 158, 120, 203, 135)
    relay.register_station("stat_2b", 75 + 158, 120 + 135, 203, 125)
    relay.register_station("stat_3", 75 + 158 + 203, 120, 121, 260)

    # Index 0 = live feed, rest are stations
    station_names = ["LIVE", "stat_1", "stat_2a", "stat_2b", "stat_3"]
    selected_idx = 0

    def on_trackbar(val):
        nonlocal selected_idx
        selected_idx = val

    cv2.namedWindow("View", cv2.WINDOW_NORMAL)
    cv2.createTrackbar(
        "View",
        "View",
        0,
        len(station_names) - 1,
        on_trackbar
    )

    while True:
        frame = relay.get_image_raw()

        if selected_idx == 0:
            # ---- LIVE FEED VIEW ----
            display = frame.copy()

            # Draw all station rectangles
            for name in station_names[1:]:
                x, y, w, h = relay.stations[name]
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)

            label = "LIVE FEED"

        else:
            # ---- STATION VIEW ----
            name = station_names[selected_idx]
            display = relay.get_station_image(name)
            label = name

        # Overlay label
        cv2.putText(
            display,
            label,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        cv2.imshow("View", display)

        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    relay.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
