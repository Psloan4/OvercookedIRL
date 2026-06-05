import cv2
import argparse
import sys
import math
import time
import socket
import numpy as np

from stream_server import start_server, publish

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def get_lan_ip():
    """Best-effort local LAN IP so we can print usable stream URLs."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't actually send anything; just picks the outbound interface.
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except OSError:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def create_detector(dict_name):
    if dict_name not in ARUCO_DICT:
        print(f"Unsupported dictionary: {dict_name}")
        sys.exit(1)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        use_new_api = True
    else:
        params = cv2.aruco.DetectorParameters_create()
        detector = None
        use_new_api = False

    return aruco_dict, params, detector, use_new_api


def open_camera(index, width, height, fps, backend):
    cap = cv2.VideoCapture(index, backend)

    if not cap.isOpened():
        print(f"Could not open camera {index}")
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6) #can play with this setting


    return cap


def detect_markers(frame, aruco_dict, params, detector, use_new_api):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if use_new_api:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=params
        )

    display = frame.copy()

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(display, corners, ids)

    return display, ids


def make_placeholder(tile_w, tile_h, text):
    img = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    cv2.putText(
        img,
        text,
        (20, tile_h // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )
    return img


def annotate_frame(frame, cam_index, ids):
    label = f"Camera {cam_index}"
    if ids is not None and len(ids) > 0:
        id_text = "IDs: " + ", ".join(map(str, ids.flatten()))
    else:
        id_text = "IDs: none"

    cv2.putText(
        frame,
        label,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        frame,
        id_text,
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    return frame


def build_dashboard(frames, tile_w, tile_h, cols=2):
    """
    frames: list of already-annotated frames
    Returns one combined dashboard image.
    """
    if len(frames) == 0:
        return make_placeholder(tile_w, tile_h, "No camera frames")

    # Resize every frame to the same tile size
    tiles = [cv2.resize(f, (tile_w, tile_h)) for f in frames]

    rows = math.ceil(len(tiles) / cols)
    needed = rows * cols

    while len(tiles) < needed:
        tiles.append(make_placeholder(tile_w, tile_h, "Empty"))

    row_images = []
    for r in range(rows):
        row_tiles = tiles[r * cols:(r + 1) * cols]
        row_img = cv2.hconcat(row_tiles)
        row_images.append(row_img)

    dashboard = cv2.vconcat(row_images)
    return dashboard


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", type=str, default="DICT_4X4_50")
    ap.add_argument(
        "--cams",
        type=int,
        nargs="+",
        default=[0, 1, 2, 3],
        help="Camera indices, e.g. --cams 0 1 2 3"
    )
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--tile-width", type=int, default=640)
    ap.add_argument("--tile-height", type=int, default=480)
    ap.add_argument(
        "--backend",
        type=str,
        default="DSHOW",
        choices=["DSHOW", "MSMF", "ANY"]
    )
    ap.add_argument(
        "--stream",
        action="store_true",
        help="Serve each camera + the dashboard as MJPEG over HTTP for other PCs"
    )
    ap.add_argument("--port", type=int, default=8080, help="HTTP port for --stream")
    ap.add_argument(
        "--headless",
        action="store_true",
        help="Don't open a local window (useful when only streaming). Ctrl+C to quit."
    )
    args = ap.parse_args()

    backend_map = {
        "DSHOW": cv2.CAP_DSHOW,
        "MSMF": cv2.CAP_MSMF,
        "ANY": cv2.CAP_ANY,
    }
    backend = backend_map[args.backend]

    aruco_dict, params, detector, use_new_api = create_detector(args.type)

    caps = {}
    for cam_index in args.cams:
        cap = open_camera(cam_index, args.width, args.height, args.fps, backend)
        if cap is not None:
            caps[cam_index] = cap

    if len(caps) == 0:
        print("No cameras could be opened.")
        sys.exit(1)

    print("Opened cameras:", list(caps.keys()))

    if args.stream:
        start_server(port=args.port, fps=args.fps)
        ip = get_lan_ip()
        print(f"Streaming on http://{ip}:{args.port}/  (open in a browser to verify)")
        for cam_index in args.cams:
            print(f"  camera {cam_index}: http://{ip}:{args.port}/cam/{cam_index}")
        print(f"  dashboard: http://{ip}:{args.port}/dashboard")

    if args.headless:
        print("Running headless. Press Ctrl+C to quit.")
    else:
        print("Press q or ESC to quit.")

    try:
        while True:
            dashboard_frames = []

            for cam_index in args.cams:
                if cam_index not in caps:
                    placeholder = make_placeholder(
                        args.tile_width, args.tile_height, f"Camera {cam_index} unavailable"
                    )
                    dashboard_frames.append(placeholder)
                    if args.stream:
                        publish(f"cam/{cam_index}", placeholder)
                    continue

                cap = caps[cam_index]
                ok, frame = cap.read()

                if not ok or frame is None:
                    placeholder = make_placeholder(
                        args.tile_width, args.tile_height, f"Camera {cam_index} no frame"
                    )
                    dashboard_frames.append(placeholder)
                    if args.stream:
                        publish(f"cam/{cam_index}", placeholder)
                    continue

                display, ids = detect_markers(frame, aruco_dict, params, detector, use_new_api)
                display = annotate_frame(display, cam_index, ids)
                dashboard_frames.append(display)
                if args.stream:
                    publish(f"cam/{cam_index}", display)

            dashboard = build_dashboard(
                dashboard_frames,
                tile_w=args.tile_width,
                tile_h=args.tile_height,
                cols=2
            )

            if args.stream:
                publish("dashboard", dashboard)

            if args.headless:
                # No GUI; pace the loop so we don't peg a core.
                time.sleep(1.0 / max(args.fps, 1))
            else:
                cv2.imshow("Multi-Camera ArUco Dashboard", dashboard)
                key = cv2.waitKey(1) & 0xFF
                if key in [ord("q"), 27]:
                    break
    except KeyboardInterrupt:
        print("\nShutting down.")

    for cap in caps.values():
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()