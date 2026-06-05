import cv2

MAX_CAMERAS = 10
cameras = []

for index in range(MAX_CAMERAS):
    # CAP_DSHOW generally opens webcams more smoothly on Windows
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)

    if cap.isOpened():
        success, frame = cap.read()

        if success:
            print(f"Camera found at index {index}")
            cameras.append((index, cap))
        else:
            cap.release()
    else:
        cap.release()

if not cameras:
    print("No cameras were found.")
    raise SystemExit

print("\nPress Q while the camera windows are selected to close everything.")

while True:
    for index, cap in cameras:
        success, frame = cap.read()

        if not success:
            continue

        cv2.putText(
            frame,
            f"CAMERA INDEX: {index}",
            (30, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 0),
            3,
        )

        cv2.imshow(f"Camera {index}", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

for _, cap in cameras:
    cap.release()

cv2.destroyAllWindows()