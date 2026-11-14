# capture.py - dataset capture for the cacao pulp sorter (USB webcam)
# A = accept, R = reject, E = empty belt, Q = quit
# Stop any service holding the camera first.

import os
import re
import sys
import time
import subprocess
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE)
from webcam_config import DEVICE_INDEX, RESOLUTION, FRAMERATE, CAMERA_CONTROLS, INITIAL_WAIT, LOCK_WAIT

BASE_DIR = os.path.join(HERE, "capture", "images")
CLASSES = {
    ord("a"): ("accept", "accept"),
    ord("r"): ("reject", "reject"),
    ord("e"): ("empty",  "empty"),
}
for _name, _ in CLASSES.values():
    os.makedirs(os.path.join(BASE_DIR, _name), exist_ok=True)


def next_number(folder, prefix):
    """Highest existing number + 1, so deleting files never causes overwrites."""
    pattern = re.compile(rf"^{prefix}_(\d+)\.jpg$")
    nums = [int(m.group(1)) for f in os.listdir(folder) if (m := pattern.match(f))]
    return max(nums, default=0) + 1


def count_images(folder):
    return len([f for f in os.listdir(folder) if f.lower().endswith(".jpg")])


def save_frame(frame, folder_name, prefix):
    folder = os.path.join(BASE_DIR, folder_name)
    n = next_number(folder, prefix)
    path = os.path.join(folder, f"{prefix}_{n}.jpg")
    cv2.imwrite(path, frame)
    print(f"Saved: {path}")


def main():
    cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open webcam at {DEVICE_INDEX}")

    # MJPG first, then size and fps (same order as main.py)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_FPS,          FRAMERATE)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    print(f"Settling... waiting {INITIAL_WAIT}s")
    time.sleep(INITIAL_WAIT)
    for ctrl, value in CAMERA_CONTROLS.items():
        r = subprocess.run(
            ["v4l2-ctl", "-d", DEVICE_INDEX, f"--set-ctrl={ctrl}={value}"],
            capture_output=True, text=True)
        if r.returncode != 0:
            print(f"[WARN] Failed to set {ctrl}={value}: {r.stderr.strip()}")
    time.sleep(LOCK_WAIT)

    print("\nA = accept | R = reject | E = empty | Q = quit\n")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            view = frame.copy()  # overlay goes on the copy, never on saved images
            counts = "  ".join(
                f"{label[0].upper()}:{count_images(os.path.join(BASE_DIR, folder))}"
                for label, folder in CLASSES.values())
            cv2.putText(view, f"A/R/E to save, Q quit   {counts}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("Capture", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in CLASSES:
                folder_name, prefix = CLASSES[key]
                save_frame(frame, folder_name, prefix)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera released")


if __name__ == "__main__":
    main()