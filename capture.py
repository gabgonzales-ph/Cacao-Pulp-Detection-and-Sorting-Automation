# capture.py - dataset capture for the cacao pulp sorter (USB webcam)
# A = accept, R = reject, E = empty belt, T = toggle conveyor motor, Q = quit
# Stop any service holding the camera first (and the GPIO pins).

import os
import re
import json
import sys
import time
import subprocess
import cv2
import RPi.GPIO as GPIO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE)
from webcam_config import DEVICE_INDEX, RESOLUTION, FRAMERATE, CAMERA_CONTROLS, INITIAL_WAIT, LOCK_WAIT
from components.motors import setup_motors, motors_on, motors_off, cleanup_motors  # conveyor only

BASE_DIR = os.path.join(HERE, "capture_480", "images")   # separate from the 320x320 images
CLASSES = {
    ord("a"): ("accept", "accept"),
    ord("r"): ("reject", "reject"),
    ord("e"): ("empty",  "empty"),
}
for _name, _ in CLASSES.values():
    os.makedirs(os.path.join(BASE_DIR, _name), exist_ok=True)


CROP_SIZE = (416, 416)    # must match CROP_W, CROP_H in main.py (full height of 640x480)
CROP_OFFSET = (0, 0)      # shift from frame centre (dx, dy), must match main.py
CROP_CONFIG = os.path.join(HERE, "crop_config.json")


def crop_box(frame):
    """(x1, y1, x2, y2) of the crop: centred in the frame, shifted by CROP_OFFSET."""
    h, w = frame.shape[:2]
    cw, ch = CROP_SIZE
    x1 = max(0, min(w - cw, (w - cw) // 2 + CROP_OFFSET[0]))
    y1 = max(0, min(h - ch, (h - ch) // 2 + CROP_OFFSET[1]))
    return x1, y1, x1 + cw, y1 + ch


def write_crop_config(frame):
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = crop_box(frame)
    with open(CROP_CONFIG, "w") as f:
        json.dump({
            "frame_size": [w, h],
            "crop_px": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
            "crop_norm": {"x1": x1 / w, "y1": y1 / h, "x2": x2 / w, "y2": y2 / h},
        }, f, indent=2)
    print(f"Crop box {x1},{y1} -> {x2},{y2} written to {CROP_CONFIG}")


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
    x1, y1, x2, y2 = crop_box(frame)
    if cv2.imwrite(path, frame[y1:y2, x1:x2]):
        print(f"Saved: {path}")
    else:
        print(f"[WARN] Failed to save: {path}")


def main():
    setup_motors()  # sets BCM mode and all three relays OFF
    cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cleanup_motors()
        GPIO.cleanup()
        raise RuntimeError(f"Could not open webcam at {DEVICE_INDEX}")

    motor_running = False
    crop_saved = False

    try:
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

        print("\nA = accept | R = reject | E = empty | T = motor on/off | Q = quit\n")
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            if not crop_saved:
                write_crop_config(frame)
                crop_saved = True

            view = frame.copy()  # overlay goes on the copy, never on saved images
            x1, y1, x2, y2 = crop_box(frame)
            cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 255), 2)  # saved area
            counts = "  ".join(
                f"{label[0].upper()}:{count_images(os.path.join(BASE_DIR, folder))}"
                for label, folder in CLASSES.values())
            cv2.putText(view, f"A/R/E save, T motor, Q quit   {counts}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(view, f"MOTOR: {'ON' if motor_running else 'OFF'}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if motor_running else (0, 0, 255), 2)
            cv2.imshow("Capture", view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("A"), ord("R"), ord("E"), ord("T"), ord("Q")):
                key += 32  # Caps Lock: treat uppercase as lowercase
            if key == ord("q"):
                break
            if key == ord("t"):
                motor_running = not motor_running
                (motors_on if motor_running else motors_off)()
                print(f"Motor {'ON' if motor_running else 'OFF'}")
            elif key in CLASSES:
                folder_name, prefix = CLASSES[key]
                save_frame(frame, folder_name, prefix)
    finally:
        # Same order as main.py: motors off first, GPIO cleanup last
        motors_off()
        cleanup_motors()
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Motor off, camera released")


if __name__ == "__main__":
    main()
