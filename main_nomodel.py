# main_nomodel.py - TEMPORARY test version of main.py
# No model and no servo. It tests: camera, Start button, relay (conveyor), LCD,
# and the timed run. Detection counts are not available, so the LCD shows frames.
# Use the real main.py once the model is trained.
#
# Run (stop the service first):
#   SHOW_PREVIEW=1 python main_nomodel.py     (needs a display; leave unset otherwise)

import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from webcam_config import DEVICE_INDEX, RESOLUTION, FRAMERATE, CAMERA_CONTROLS, INITIAL_WAIT, LOCK_WAIT
from components.buttons import setup_buttons, start_pressed, cleanup_buttons
from components.motors import setup_motors, motors_on, motors_off, cleanup_motors
from components.lcd import setup_lcd, lcd_show, lcd_clear

RUN_SECONDS = 20            # short test run
SETTLE_SEC = 1.0
SHOW_PREVIEW = os.environ.get("SHOW_PREVIEW") == "1"
ERROR_LOG = os.path.join(HERE, "error.log")

_running = True


def _on_sigterm(signum, frame):
    global _running
    _running = False


signal.signal(signal.SIGTERM, _on_sigterm)
signal.signal(signal.SIGINT, _on_sigterm)


def log_error(msg):
    print(msg, file=sys.stderr, flush=True)
    with open(ERROR_LOG, "a") as f:
        f.write(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] {msg}\n")


class Camera:
    """Opens the webcam, locks the v4l2 controls, and grabs frames in its own thread."""

    def __init__(self):
        self.cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open webcam at {DEVICE_INDEX}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
        self.cap.set(cv2.CAP_PROP_FPS, FRAMERATE)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        time.sleep(INITIAL_WAIT)
        for ctrl, value in CAMERA_CONTROLS.items():
            r = subprocess.run(["v4l2-ctl", "-d", DEVICE_INDEX, f"--set-ctrl={ctrl}={value}"],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print(f"[WARN] Failed to set {ctrl}={value}: {r.stderr.strip()}", flush=True)
        time.sleep(LOCK_WAIT)

        self._lock = threading.Lock()
        self._frame = None
        self.fps = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        count, t0 = 0, time.time()
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame = frame
            count += 1
            if time.time() - t0 >= 1.0:
                self.fps = count / (time.time() - t0)
                count, t0 = 0, time.time()

    def latest(self):
        with self._lock:
            return self._frame

    def release(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


def main():
    camera = None
    lcd_ok = False
    try:
        try:
            setup_lcd()
            lcd_ok = True
        except Exception as e:
            log_error(f"LCD init failed: {e}")
        show = lambda a, b="": lcd_show(a, b) if lcd_ok else None

        show("Booting...", "TEST MODE")
        setup_buttons()
        setup_motors()
        show("Init Camera")
        camera = Camera()
        show("System Ready", "TEST MODE")
        time.sleep(0.5)
    except Exception:
        log_error(traceback.format_exc())
        if camera:
            camera.release()
        cleanup_motors()
        cleanup_buttons()
        sys.exit(1)

    state = "IDLE"
    done_text = "Idle (test)"
    run_end = settle_end = 0.0
    last_lcd = 0.0

    try:
        while _running:
            now = time.time()
            frame = camera.latest()

            if start_pressed():
                if state == "IDLE":
                    motors_on()
                    run_end = now + RUN_SECONDS
                    state = "RUNNING"
                    print("Start: conveyor ON", flush=True)
                elif state == "RUNNING":
                    motors_off()
                    settle_end = now + SETTLE_SEC
                    done_text = "Stopped"
                    state = "STOPPING"
                    print("Start: aborted, conveyor OFF", flush=True)

            if state == "RUNNING" and now >= run_end:
                motors_off()
                settle_end = now + SETTLE_SEC
                done_text = "Complete"
                state = "STOPPING"
                print("Timer done: conveyor OFF", flush=True)
            if state == "STOPPING" and now >= settle_end:
                state = "IDLE"

            if now - last_lcd >= 0.25:
                last_lcd = now
                if state == "IDLE":
                    show(done_text, "Press Start")
                elif state == "RUNNING":
                    left = max(0, int(run_end - now))
                    show(f"Time left {left // 60:02d}:{left % 60:02d}", f"F:{camera.fps:.0f} no model")
                else:
                    show("Stopping belt", "")

            if SHOW_PREVIEW and frame is not None:
                view = frame.copy()
                cv2.putText(view, f"{state} (test, no model)  F:{camera.fps:.0f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Cacao Test", view)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.02)
    except Exception:
        log_error(traceback.format_exc())
        raise
    finally:
        motors_off()
        cleanup_motors()
        camera.release()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        if lcd_ok:
            lcd_clear()
        cleanup_buttons()      # GPIO.cleanup() last


if __name__ == "__main__":
    main()