# main.py - cacao pulp sorter
# Detect pulp on the conveyor: accept (milky/off-white) vs reject (everything else).
# States: IDLE -> RUNNING -> STOPPING -> IDLE
#   IDLE      relay off, Start button starts a timed run
#   RUNNING   conveyor on for RUN_SECONDS, detect + kick rejects. Start again = abort
#   STOPPING  conveyor off, keep detecting for a settle time so pulp already on
#             the belt is still sorted, then back to IDLE ("Complete")
# Safe Shutdown (GPIO17) is handled by the kernel overlay; SIGTERM cleans up here.
#
# If the model file is missing or fails to load, the script runs without detection:
# the conveyor, buttons and LCD still work; no kicks are fired.
#
# Run manually (stop the service first):
#   SHOW_PREVIEW=1 python main.py      (needs a display; leave unset for the service)

import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from webcam_config import DEVICE_INDEX, RESOLUTION, FRAMERATE, CAMERA_CONTROLS, INITIAL_WAIT, LOCK_WAIT
from components.buttons import setup_buttons, start_pressed, cleanup_buttons
from components.motors import setup_motors, motors_on, motors_off, cleanup_motors
from components.rejector import setup_rejector, schedule_kick, pending_kicks, clear_pending, cleanup_rejector
from components.lcd import setup_lcd, lcd_show, lcd_clear

# ---------------- Settings ----------------
MODEL_PATH = os.path.join(HERE, "models", "cacao_pulp_model_v1_float32.tflite")
CLASS_NAMES = ["accept", "reject"]      # same order as data.yaml
REJECT_CLASS = 1
IMG_SIZE = 320
CONF_THRESH = 0.50
NMS_THRESH = 0.45

CONFIRM_FRAMES = 6          # inference frames a pulp must be seen before it counts
MAX_MISS_FRAMES = 5         # frames a track may vanish before it is dropped
MAX_MATCH_DIST = 100        # px (at 640x480) a pulp may move between inference frames

RUN_SECONDS = 60            # TODO: set the conveyor run time per batch
SETTLE_SEC = 1.0            # extra time after the conveyor stops
KICK_DELAY_SEC = 0.0        # TODO: camera-to-kicker distance / belt speed (0 = kicker at camera zone)

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


# ---------------- Camera ----------------
class Camera:
    """Opens the webcam, locks the v4l2 controls, and grabs frames in its own thread."""

    def __init__(self):
        self.cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open webcam at {DEVICE_INDEX}")
        # MJPG first, then size and fps (YUYV caps at 25 fps on this camera)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
        self.cap.set(cv2.CAP_PROP_FPS, FRAMERATE)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        time.sleep(INITIAL_WAIT)
        for ctrl, value in CAMERA_CONTROLS.items():     # dict order = apply order
            r = subprocess.run(["v4l2-ctl", "-d", DEVICE_INDEX, f"--set-ctrl={ctrl}={value}"],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print(f"[WARN] Failed to set {ctrl}={value}: {r.stderr.strip()}", flush=True)
        time.sleep(LOCK_WAIT)

        self._lock = threading.Lock()
        self._frame = None
        self._frame_id = 0
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
                self._frame_id += 1
            count += 1
            if time.time() - t0 >= 1.0:
                self.fps = count / (time.time() - t0)
                count, t0 = 0, time.time()

    def latest(self):
        """(frame, frame_id) of the newest frame, or (None, 0)."""
        with self._lock:
            return self._frame, self._frame_id

    def release(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


# ---------------- Detector ----------------
class Detector:
    def __init__(self, path):
        from ai_edge_litert.interpreter import Interpreter
        self.interp = Interpreter(model_path=path, num_threads=4)
        self.interp.allocate_tensors()
        self.inp = self.interp.get_input_details()[0]
        self.out = self.interp.get_output_details()[0]

    def detect(self, frame):
        """Returns [(cx, cy, class_id, conf), ...] in original-frame pixels."""
        h, w = frame.shape[:2]
        r = min(IMG_SIZE / h, IMG_SIZE / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        px, py = (IMG_SIZE - nw) // 2, (IMG_SIZE - nh) // 2
        canvas = np.full((IMG_SIZE, IMG_SIZE, 3), 114, np.uint8)
        canvas[py:py + nh, px:px + nw] = cv2.resize(frame, (nw, nh))
        rgb = canvas[:, :, ::-1]
        data = np.expand_dims(rgb.astype(np.float32) / 255.0, 0)

        self.interp.set_tensor(self.inp["index"], data)
        self.interp.invoke()
        out = self.interp.get_tensor(self.out["index"])[0]
        if out.shape[0] < out.shape[1]:      # (6, N) -> (N, 6)
            out = out.T

        boxes, scores = out[:, :4].copy(), out[:, 4:]
        cls = scores.argmax(axis=1)
        conf = scores.max(axis=1)
        keep = conf >= CONF_THRESH
        if not keep.any():
            return []
        boxes, cls, conf = boxes[keep], cls[keep], conf[keep]
        if boxes.max() <= 1.5:               # normalised coordinates
            boxes *= IMG_SIZE

        # xywh (letterboxed) -> xywh in the original frame
        cx = (boxes[:, 0] - px) / r
        cy = (boxes[:, 1] - py) / r
        bw, bh = boxes[:, 2] / r, boxes[:, 3] / r
        nms_boxes = [[float(cx[i] - bw[i] / 2), float(cy[i] - bh[i] / 2), float(bw[i]), float(bh[i])]
                     for i in range(len(cx))]
        idx = cv2.dnn.NMSBoxes(nms_boxes, conf.tolist(), CONF_THRESH, NMS_THRESH)
        idx = np.array(idx).flatten() if len(idx) else []
        return [(float(cx[i]), float(cy[i]), int(cls[i]), float(conf[i])) for i in idx]


# ---------------- Confirmation tracker ----------------
class Tracker:
    """A pulp counts once it has been seen CONFIRM_FRAMES times; its class is the majority vote."""

    def __init__(self):
        self.tracks = []

    def clear(self):
        self.tracks = []

    def update(self, dets):
        for t in self.tracks:
            t["seen"] = False
        for cx, cy, c, _ in dets:
            best, best_d = None, MAX_MATCH_DIST
            for t in self.tracks:
                if t["seen"]:
                    continue
                d = ((t["cx"] - cx) ** 2 + (t["cy"] - cy) ** 2) ** 0.5
                if d < best_d:
                    best, best_d = t, d
            if best is None:
                best = {"cx": cx, "cy": cy, "hits": 0, "miss": 0,
                        "votes": [0] * len(CLASS_NAMES), "confirmed": False}
                self.tracks.append(best)
            best.update(cx=cx, cy=cy, seen=True, miss=0)
            best["hits"] += 1
            best["votes"][c] += 1
        for t in self.tracks:
            if not t["seen"]:
                t["miss"] += 1
        self.tracks = [t for t in self.tracks if t["miss"] <= MAX_MISS_FRAMES]

        newly = []
        for t in self.tracks:
            if not t["confirmed"] and t["hits"] >= CONFIRM_FRAMES:
                t["confirmed"] = True
                newly.append(int(np.argmax(t["votes"])))
        return newly


# ---------------- Main ----------------
def draw_preview(frame, state, accepts, rejects, no_model):
    view = frame.copy()
    label = f"{state}  A:{accepts} R:{rejects}"
    if no_model:
        label += "  [NO MODEL]"
    cv2.putText(view, label, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.imshow("Cacao Sorter", view)
    return cv2.waitKey(1) & 0xFF != ord("q")


def main():
    global _running
    camera = None
    try:
        lcd_ok = False
        try:
            setup_lcd()
            lcd_ok = True
        except Exception as e:
            log_error(f"LCD init failed: {e}")      # run without LCD rather than not at all
        step = lambda a, b="": lcd_show(a, b) if lcd_ok else None

        step("Booting...")
        setup_buttons()
        setup_motors()
        setup_rejector()

        # Try to load the model — not fatal if missing
        detector = None
        step("Loading Model")
        if not os.path.exists(MODEL_PATH):
            log_error(f"[WARN] Model not found: {MODEL_PATH} — running without detection")
            step("No model found", "belt test only")
            time.sleep(2.0)
        else:
            try:
                detector = Detector(MODEL_PATH)
                print("[INFO] Model loaded OK", flush=True)
            except Exception as e:
                log_error(f"[WARN] Model load failed: {e} — running without detection")
                step("Model load fail", "belt test only")
                time.sleep(2.0)

        step("Init Camera")
        camera = Camera()
        step("Calibrating...")
        time.sleep(1.0)
        if detector is None:
            step("System Ready", "No detection")
        else:
            step("System Ready")
        time.sleep(0.5)

    except Exception:
        log_error(traceback.format_exc())
        try:
            lcd_show("Init Failed", "See error.log")
        except Exception:
            pass
        if camera:
            camera.release()
        cleanup_motors()
        cleanup_rejector()
        cleanup_buttons()
        sys.exit(1)

    tracker = Tracker()
    state = "IDLE"
    done_text = "Idle"
    accepts = rejects = 0
    run_end = settle_end = 0.0
    last_id = 0
    last_lcd = 0.0
    no_model = detector is None

    try:
        while _running:
            now = time.time()
            frame, fid = camera.latest()

            # ---- buttons (polled in every state) ----
            if start_pressed():
                if state == "IDLE":
                    tracker.clear()
                    clear_pending()
                    accepts = rejects = 0
                    last_id = fid
                    motors_on()
                    run_end = now + RUN_SECONDS
                    state = "RUNNING"
                elif state == "RUNNING":                # abort early
                    motors_off()
                    settle_end = now + SETTLE_SEC + KICK_DELAY_SEC
                    done_text = "Stopped"
                    state = "STOPPING"

            # ---- state transitions on time ----
            if state == "RUNNING" and now >= run_end:
                motors_off()
                settle_end = now + SETTLE_SEC + KICK_DELAY_SEC
                done_text = "Complete"
                state = "STOPPING"
            if state == "STOPPING" and now >= settle_end and pending_kicks() == 0:
                state = "IDLE"

            # ---- detection (skipped when no model) ----
            if state in ("RUNNING", "STOPPING") and frame is not None and fid != last_id:
                last_id = fid
                if detector is not None:
                    for c in tracker.update(detector.detect(frame)):
                        if c == REJECT_CLASS:
                            rejects += 1
                            schedule_kick(KICK_DELAY_SEC)
                        else:
                            accepts += 1
            else:
                time.sleep(0.01)

            # ---- LCD (at most every 0.25 s) ----
            if now - last_lcd >= 0.25:
                last_lcd = now
                if state == "IDLE":
                    line2 = "Press Start" if not no_model else "No detection"
                    lcd_show(done_text, line2)
                elif state == "RUNNING":
                    left = max(0, int(run_end - now))
                    if no_model:
                        lcd_show(f"Time {left // 60:02d}:{left % 60:02d}",
                                 f"F:{camera.fps:.0f} no model")
                    else:
                        lcd_show(f"Time left {left // 60:02d}:{left % 60:02d}",
                                 f"A:{accepts} R:{rejects} F:{camera.fps:.0f}")
                else:
                    lcd_show("Stopping belt",
                             f"A:{accepts} R:{rejects}" if not no_model else "")

            if SHOW_PREVIEW and frame is not None:
                if not draw_preview(frame, state, accepts, rejects, no_model):
                    break
    except Exception:
        log_error(traceback.format_exc())
        raise
    finally:
        # Cleanup order matters: GPIO.cleanup() (cleanup_buttons) must be last
        motors_off()
        cleanup_motors()
        cleanup_rejector()
        camera.release()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        if lcd_ok:
            lcd_clear()
        cleanup_buttons()


if __name__ == "__main__":
    main()
