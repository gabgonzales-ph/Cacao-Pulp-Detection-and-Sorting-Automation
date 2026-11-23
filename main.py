#main.py // do not remove this comment
# Cacao pulp sorter: main pipeline.
# Detection settings and flow taken from the coffee-bean project (centre-crop
# detection zone, tracker with class lock, trigger line, camera-failure
# recovery, threaded inference, clean shutdown on SIGTERM), wired to the cacao
# hardware: 3 relays (conveyor, cracker, dispenser), servo kicker, 16x2 LCD, Start button.
#
#   IDLE --Start--> FEEDING <--> SORTING --> IDLE
#
#   IDLE      all relays off. LCD: "System Ready / Press Start".
#   FEEDING   cracker + dispenser + conveyor on, detection active.
#             LCD: "Feeding" + counts. Start = switch to SORTING.
#   SORTING   cracker off. Two stages:
#               1) DISPENSE: dispenser + conveyor run DISPENSER_SEC
#               2) BELT: dispenser off, conveyor runs alone until the camera
#                  sees no newly CONFIRMED pulp for EMPTY_WINDOW_SEC (then IDLE).
#                  Without a camera/model it falls back to a fixed BELT_SEC.
#             LCD: "Sorting" + counts.
#             Start during either SORTING stage switches back to FEEDING:
#             all three relays turn on and the SORTING timers are reset.
#             Press Start again to return to SORTING, restarting both timers.
#             Detection and batch counts continue across these mode switches.
#             When the conveyor timer finishes, the process returns to IDLE.
#   The empty window only starts when the dispenser turns off, so it is never
#   running while the dispenser is on.
#
# Camera dropout: the belt pauses and the LCD shows "Camera problem" over the
# Feeding/Sorting text. DISPENSE stage: countdown pauses, no time lost.
# BELT stage: the empty window restarts at full EMPTY_WINDOW_SEC on recovery.
#
# Emergency stop (Y090, NC contact on GPIO5, components/estop.py): pressed ->
# ONLY the dispenser and cracker relays turn off. The conveyor keeps running and
# detection/kicking carry on. While it is pressed the dispenser countdown is
# frozen, Start cannot begin a new batch, and line 1 of the LCD reads Emergency Stop. Released ->
# the dispenser/cracker relays are restored for the current state and the
# process resumes.
#
# The Shutdown button (GPIO17) is read in components/buttons.py (the kernel
# gpio-shutdown overlay must be removed from config.txt):
#   hold 2 s while IDLE -> clean shutdown of the Pi (ignored during a batch)

import os
import queue
import signal
import subprocess
import threading
import time

import cv2
import numpy as np

from webcam_config import (DEVICE_INDEX, RESOLUTION, FRAMERATE,
                           CAMERA_CONTROLS, INITIAL_WAIT, LOCK_WAIT)
from components.buttons import (setup_buttons, start_pressed, shutdown_event,
                                cleanup_buttons)
from components.estop import setup_estop, estop_active
from components.motors import (setup_motors, all_on, all_off, motors_on,
                               cracker_off, dispenser_on, cleanup_motors)
try:
    from components.motors import dispenser_off
except ImportError:
    def dispenser_off():
        # Fallback if motors.py has no per-relay dispenser_off:
        # everything off, then conveyor back on.
        all_off()
        motors_on()
motors_off = all_off
from components.rejector import (setup_rejector, schedule_kick,
                                 clear_pending, cleanup_rejector)
from components.lcd import setup_lcd, lcd_show, lcd_clear

HERE = os.path.dirname(os.path.abspath(__file__))

# ─── Debug / Display Toggle ──────────────────────────────────────────────────
# SHOW_PREVIEW=1 when running manually with a display. Leave unset under systemd.
SHOW_PREVIEW = os.environ.get("SHOW_PREVIEW", "0") == "1"
PREVIEW_WIDTH = 640        # preview window width; the frame is shrunk to this for display only
# DEBUG_SCORES=1 prints the best raw model score about once a second, even when it is
# below CONF_THRESHOLD, so you can tell "model sees nothing" from "threshold too high".
DEBUG_SCORES = os.environ.get("DEBUG_SCORES", "0") == "1"

# ─── Model / detection settings (same values as the coffee project) ──────────
MODEL_PATH     = os.path.join(HERE, "models", "v3.tflite")
CLASSES        = {0: "good", 1: "reject"}      # same order as training data.yaml
REJECT_CLASS   = 1
CONF_THRESHOLD = 0.5       # minimum model confidence (0-1) to count a detection; tune on the belt
NMS_THRESHOLD  = 0.4
NUM_THREADS    = 3

CONFIRM_FRAMES = 2         # frames a pulp must be seen before it is counted
PROXIMITY_PX   = 80        # max centre distance to match a detection to a track
MAX_MISSED     = 4         # frames a track survives with no match

COLOR_PENDING = (0, 255, 255)
COLOR_GOOD    = (0, 255, 0)
COLOR_REJECT  = (0, 0, 255)

LCD_INTERVAL = 0.25        # seconds between LCD refreshes (I2C writes are slow)

# ─── Detection zone (centre crop) ────────────────────────────────────────────
# The V2 model was trained on 416x416 centre crops of the 640x480 frame. Keep these
# equal to CROP_SIZE / CROP_OFFSET in capture.py. Only pulp inside this box is
# detected, so the belt must pass through it. Drawn in cyan with SHOW_PREVIEW=1.
CROP_W, CROP_H = 416, 423
CROP_OFFSET    = (0, 0)    # (dx, dy) shift from the exact centre
# These are in BASE_RESOLUTION pixels (640x480). At a higher capture resolution
# the box, offset and trigger line are scaled up (width / BASE_W), so the same
# view of the belt is used; the crop is resized to the model's 416x416 input.
# The tracker, PROXIMITY_PX and TRIGGER_LINE_OFFSET all stay in base units.
BASE_W, BASE_H = 640, 480   # the resolution the model's 416x416 box was defined at

# ─── Reject trigger line ─────────────────────────────────────────────────────
# A confirmed reject triggers the kicker once its box touches this horizontal
# line. 0 = top edge of the zone; raise to move it down (pixels). Drawn white.
TRIGGER_LINE_OFFSET = 416

# ─── Batch settings ──────────────────────────────────────────────────────────
DISPENSER_SEC  = 120.0     # SORTING stage 1: dispenser + conveyor run this long
BELT_SEC       = 160.0     # SORTING stage 2 fallback ONLY when there is no camera/model
EMPTY_WINDOW_SEC = 20.0    # SORTING stage 2: no new confirmed pulp this long -> belt stops, IDLE
KICK_DELAY_SEC = 0.0       # TODO: camera-to-kicker distance / belt speed

# ─── Camera failure handling (same as coffee) ────────────────────────────────
CAM_PAUSE_SEC  = 0.5       # no frame this long -> stop the belt
CAM_REOPEN_SEC = 2.0       # then close/reopen the webcam (again every 4 s)
CAM_ABORT_SEC  = 15.0      # give up, back to IDLE with "Camera Error"
CAM_BY_ID = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Webcam_Webcam-video-index0"

ERROR_LOG = os.path.join(HERE, "error.log")


def log_error(msg):
    print(msg)
    try:
        with open(ERROR_LOG, "a") as f:
            f.write(f"{time.ctime()}: {msg}\n")
    except Exception:
        pass


# ─── Init ────────────────────────────────────────────────────────────────────
stop_event = threading.Event()


def _handle_shutdown_signal(signum, frame):
    # systemd sends SIGTERM; make the main loop exit so cleanup actually runs.
    print(f"[SIGNAL] Received signal {signum} - requesting clean shutdown")
    stop_event.set()


signal.signal(signal.SIGTERM, _handle_shutdown_signal)
signal.signal(signal.SIGINT, _handle_shutdown_signal)

lcd_ok = False
interpreter = None          # None = no-model fallback
cap = None
img_w = img_h = 416
channels_first = False

try:
    try:
        setup_lcd()
        lcd_ok = True
        lcd_show("Booting...")
    except Exception as e:
        log_error(f"[WARN] LCD init failed: {e!r}")

    setup_buttons()
    setup_estop()
    setup_motors()
    setup_rejector()

    if not os.path.exists(MODEL_PATH):
        log_error(f"[WARN] Model not found: {MODEL_PATH} - running without detection")
    else:
        try:
            from ai_edge_litert.interpreter import Interpreter
            interpreter = Interpreter(model_path=MODEL_PATH, num_threads=NUM_THREADS)
            interpreter.allocate_tensors()
            in_det = interpreter.get_input_details()[0]
            out_det = interpreter.get_output_details()[0]
            # [1,H,W,3] (channels-last) on older exports, [1,3,H,W] on newer ones.
            _s = [int(v) for v in in_det["shape"]]
            channels_first = _s[1] == 3 and _s[3] != 3
            img_h, img_w = (_s[2], _s[3]) if channels_first else (_s[1], _s[2])
            _n_rows = int(out_det["shape"][1])
            print(f"Model output shape: {[int(v) for v in out_det['shape']]} "
                  f"({_n_rows - 4} classes; CLASSES has {len(CLASSES)})")
            if _n_rows != 4 + len(CLASSES):
                log_error(f"[WARN] Model has {_n_rows - 4} classes but CLASSES has "
                          f"{len(CLASSES)} - this may be the wrong model file")
            print(f"Model loaded - input: {img_w}x{img_h} "
                  f"({'channels-first' if channels_first else 'channels-last'}), "
                  f"threads={NUM_THREADS}")
        except Exception as e:
            interpreter = None
            log_error(f"[WARN] Model failed to load: {e!r} - running without detection")

    if interpreter is not None:
        cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open webcam at {DEVICE_INDEX}")
        # MJPG must be requested BEFORE width/height/fps (YUYV caps at 25 fps).
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RESOLUTION[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
        cap.set(cv2.CAP_PROP_FPS,          FRAMERATE)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # newest frame only
        print(f"Webcam opened at {DEVICE_INDEX} - "
              f"requested {RESOLUTION[0]}x{RESOLUTION[1]} @ {FRAMERATE}fps")
        print(f"Settling... waiting {INITIAL_WAIT}s")
        time.sleep(INITIAL_WAIT)

        # Applied in dict order: auto modes are switched to manual first.
        for ctrl, value in CAMERA_CONTROLS.items():
            r = subprocess.run(
                ["v4l2-ctl", "-d", DEVICE_INDEX, f"--set-ctrl={ctrl}={value}"],
                capture_output=True, text=True)
            if r.returncode != 0:
                print(f"[WARN] Failed to set {ctrl}={value}: {r.stderr.strip()}")
            else:
                print(f"Set {ctrl} = {value}")
        print(f"Waiting {LOCK_WAIT}s for settings to take effect...")
        time.sleep(LOCK_WAIT)
        print("Camera ready")

    if lcd_ok:
        lcd_show("System Ready", "Press Start")
    time.sleep(1)

except Exception as e:
    log_error(f"[INIT FAILED] {e!r}")
    if lcd_ok:
        lcd_show("Init Failed", "See error.log")
    try:
        motors_off()
    except Exception:
        pass
    raise

# ─── Tracker ─────────────────────────────────────────────────────────────────
def trigger_actuator():
    """Queue one servo kick (non-blocking; rejector worker fires it in order)."""
    print("[ACTUATOR] Kicker triggered - rejecting pulp")
    schedule_kick(KICK_DELAY_SEC)


class PulpTracker:
    def __init__(self, confirm_frames, proximity_px, max_missed):
        self.confirm_frames = confirm_frames
        self.proximity_px   = proximity_px
        self.max_missed     = max_missed
        self.pool = []
        self.counts = {0: 0, 1: 0}      # per-batch totals, counted once at confirm

    def reset(self):
        self.pool = []
        self.counts = {0: 0, 1: 0}

    @staticmethod
    def _center(box):
        x1, y1, x2, y2 = box
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def _nearest(self, cx, cy):
        best, best_d = None, float("inf")
        for e in self.pool:
            if e["seen_this_frame"]:
                continue                # two pulp can't share a track
            ex, ey = self._center(e["box"])
            d = ((cx - ex) ** 2 + (cy - ey) ** 2) ** 0.5
            if d < self.proximity_px and d < best_d:
                best, best_d = e, d
        return best

    @staticmethod
    def _touches(box, line_y):
        return box[1] <= line_y <= box[3]

    def _evaluate(self, e, line_y):
        if not e["confirmed"] and e["hits"] >= self.confirm_frames:
            e["confirmed"] = True
            self.counts[e["class_id"]] += 1
            print(f"[CONFIRMED] {e['class_name']} (conf={e['confidence']:.2f}) "
                  f"after {e['hits']} frames")

        if (line_y is not None and e["class_id"] == REJECT_CLASS
                and self._touches(e["box"], line_y)):
            if not e["touched_line"]:
                print("[LINE] reject touched the trigger line")
            e["touched_line"] = True

        # Kick only once: confirmed AND touched the line.
        if (e["confirmed"] and e["class_id"] == REJECT_CLASS
                and e["touched_line"] and not e["actuator_fired"]):
            e["actuator_fired"] = True
            trigger_actuator()

    def update(self, detections, line_y=None):
        for e in self.pool:
            e["seen_this_frame"] = False

        for det in detections:
            cx, cy = self._center(det["box"])
            e = self._nearest(cx, cy)
            if e is not None:
                e["box"] = det["box"]
                e["confidence"] = det["confidence"]
                e["seen_this_frame"] = True
                e["missed"] = 0
                # Class is locked once confirmed so flicker can't flip it.
                if not e["confirmed"]:
                    e["class_id"] = det["class_id"]
                    e["class_name"] = det["class_name"]
                    e["hits"] += 1
            else:
                e = {"box": det["box"], "class_id": det["class_id"],
                     "class_name": det["class_name"],
                     "confidence": det["confidence"], "hits": 1,
                     "confirmed": False, "actuator_fired": False,
                     "touched_line": False, "missed": 0,
                     "seen_this_frame": True}
                self.pool.append(e)
            self._evaluate(e, line_y)

        for e in self.pool:
            if not e["seen_this_frame"]:
                e["missed"] += 1
        self.pool = [e for e in self.pool if e["missed"] <= self.max_missed]
        return self.pool


tracker = PulpTracker(CONFIRM_FRAMES, PROXIMITY_PX, MAX_MISSED)

# ─── Inference ───────────────────────────────────────────────────────────────
def get_crop_box(frame_w, frame_h):
    """Detection zone (x0, y0, w, h) in full-frame pixels. Same rule as capture.py."""
    s = frame_w / BASE_W
    cw = min(round(CROP_W * s), frame_w)
    ch = min(round(CROP_H * s), frame_h)
    x0 = (frame_w - cw) // 2 + round(CROP_OFFSET[0] * s)
    y0 = (frame_h - ch) // 2 + round(CROP_OFFSET[1] * s)
    x0 = max(0, min(x0, frame_w - cw))
    y0 = max(0, min(y0, frame_h - ch))
    return x0, y0, cw, ch


_last_score_print = 0.0
_last_crop_save = 0.0


def _debug_save_crop(rgb):
    """DEBUG_SCORES=1: save the exact image fed to the model (debug_crop.jpg, every 5 s)."""
    global _last_crop_save
    t = time.time()
    if t - _last_crop_save >= 5.0:
        _last_crop_save = t
        cv2.imwrite(os.path.join(HERE, "debug_crop.jpg"),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def _debug_scores(confs, class_ids):
    global _last_score_print
    t = time.time()
    if t - _last_score_print >= 1.0:
        _last_score_print = t
        k = int(np.argmax(confs))
        print(f"[SCORE] best raw conf {confs[k]:.3f} "
              f"({CLASSES.get(int(class_ids[k]), '?')}), threshold {CONF_THRESHOLD}")


def run_inference(frame_bgr):
    orig_h, orig_w = frame_bgr.shape[:2]
    cx0, cy0, cw, ch = get_crop_box(orig_w, orig_h)
    crop = frame_bgr[cy0:cy0 + ch, cx0:cx0 + cw]       # same view as training

    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    if (cw, ch) != (img_w, img_h):
        rgb = cv2.resize(rgb, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
    if DEBUG_SCORES:
        _debug_save_crop(rgb)
    x = rgb.astype(np.float32) / 255.0
    if channels_first:
        x = np.transpose(x, (2, 0, 1))
    interpreter.set_tensor(in_det["index"], np.expand_dims(np.ascontiguousarray(x), 0))
    interpreter.invoke()
    output = interpreter.get_tensor(out_det["index"])[0]

    scores_all = output[4:, :]
    class_ids = np.argmax(scores_all, axis=0)
    confs = scores_all[class_ids, np.arange(scores_all.shape[1])]
    if DEBUG_SCORES:
        _debug_scores(confs, class_ids)
    keep_mask = confs >= CONF_THRESHOLD
    if not np.any(keep_mask):
        return []

    xc, yc = output[0, keep_mask], output[1, keep_mask]
    w, h = output[2, keep_mask], output[3, keep_mask]
    confs, class_ids = confs[keep_mask], class_ids[keep_mask]

    # Boxes are normalised to the crop: scale by crop size, add crop position,
    # then divide by s so they are in base-resolution (640x480) units. The
    # tracker, proximity and trigger line all work in those units.
    s = orig_w / BASE_W
    x1 = ((xc - w / 2) * cw + cx0) / s
    y1 = ((yc - h / 2) * ch + cy0) / s
    bw, bh = (w * cw) / s, (h * ch) / s
    boxes = np.stack([x1, y1, bw, bh], axis=1).astype(int).tolist()

    # Class-agnostic NMS: one box per pulp even if the model is torn between classes.
    keep = cv2.dnn.NMSBoxes(boxes, confs.tolist(), CONF_THRESHOLD, NMS_THRESHOLD)
    dets = []
    for k in np.array(keep).flatten():
        cid = int(class_ids[k])
        bx, by, bwk, bhk = boxes[k]
        dets.append({"class_id": cid, "class_name": CLASSES.get(cid, "unknown"),
                     "confidence": round(float(confs[k]), 4),
                     "box": [bx, by, bx + bwk, by + bhk]})
    dets.sort(key=lambda d: d["confidence"], reverse=True)
    return dets


frame_queue = queue.Queue(maxsize=1)
detection_queue = queue.Queue(maxsize=1)


def inference_worker():
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        t0 = time.time()
        dets = run_inference(frame)
        inf_ms = (time.time() - t0) * 1000
        if not detection_queue.empty():
            try:
                detection_queue.get_nowait()
            except queue.Empty:
                pass
        detection_queue.put(dets)
        if dets:
            print(f"[DETECTIONS] {len(dets)} raw found (inference {inf_ms:.0f} ms)")
            for d in dets:
                print(f"  {d['class_name']} {d['confidence']:.2f}")


inference_thread = None
if interpreter is not None:
    inference_thread = threading.Thread(target=inference_worker, daemon=True)
    inference_thread.start()


def _drain(q):
    try:
        q.get_nowait()
    except queue.Empty:
        pass


def reopen_camera():
    """Release and reopen the webcam, re-apply locked controls. True if a frame reads."""
    global cap
    try:
        cap.release()
    except Exception:
        pass
    time.sleep(1.0)
    dev = os.path.realpath(CAM_BY_ID) if os.path.exists(CAM_BY_ID) else DEVICE_INDEX
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[CAMERA] reopen failed: {dev} did not open")
        return False
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_FPS,          FRAMERATE)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for ctrl, value in CAMERA_CONTROLS.items():
        subprocess.run(["v4l2-ctl", "-d", dev, f"--set-ctrl={ctrl}={value}"],
                       capture_output=True, text=True)
    time.sleep(LOCK_WAIT)
    ok, _ = cap.read()
    print(f"[CAMERA] reopened {dev}, first read {'ok' if ok else 'FAILED'}")
    return ok


# ─── State machine ───────────────────────────────────────────────────────────
#   IDLE      relays off, waiting for Start
#   FEEDING   cracker + dispenser + conveyor on, detection active.
#             Start switches to SORTING.
#   SORTING   phase "DISPENSE": dispenser + conveyor on for DISPENSER_SEC
#             phase "BELT":     conveyor alone on until EMPTY_WINDOW_SEC passes
#                               with no new confirmed pulp, then back to IDLE.
#             Start switches back to FEEDING and resets the SORTING timers.
state = "IDLE"
phase = ""                  # "DISPENSE" or "BELT" while state == "SORTING"
idle_title = ""             # only used for the "Camera Error" overlay on the Idle screen
dispenser_end = 0.0
belt_end = 0.0
last_confirm_t = 0.0        # last time a new pulp was confirmed (BELT stage)
last_total = 0              # tracker good+reject total at that time
tracked = []
fps_counter, fps_timer, current_fps = 0, time.time(), 0.0
last_lcd = 0.0
cam_fail_since = None
cam_next_reopen = 0.0
cam_paused = False
cam_paused_at = 0.0
shutdown_requested = False
estop_paused = False        # True while the emergency stop is pressed
estop_last = 0.0


def apply_outputs():
    """Set the relays for the current state (used after a camera pause or e-stop release)."""
    if estop_paused and state in ("FEEDING", "SORTING"):
        cracker_off()            # emergency stop: dispenser + cracker stay off,
        dispenser_off()          # the conveyor keeps running
        motors_on()
    elif state == "FEEDING":
        all_on()
    elif state == "SORTING":
        if phase == "DISPENSE":
            cracker_off()
            dispenser_on()
            motors_on()          # conveyor
        else:                    # BELT: conveyor only
            all_off()
            motors_on()
    else:
        all_off()


def begin_sorting():
    """Feeding stopped: cracker off, dispenser + conveyor keep running."""
    global state, phase, dispenser_end
    cracker_off()
    phase = "DISPENSE"
    dispenser_end = time.time() + DISPENSER_SEC
    state = "SORTING"


if lcd_ok:
    lcd_clear()

while not stop_event.is_set():

    # ── EMERGENCY STOP: dispenser + cracker only (conveyor keeps running) ───
    if estop_active():
        t_e = time.time()
        if not estop_paused:
            estop_paused = True
            estop_last = t_e
            print("[ESTOP] Pressed - dispenser and cracker off, conveyor keeps running")
            if state != "IDLE" and not cam_paused:
                cracker_off()
                dispenser_off()
        elif state != "IDLE" and not cam_paused:
            cracker_off()                    # re-assert every pass
            dispenser_off()
        if state == "SORTING" and phase == "DISPENSE":
            dispenser_end += t_e - estop_last    # freeze the dispenser countdown
        estop_last = t_e
    elif estop_paused:
        estop_paused = False
        print(f"[ESTOP] Released - resuming ({state})")
        if state != "IDLE" and not cam_paused:
            apply_outputs()                  # camera recovery restores them otherwise
        last_lcd = 0.0                       # redraw the status line immediately

    # ── IDLE ────────────────────────────────────────────────────────────────
    if state == "IDLE":
        if estop_paused:
            lcd_show("Emergency Stop", "Release to Start")
        else:
            lcd_show(idle_title or "System Ready", "Press Start")

        if shutdown_event(allow_hold=True) == "hold":
            print("[STATE] Shutdown button held - shutting down")
            shutdown_requested = True
            # Immediately replace the idle message; leave this on the LCD
            # while the motors, camera, and GPIO are being cleaned up.
            if lcd_ok:
                lcd_show("Shutting Down", "Please wait...")
            break

        if start_pressed() and not stop_event.is_set() and not estop_paused:
            print("[STATE] Start pressed - Feeding")
            tracker.reset()
            tracked = []
            clear_pending()
            _drain(frame_queue)
            _drain(detection_queue)
            if cap is not None:
                for _ in range(5):          # drop stale buffered frames
                    cap.grab()
            cam_fail_since, cam_paused = None, False
            idle_title = ""
            state = "FEEDING"
            all_on()
        else:
            time.sleep(0.05)
        continue

    # ── Shutdown button outside IDLE: poll and discard ──────────────────────
    # A hold that starts during a batch must never become a shutdown, even if
    # the button is still held when the batch ends (allow_hold=False).
    shutdown_event(allow_hold=False)

    # ── Start button during FEEDING = switch to SORTING ─────────────────────
    if state == "FEEDING" and start_pressed():
        print(f"[STATE] Start pressed - switching to Sorting: dispenser "
              f"{DISPENSER_SEC:.0f} s, then belt {BELT_SEC:.0f} s")
        begin_sorting()
        continue

    # ── Start button during SORTING = return to FEEDING + SORTING ────────────
    if state == "SORTING" and start_pressed():
        print("[STATE] Start pressed during Sorting - returning to Feeding+Sorting")
        # Reset both SORTING timers. The next Start press will begin a fresh
        # DISPENSE phase followed by a fresh BELT phase.
        phase = ""
        dispenser_end = 0.0
        belt_end = 0.0
        state = "FEEDING"
        # If the camera is paused, keep all relays off. apply_outputs() will
        # restore the correct FEEDING outputs after the camera recovers.
        if not cam_paused:
            apply_outputs()      # all on, except dispenser/cracker if e-stop is pressed
        continue

    # ── SORTING timers (frozen while the camera is paused) ──────────────────
    if state == "SORTING" and not cam_paused:
        t_now = time.time()
        if phase == "DISPENSE" and t_now >= dispenser_end:
            print("[STATE] Dispenser stopped - belt runs on its own")
            dispenser_off()
            phase = "BELT"
            belt_end = t_now + BELT_SEC          # only used if there is no camera
            last_confirm_t = t_now               # empty window starts only now
            last_total = tracker.counts[0] + tracker.counts[1]
        elif phase == "BELT":
            total = tracker.counts[0] + tracker.counts[1]
            if total > last_total:               # a new pulp was confirmed
                last_total = total
                last_confirm_t = t_now
            if cap is not None:
                finished = t_now - last_confirm_t >= EMPTY_WINDOW_SEC
            else:
                finished = t_now >= belt_end     # no sensor: fixed timer
            if finished:
                print(f"[STATE] No confirmed pulp for {EMPTY_WINDOW_SEC:.0f} s "
                      f"- belt stopped, back to Idle")
                all_off()
                state = "IDLE"
                continue

    # ── Frame grab (FEEDING, SORTING) ───────────────────────────────────────
    frame = None
    if cap is not None:
        ret, frame = cap.read()
        if not ret:
            frame = None
            now_f = time.time()
            if cam_fail_since is None:
                cam_fail_since = now_f
                cam_next_reopen = CAM_REOPEN_SEC
                print("[WARN] Frame grab failed - retrying")
            failed_for = now_f - cam_fail_since
            if failed_for >= CAM_PAUSE_SEC and not cam_paused:
                print(f"[CAMERA] no frame for {failed_for:.1f} s - belt paused")
                motors_off()
                cam_paused, cam_paused_at = True, now_f
                lcd_show("Camera problem", "Belt paused")
            if failed_for >= cam_next_reopen:
                cam_next_reopen += 4.0
                reopen_camera()
            if failed_for >= CAM_ABORT_SEC:
                log_error("[CAMERA] still no frame - aborting batch")
                motors_off()
                tracker.pool, tracked = [], []
                cam_fail_since, cam_paused = None, False
                clear_pending()
                idle_title = "Camera Error"
                state = "IDLE"
                continue
            time.sleep(0.05)
            continue

        if cam_fail_since is not None:
            print(f"[CAMERA] recovered after {time.time() - cam_fail_since:.1f} s")
            cam_fail_since = None
            if cam_paused:
                if state == "SORTING":
                    if phase == "DISPENSE":      # pause: don't lose dispenser time
                        dispenser_end += time.time() - cam_paused_at
                    else:                        # belt-only stage: restart the window
                        belt_end = time.time() + BELT_SEC
                        last_confirm_t = time.time()
                        last_total = tracker.counts[0] + tracker.counts[1]
                apply_outputs()
                cam_paused = False

    # ── Detection ───────────────────────────────────────────────────────────
    if frame is not None:
        if frame_queue.empty():
            frame_queue.put(frame.copy())
        s = frame.shape[1] / BASE_W          # frame pixels per base pixel
        zx0, zy0, zw, zh = get_crop_box(frame.shape[1], frame.shape[0])
        line_y = zy0 / s + TRIGGER_LINE_OFFSET    # base units, like the boxes

        if not detection_queue.empty():
            tracked = tracker.update(detection_queue.get_nowait(), line_y)

        for b in tracked:
            x1, y1, x2, y2 = (int(v * s) for v in b["box"])   # base -> frame pixels
            if not b["confirmed"]:
                color = COLOR_PENDING
                status = f"? {b['class_name']} {b['confidence']:.2f} ({b['hits']}/{CONFIRM_FRAMES})"
            elif b["class_id"] == REJECT_CLASS:
                color = COLOR_REJECT
                status = f"REJECT {b['confidence']:.2f}"
            else:
                color = COLOR_GOOD
                status = f"OK {b['class_name']} {b['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, status, (x1, max(y1 - 10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

        fps_counter += 1
        if time.time() - fps_timer >= 1.0:
            current_fps = fps_counter / (time.time() - fps_timer)
            fps_counter, fps_timer = 0, time.time()
            print(f"[FPS] {current_fps:.1f}")

    good_n, rej_n = tracker.counts[0], tracker.counts[1]

    # ── LCD: FEEDING / SORTING status (camera errors overlay it) ────────────
    # 16 columns max.
    now = time.time()
    if not cam_paused and now - last_lcd >= LCD_INTERVAL:
        last_lcd = now
        if state == "FEEDING":
            lcd_show("Emergency Stop" if estop_paused else "Feeding",
                     f"G:{good_n} R:{rej_n}")
        else:                                   # SORTING (both stages)
            lcd_show("Emergency Stop" if estop_paused else "Sorting",
                     f"G:{good_n} R:{rej_n}")

    if SHOW_PREVIEW and frame is not None:
        pending_n = sum(1 for b in tracked if not b["confirmed"])
        cv2.putText(frame, f"Confirmed G:{good_n} R:{rej_n}  Pending: {pending_n}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.rectangle(frame, (zx0, zy0), (zx0 + zw, zy0 + zh), (255, 255, 0), 1)
        line_px = int(line_y * s)
        cv2.line(frame, (zx0, line_px), (zx0 + zw, line_px), (255, 255, 255), 2)
        view = frame
        if frame.shape[1] > PREVIEW_WIDTH:      # display only; detection uses the full frame
            sc = PREVIEW_WIDTH / frame.shape[1]
            view = cv2.resize(frame, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
        cv2.imshow("Cacao Pulp Detection", view)
        if cv2.waitKey(1) == ord("q"):
            break

    if cap is None:
        time.sleep(0.02)    # no-model mode: nothing to read, don't spin

# ─── Cleanup (GPIO.cleanup must be last) ─────────────────────────────────────
stop_event.set()
if inference_thread is not None:
    inference_thread.join(timeout=3.0)
motors_off()
cleanup_motors()
cleanup_rejector()
if cap is not None:
    cap.release()
if SHOW_PREVIEW:
    cv2.destroyAllWindows()
if lcd_ok:
    if shutdown_requested:
        # Do not clear the LCD: keep the shutdown message visible until the
        # Raspberry Pi powers off.
        lcd_show("Shutting Down", "Please wait...")
    else:
        lcd_clear()
        lcd_show("Stopped")
cleanup_buttons()
print("Stopped")

if shutdown_requested:
    # Needs root (service running as root) or a passwordless sudo rule.
    r = subprocess.run(["sudo", "-n", "shutdown", "-h", "now"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        log_error(f"[SHUTDOWN] failed: {r.stderr.strip()}")
