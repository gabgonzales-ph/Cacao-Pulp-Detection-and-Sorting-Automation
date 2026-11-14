# components/rejector.py
# Reject servo on GPIO12 (software PWM, 50 Hz).
# Rejects are queued with an optional delay (camera-to-kicker travel time) and
# fired one at a time by a single worker thread, so kicks can never overlap.
#
# TUNE on the mounted arm: REST_ANGLE, KICK_ANGLE, MOVE_SEC, HOLD_SEC.

import threading
import time
import RPi.GPIO as GPIO

SERVO_PIN = 12
REST_ANGLE = 0      # arm clear of the belt
KICK_ANGLE = 90     # arm pushed across the belt
MOVE_SEC = 0.20     # time allowed for the arm to travel
HOLD_SEC = 0.15     # time held at the kick position

_pwm = None
_queue = []                 # due times (time.monotonic())
_lock = threading.Lock()
_stop = threading.Event()
_worker = None


def _duty(angle):
    return 2.5 + angle / 18.0   # 0 deg = 2.5, 90 deg = 7.5, 180 deg = 12.5


def _move(angle):
    _pwm.ChangeDutyCycle(_duty(angle))
    time.sleep(MOVE_SEC)
    _pwm.ChangeDutyCycle(0)     # stop PWM to avoid jitter at rest


def _kick():
    _move(KICK_ANGLE)
    time.sleep(HOLD_SEC)
    _move(REST_ANGLE)


def _run():
    while not _stop.is_set():
        due = False
        with _lock:
            now = time.monotonic()
            for t in _queue:
                if t <= now:
                    _queue.remove(t)
                    due = True
                    break
        if due:
            _kick()
        else:
            time.sleep(0.01)


def setup_rejector():
    global _pwm, _worker
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    _pwm = GPIO.PWM(SERVO_PIN, 50)
    _pwm.start(0)
    _move(REST_ANGLE)
    _stop.clear()
    _worker = threading.Thread(target=_run, daemon=True)
    _worker.start()


def schedule_kick(delay_sec=0.0):
    """Queue one kick to fire after delay_sec. Returns immediately."""
    with _lock:
        _queue.append(time.monotonic() + max(0.0, delay_sec))


def pending_kicks():
    with _lock:
        return len(_queue)


def clear_pending():
    with _lock:
        _queue.clear()


def kick_reject():
    """Immediate blocking kick (used by test_rejector.py)."""
    _kick()


def cleanup_rejector():
    """Stop the worker, return to rest, stop PWM. Does NOT call GPIO.cleanup()."""
    global _pwm
    _stop.set()
    clear_pending()
    if _worker is not None:
        _worker.join(timeout=2.0)
    try:
        if _pwm is not None:
            _move(REST_ANGLE)
            _pwm.stop()
            _pwm = None
    except Exception:
        pass
