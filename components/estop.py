# components/estop.py: emergency stop button.
#
# Set ESTOP_NORMALLY_CLOSED to match the contact you actually wired:
#
#   False (current): plain push button / NO contact between GPIO5 (pin 29) and
#       GND (pin 30), internal pull-up.
#         Not pressed -> pin HIGH -> run
#         Pressed     -> pin LOW  -> STOP
#       NOTE: a broken or unplugged wire is NOT detected (it reads as "run").
#
#   True: NC contact between the same pins.
#         Not pressed -> contact closed -> pin LOW  -> run
#         Pressed     -> contact open   -> pin HIGH -> STOP
#         Wire broken -> pin HIGH                   -> STOP (fail-safe)
#
# Stop is immediate (no debounce). Release must read the released level for
# ESTOP_RELEASE_SEC before it counts, so contact bounce can't restart the motors.
# This module does not call GPIO.cleanup(); cleanup_buttons() does that last.

import time

import RPi.GPIO as GPIO

ESTOP_PIN = 5
ESTOP_NORMALLY_CLOSED = False
ESTOP_RELEASE_SEC = 0.05

_PRESSED_LEVEL = GPIO.HIGH if ESTOP_NORMALLY_CLOSED else GPIO.LOW

_active = True          # assume stopped until the pin proves healthy
_released_since = None


def setup_estop():
    global _active, _released_since
    if GPIO.getmode() is None:
        GPIO.setmode(GPIO.BCM)
    GPIO.setup(ESTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    _active = GPIO.input(ESTOP_PIN) == _PRESSED_LEVEL
    _released_since = None if _active else time.time()


def estop_active():
    """True while the emergency stop is pressed.
    Non-blocking: call it every loop pass."""
    global _active, _released_since
    if GPIO.input(ESTOP_PIN) == _PRESSED_LEVEL:
        _active = True
        _released_since = None
    elif _active:
        now = time.time()
        if _released_since is None:
            _released_since = now
        elif now - _released_since >= ESTOP_RELEASE_SEC:
            _active = False
            _released_since = None
    return _active
