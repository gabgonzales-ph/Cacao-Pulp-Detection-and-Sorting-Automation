# components/buttons.py
# Start button    : GPIO27 (one leg to the pin, one leg to GND, internal pull-up).
# Shutdown button : GPIO17 (same wiring). Read HERE, so the kernel gpio-shutdown
#                   overlay must be removed from /boot/firmware/config.txt,
#                   otherwise any press powers the Pi off.
#
# Both buttons are polled and never block. Call start_pressed() and
# shutdown_event() on every pass of the main loop.

import time
import RPi.GPIO as GPIO

START_BUTTON_PIN = 27
SHUTDOWN_BUTTON_PIN = 17
DEBOUNCE_SEC = 0.05
SHUTDOWN_HOLD_SEC = 2.0

# Start-button state
_start_pending_since = None   # when the pin first read LOW (debounce timer)
_start_latched = False        # press already reported; wait for release

# Shutdown-button state
_pending_since = None     # when the pin first read LOW (debounce timer)
_down_since = None        # when a debounced press began
_hold_armed = False       # this press may turn into a shutdown hold
_hold_reported = False


def setup_buttons():
    global _start_latched
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(SHUTDOWN_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # If Start is already held (or shorted) at boot, require a release first.
    _start_latched = GPIO.input(START_BUTTON_PIN) == GPIO.LOW


def start_pressed():
    """Non-blocking. True once per physical press, when a debounced press begins.
    The button must be released before it can fire again."""
    global _start_pending_since, _start_latched
    now = time.time()

    if GPIO.input(START_BUTTON_PIN) != GPIO.LOW:           # released
        _start_pending_since = None
        _start_latched = False
        return False

    if _start_latched:                                     # still held, already reported
        return False

    if _start_pending_since is None:
        _start_pending_since = now
        return False
    if now - _start_pending_since >= DEBOUNCE_SEC:
        _start_latched = True
        _start_pending_since = None
        return True
    return False


def shutdown_event(allow_hold):
    """Poll the shutdown button. Never blocks. Returns:
         "press" - once, when a debounced press begins
         "hold"  - once, after SHUTDOWN_HOLD_SEC of holding, but only if the
                   press began while allow_hold was True (i.e. while idle) and
                   allow_hold is still True now
         None    - nothing new
    Call it on EVERY loop pass (with allow_hold=False outside IDLE) so a press
    that starts during a batch is recorded as such and can never become a
    shutdown, even if the batch ends while the button is still held."""
    global _pending_since, _down_since, _hold_armed, _hold_reported
    now = time.time()

    if GPIO.input(SHUTDOWN_BUTTON_PIN) != GPIO.LOW:        # released
        _pending_since = _down_since = None
        _hold_armed = _hold_reported = False
        return None

    if _down_since is None:                                # not yet confirmed
        if _pending_since is None:
            _pending_since = now
        elif now - _pending_since >= DEBOUNCE_SEC:
            _down_since = now
            _hold_armed = allow_hold
            _hold_reported = False
            return "press"
        return None

    if (allow_hold and _hold_armed and not _hold_reported
            and now - _down_since >= SHUTDOWN_HOLD_SEC):
        _hold_reported = True
        return "hold"
    return None


def cleanup_buttons():
    """Calls GPIO.cleanup(). Must be the LAST thing in the cleanup sequence."""
    GPIO.cleanup()
