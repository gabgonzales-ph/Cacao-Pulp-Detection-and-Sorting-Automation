# components/buttons.py
# Start button on GPIO27 (one leg to the pin, one leg to GND, internal pull-up).
# The Safe Shutdown button (GPIO17) is handled by the kernel gpio-shutdown
# overlay. NEVER read GPIO17 here.

import time
import RPi.GPIO as GPIO

START_BUTTON_PIN = 27
DEBOUNCE_SEC = 0.05


def setup_buttons():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def start_pressed():
    """Non-blocking. True once per physical press (debounced, waits for release)."""
    if GPIO.input(START_BUTTON_PIN) != GPIO.LOW:
        return False
    time.sleep(DEBOUNCE_SEC)
    if GPIO.input(START_BUTTON_PIN) != GPIO.LOW:
        return False
    # Wait for release so one press cannot trigger twice
    while GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.01)
    time.sleep(DEBOUNCE_SEC)
    return True


def cleanup_buttons():
    """Calls GPIO.cleanup(). Must be the LAST thing in the cleanup sequence."""
    GPIO.cleanup()
