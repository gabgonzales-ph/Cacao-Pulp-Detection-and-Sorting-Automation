# components/motors.py
# Single relay (GPIO23) driving the conveyor motor. Active LOW:
# pin LOW = relay ON (motor runs), pin HIGH = relay OFF.
# If the motor acts backwards, flip ACTIVE_LOW.

import RPi.GPIO as GPIO

RELAY_PIN = 23
ACTIVE_LOW = True

_ON = GPIO.LOW if ACTIVE_LOW else GPIO.HIGH
_OFF = GPIO.HIGH if ACTIVE_LOW else GPIO.LOW


def setup_motors():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RELAY_PIN, GPIO.OUT, initial=_OFF)


def motors_on():
    GPIO.output(RELAY_PIN, _ON)


def motors_off():
    GPIO.output(RELAY_PIN, _OFF)


def cleanup_motors():
    """Relay off. Does NOT call GPIO.cleanup() (buttons.cleanup_buttons does, last)."""
    try:
        GPIO.output(RELAY_PIN, _OFF)
    except Exception:
        pass
