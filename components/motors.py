# components/motors.py
# Three relays, all active LOW: pin LOW = relay ON, pin HIGH = relay OFF.
#   Conveyor  GPIO23 (pin 16)  belt motor
#   Cracker   GPIO24 (pin 18)  cracking machine
#   Dispenser GPIO25 (pin 22)  pulp dispenser
# motors_on()/motors_off() control ONLY the conveyor (capture.py relies on this).
# If a relay acts backwards, flip ACTIVE_LOW.

import RPi.GPIO as GPIO

CONVEYOR_PIN = 23
CRACKER_PIN = 24
DISPENSER_PIN = 25
ACTIVE_LOW = True

_ON = GPIO.LOW if ACTIVE_LOW else GPIO.HIGH
_OFF = GPIO.HIGH if ACTIVE_LOW else GPIO.LOW

_ALL_PINS = (CONVEYOR_PIN, CRACKER_PIN, DISPENSER_PIN)


def setup_motors():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in _ALL_PINS:
        GPIO.setup(pin, GPIO.OUT, initial=_OFF)   # all OFF at start


# ── Per-relay control ───────────────────────────────────────────────────────
def conveyor_on():
    GPIO.output(CONVEYOR_PIN, _ON)


def conveyor_off():
    GPIO.output(CONVEYOR_PIN, _OFF)


def cracker_on():
    GPIO.output(CRACKER_PIN, _ON)


def cracker_off():
    GPIO.output(CRACKER_PIN, _OFF)


def dispenser_on():
    GPIO.output(DISPENSER_PIN, _ON)


def dispenser_off():
    GPIO.output(DISPENSER_PIN, _OFF)


# ── Conveyor only (kept for capture.py) ─────────────────────────────────────
def motors_on():
    conveyor_on()


def motors_off():
    conveyor_off()


# ── All relays ──────────────────────────────────────────────────────────────
def all_on():
    for pin in _ALL_PINS:
        GPIO.output(pin, _ON)


def all_off():
    for pin in _ALL_PINS:
        GPIO.output(pin, _OFF)


def cleanup_motors():
    """All relays off. Does NOT call GPIO.cleanup() (buttons.cleanup_buttons does, last)."""
    for pin in _ALL_PINS:
        try:
            GPIO.output(pin, _OFF)
        except Exception:
            pass
