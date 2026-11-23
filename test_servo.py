#!/usr/bin/env python3
"""
test_servo.py - interactive servo tester for the reject kicker (SG90, GPIO12).

Run on the Pi with the sorter service stopped (it owns GPIO12):
    sudo systemctl stop cacao-sorter.service
    cd ~/cacao_pulp && source venv313/bin/activate
    python test_servo.py                # positional servo (angle control)
    python test_servo.py --continuous   # 360-degree continuous-rotation servo

Positional mode  : the signal sets an ANGLE (same maths as rejector.py).
Continuous mode  : a "360" SG90 does NOT hold angles. The signal sets SPEED and
                   DIRECTION (1.5 ms = stop), so a "kick" is a timed spin out and
                   a timed spin back. Use `cal` to measure degrees-per-second.

Keep the arm clear of the belt and fingers while testing. Ctrl+C or `q` quits.
"""
import argparse
import time

import RPi.GPIO as GPIO

SERVO_PIN = 12
FREQ_HZ = 50
PERIOD_US = 1_000_000 / FREQ_HZ  # 20 000 us

# Defaults mirror components/rejector.py
REST_ANGLE = 90
KICK_ANGLE = 0
MOVE_SEC = 0.20
HOLD_SEC = 0.15


def duty_from_angle(angle):
    """Same formula as rejector._duty (2.5%..12.5% for 0..180 deg)."""
    return 2.5 + angle / 18.0


def duty_from_us(us):
    return us / PERIOD_US * 100.0


# ----------------------------------------------------------------- positional
def run_positional(pwm):
    st = {"rest": REST_ANGLE, "kick": KICK_ANGLE, "move": MOVE_SEC,
          "hold": HOLD_SEC, "now": None}

    def go(angle):
        angle = max(0.0, min(180.0, float(angle)))
        pwm.ChangeDutyCycle(duty_from_angle(angle))
        time.sleep(st["move"])
        pwm.ChangeDutyCycle(0)  # stop pulses -> no buzzing/jitter
        st["now"] = angle

    def kick():
        go(st["kick"])
        time.sleep(st["hold"])
        go(st["rest"])

    print(__doc__.split("Positional mode")[0].strip().splitlines()[0])
    print("""
Commands:
  <0-180>        move to that angle        k          do one kick (rest>kick>rest)
  kick <deg>     set KICK_ANGLE            rest <deg> set REST_ANGLE
  move <sec>     set settle time           hold <sec> set hold time at kick
  + / -          nudge current angle +/-5  sweep      slow 0>180>0 sweep
  show           print settings            q          quit
""")
    go(st["rest"])
    while True:
        raw = input(f"[pos={st['now']}] > ").strip().lower()
        if not raw:
            continue
        cmd, *args = raw.split()
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "k":
                kick()
            elif cmd in ("kick", "rest"):
                st[cmd] = float(args[0])
                print(f"{cmd.upper()}_ANGLE = {st[cmd]}")
            elif cmd in ("move", "hold"):
                st[cmd] = float(args[0])
                print(f"{cmd.upper()}_SEC = {st[cmd]}")
            elif cmd == "+":
                go((st["now"] or 0) + 5)
            elif cmd == "-":
                go((st["now"] or 0) - 5)
            elif cmd == "sweep":
                for a in list(range(0, 181, 10)) + list(range(170, -1, -10)):
                    go(a)
            elif cmd == "show":
                print(st)
            else:
                go(float(cmd))
        except (ValueError, IndexError):
            print("? bad input - try: 90, k, kick 75, hold 0.2")
    go(st["rest"])
    print("\nPaste into components/rejector.py:")
    print(f"REST_ANGLE = {st['rest']:g}\nKICK_ANGLE = {st['kick']:g}\n"
          f"MOVE_SEC   = {st['move']:g}\nHOLD_SEC   = {st['hold']:g}")


# ----------------------------------------------------------------- continuous
def run_continuous(pwm):
    st = {"trim": 0, "speed": 40, "deg_per_sec": None}

    def stop():
        pwm.ChangeDutyCycle(duty_from_us(1500 + st["trim"]))

    def spin(direction, sec):
        """direction: +1 / -1. Speed 100 = +/-500 us from neutral."""
        us = 1500 + st["trim"] + direction * st["speed"] / 100.0 * 500
        pwm.ChangeDutyCycle(duty_from_us(us))
        time.sleep(sec)
        stop()

    print("360-degree continuous-rotation mode")
    print("""
Commands:
  cw <sec> / ccw <sec>   spin for that many seconds
  speed <1-100>          set speed (default 40)
  trim <us>              fix creep at stop, e.g. trim 20 / trim -15
  stop                   send neutral pulse     off   stop sending pulses
  cal <sec>              spin cw, you enter the degrees it turned
  deg <n>                turn n degrees (+ cw, - ccw) using calibration
  kick <n> <hold>        turn out n deg, wait hold sec, turn back
  q                      quit
""")
    stop()
    while True:
        raw = input("> ").strip().lower()
        if not raw:
            continue
        cmd, *args = raw.split()
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("cw", "ccw"):
                spin(1 if cmd == "cw" else -1, float(args[0]))
            elif cmd == "speed":
                st["speed"] = max(1, min(100, float(args[0])))
                st["deg_per_sec"] = None
                print("speed set - recalibrate with `cal`")
            elif cmd == "trim":
                st["trim"] = float(args[0])
                stop()
            elif cmd == "stop":
                stop()
            elif cmd == "off":
                pwm.ChangeDutyCycle(0)
            elif cmd == "cal":
                sec = float(args[0])
                spin(1, sec)
                deg = float(input("How many degrees did the horn turn? "))
                st["deg_per_sec"] = deg / sec
                print(f"deg/sec = {st['deg_per_sec']:.1f} at speed {st['speed']:g}")
            elif cmd in ("deg", "kick"):
                if not st["deg_per_sec"]:
                    print("Run `cal <sec>` first.")
                    continue
                deg = float(args[0])
                sec = abs(deg) / st["deg_per_sec"]
                d = 1 if deg > 0 else -1
                spin(d, sec)
                if cmd == "kick":
                    time.sleep(float(args[1]) if len(args) > 1 else HOLD_SEC)
                    spin(-d, sec)
                print(f"spin time per leg: {sec:.3f}s")
            else:
                print("? unknown command")
        except (ValueError, IndexError):
            print("? bad input")
    pwm.ChangeDutyCycle(0)
    print(f"\nFinal: speed={st['speed']:g} trim={st['trim']:g}us "
          f"deg_per_sec={st['deg_per_sec']}")


def main():
    ap = argparse.ArgumentParser(description="SG90 reject-kicker servo tester")
    ap.add_argument("--continuous", action="store_true",
                    help="360-degree continuous-rotation servo (speed control)")
    ap.add_argument("--pin", type=int, default=SERVO_PIN, help="BCM pin (default 12)")
    args = ap.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(args.pin, GPIO.OUT)
    pwm = GPIO.PWM(args.pin, FREQ_HZ)
    pwm.start(0)
    try:
        (run_continuous if args.continuous else run_positional)(pwm)
    except (KeyboardInterrupt, EOFError):
        print("\nInterrupted.")
    finally:
        pwm.ChangeDutyCycle(0)
        pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
