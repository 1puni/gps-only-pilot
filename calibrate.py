#!/usr/bin/env python3
"""
Interactive bench/dockside calibration for the standalone GPS-only autopilot.

Drives the BTS7960 motor directly (bypassing pypilot) and asks the operator
to observe the physical rudder, so we can record, per GPS-ONLY-NAV.md's
"Rudder model":

  - which command sign drives which physical direction (port/starboard)
  - full hard-over travel time in each direction (the reference point that
    resets the software travel budget)
  - a rough degrees-per-second constant, from timed pulses matched to the
    control loop's pulse table (100/200/300ms)

Results are written to calibration.json. Nothing here reads GPS or MQTT;
it only exists to produce the constants the autopilot script will consume.

Run on the Pi with pigpiod already running (`sudo pigpiod`).  Keep a hand on
the physical kill switch throughout -- there is no rudder position sensor,
current sensing, or end-stop feedback, so the only thing preventing a jam
from being driven into is the operator and the safety timeout below.
"""
import json
import select
import sys
import time
from pathlib import Path

from vendor.bts7960_servo import BTS7960Config, BTS7960ServoDriver

CALIBRATION_PATH = Path(__file__).parent / "calibration.json"

# Matches the pulse durations in GPS-ONLY-NAV.md's control loop table.
PULSE_DURATIONS_MS = [100, 200, 300]

# Continuous hard-over drive should take well under this on any real rudder
# linkage; abort and disengage if it doesn't, rather than keep driving into
# a jam because the operator forgot to press Enter.
HARD_OVER_SAFETY_TIMEOUT_S = 20.0

# How often to refresh the driver's command while holding a continuous
# drive. Must be well under BTS7960Config.watchdog_seconds (default 1.0s).
DRIVE_REFRESH_S = 0.1


def load_calibration():
    if CALIBRATION_PATH.exists():
        return json.loads(CALIBRATION_PATH.read_text())
    return {}


def save_calibration(data):
    CALIBRATION_PATH.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    print(f"saved -> {CALIBRATION_PATH}")


def confirm(prompt):
    return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")


def wait_for_enter(timeout):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return False
    sys.stdin.readline()
    return True


def drive_until_stopped(driver, sign, safety_timeout):
    """Holds command(sign) until Enter is pressed or safety_timeout elapses.

    Returns (elapsed_seconds, hit_safety_timeout).
    """
    start = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= safety_timeout:
                return elapsed, True
            driver.command(sign)
            if wait_for_enter(min(DRIVE_REFRESH_S, safety_timeout - elapsed)):
                return time.monotonic() - start, False
    finally:
        driver.disengage()


def pulse(driver, sign, duration_ms):
    start = time.monotonic()
    end = start + duration_ms / 1000.0
    try:
        while time.monotonic() < end:
            driver.command(sign)
            time.sleep(min(DRIVE_REFRESH_S, duration_ms / 1000.0))
    finally:
        driver.disengage()


def step_direction_check(driver, calibration):
    print("\n=== direction check ===")
    print("Driving command(+1) briefly at low duty. Watch the rudder.")
    if not confirm("Ready?"):
        return
    pulse(driver, 1, 300)
    positive = input("Which way did it move? (port/starboard/none): ").strip().lower()

    print("\nDriving command(-1) briefly.")
    if not confirm("Ready?"):
        return
    pulse(driver, -1, 300)
    negative = input("Which way did it move? (port/starboard/none): ").strip().lower()

    calibration["direction_map"] = {
        "positive_command": positive,
        "negative_command": negative,
    }
    save_calibration(calibration)


def step_hard_over(driver, calibration, sign, label):
    print(f"\n=== hard-over timing: {label} (command sign {sign:+d}) ===")
    print("Center the rudder first if it isn't already.")
    print(f"Will drive continuously until you press Enter, or {HARD_OVER_SAFETY_TIMEOUT_S:.0f}s safety cutoff.")
    print("Press Enter the instant the rudder hits its mechanical stop.")
    if not confirm("Start?"):
        return

    elapsed, hit_timeout = drive_until_stopped(driver, sign, HARD_OVER_SAFETY_TIMEOUT_S)

    if hit_timeout:
        print(f"!! safety timeout hit at {elapsed:.2f}s -- driver disengaged. Not saved.")
        print("!! check for a mechanical jam before retrying.")
        return

    print(f"hard-over took {elapsed:.2f}s")
    calibration.setdefault("hard_over_seconds", {})[label] = round(elapsed, 2)
    save_calibration(calibration)


def step_pulse_table(driver, calibration, sign, label):
    print(f"\n=== pulse/angle table: {label} (command sign {sign:+d}) ===")
    print("Center the rudder first if it isn't already -- this builds a table")
    print("from a known zero, driving the same pulse widths the autopilot uses.")
    if not confirm("Ready?"):
        return

    samples = []
    cumulative_ms = 0
    for duration_ms in PULSE_DURATIONS_MS:
        input(f"Press Enter to fire a {duration_ms}ms pulse...")
        pulse(driver, sign, duration_ms)
        cumulative_ms += duration_ms
        angle_deg = float(input(f"Rudder angle now (deg, cumulative pulse time {cumulative_ms}ms): "))
        samples.append({"cumulative_ms": cumulative_ms, "angle_deg": angle_deg})

    calibration.setdefault("pulse_samples", {})[label] = samples
    deg_per_second = estimate_deg_per_second(samples)
    if deg_per_second is not None:
        calibration.setdefault("deg_per_second_estimate", {})[label] = round(deg_per_second, 3)
        print(f"estimated {deg_per_second:.3f} deg/sec for {label}")
    save_calibration(calibration)


def estimate_deg_per_second(samples):
    if len(samples) < 2:
        return None
    first, last = samples[0], samples[-1]
    dt = (last["cumulative_ms"] - first["cumulative_ms"]) / 1000.0
    if dt <= 0:
        return None
    return (last["angle_deg"] - first["angle_deg"]) / dt


def print_summary(calibration):
    print("\n=== current calibration.json ===")
    print(json.dumps(calibration, indent=2, sort_keys=True))


def main():
    calibration = load_calibration()
    calibration.setdefault("duty_cap_used", None)

    print("GPS-only autopilot: rudder calibration")
    print("Keep a hand on the physical kill switch for all of this.\n")

    config = BTS7960Config()
    calibration["duty_cap_used"] = round(config.max_duty, 3)

    driver = BTS7960ServoDriver(config)
    try:
        menu = {
            "1": ("direction check (which sign drives which side)",
                  lambda: step_direction_check(driver, calibration)),
            "2": ("hard-over timing, command(+1)",
                  lambda: step_hard_over(driver, calibration, 1, "positive")),
            "3": ("hard-over timing, command(-1)",
                  lambda: step_hard_over(driver, calibration, -1, "negative")),
            "4": ("pulse/angle table, command(+1)",
                  lambda: step_pulse_table(driver, calibration, 1, "positive")),
            "5": ("pulse/angle table, command(-1)",
                  lambda: step_pulse_table(driver, calibration, -1, "negative")),
            "6": ("show current calibration.json",
                  lambda: print_summary(calibration)),
        }

        while True:
            print("\n--- menu ---")
            for key, (description, _) in menu.items():
                print(f"  {key}) {description}")
            print("  q) quit")
            choice = input("> ").strip().lower()
            if choice == "q":
                break
            action = menu.get(choice)
            if action is None:
                print("unknown choice")
                continue
            try:
                action[1]()
            except KeyboardInterrupt:
                print("\ninterrupted -- stopping.")
                driver.disengage()
            except Exception as exc:
                print(f"error: {exc} -- stopping.")
                driver.disengage()
    finally:
        driver.close()
        print("\ndriver closed, all outputs low.")


if __name__ == "__main__":
    sys.exit(main())
