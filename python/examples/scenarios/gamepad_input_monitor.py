#!/usr/bin/env python3
"""gamepad_input_monitor: print named buttons and sticks from gamepad.latest()."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _cli import (
    catch_sigint,
    create_session_or_exit,
    parse_session_args,
    read_finite_float,
    stop_requested,
    wants_help,
)

BUTTONS = ("LB", "RB", "A", "B", "X", "Y", "Back", "Start", "Up", "Down", "Left", "Right")
AXES = ("LT", "RT", "LeftStickX", "LeftStickY", "RightStickX", "RightStickY")


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--watch] [--interval SEC] [--json]\n"
        "  One sample by default. --watch repeats until Ctrl+C.",
        file=sys.stderr,
    )


def print_pad(pad, as_json):
    if pad is None:
        if as_json:
            print('{"sample": false}')
        else:
            print("gamepad=(no sample)")
            print(
                "hint: confirm the robot is publishing gamepad data, --domain-id matches ROS_DOMAIN_ID, "
                "and the NIC selection can see the robot."
            )
        return
    if as_json:
        buttons = ", ".join(f'"{name}": {pad.digital_states[i]}' for i, name in enumerate(BUTTONS))
        axes = ", ".join(f'"{name}": {pad.analog_states[i]}' for i, name in enumerate(AXES))
        connected = "true" if pad.hardware_connected else "false"
        print(
            '{"sample": true, "hardware_connected": '
            + connected
            + ', "buttons": {'
            + buttons
            + '}, "axes": {'
            + axes
            + "}}"
        )
        return
    print(f"hardware_connected={'true' if pad.hardware_connected else 'false'}")
    if not pad.hardware_connected:
        print("hint: gamepad sample arrived but hardware_connected is false. Check the physical pad.")
    print(" ".join(f"{name}={pad.digital_states[i]}" for i, name in enumerate(BUTTONS)))
    print(" ".join(f"{name}={pad.analog_states[i]}" for i, name in enumerate(AXES)))


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    watch = False
    as_json = False
    interval = 0.2
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--watch":
            watch = True
        elif arg == "--json":
            as_json = True
        elif arg == "--interval" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "interval")
            if parsed is None:
                return 2
            interval = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if interval <= 0.0:
        print("interval must be > 0", file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    discover = time.monotonic() + 5.0
    while time.monotonic() < discover and session.gamepad.latest() is None:
        time.sleep(0.02)
    catch_sigint()
    saw_sample = False
    while True:
        pad = session.gamepad.latest()
        if pad is not None:
            saw_sample = True
        print_pad(pad, as_json)
        if not watch or stop_requested():
            break
        time.sleep(interval)
    session.close()
    return 0 if saw_sample else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
