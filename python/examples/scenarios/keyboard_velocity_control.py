#!/usr/bin/env python3
"""keyboard_velocity_control: low-speed WASD velocity on this terminal."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import BodyVelocityCommand

from _cli import (
    TerminalRaw,
    catch_sigint,
    check_ok,
    create_session_or_exit,
    motion_or_none,
    parse_session_args,
    read_finite_float,
    pause_before_close,
    print_no_robot_data_hint,
    request_stop,
    stop_requested,
    wait_for_motion_sample,
    wants_help,
)


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--max-x V] [--max-y V] [--max-yaw-rate W] [--deadman SEC] [--rate HZ] [--duration SEC]\n"
        "  W/S forward/back, A/D lateral, Q/E yaw, Space zero, Esc quit.\n"
        "  Defaults: max-x=0.5 max-y=0.5 max-yaw-rate=0.5 deadman=0.4 rate=20 duration=30.\n"
        "  duration 0 runs until Esc. Exit, timeout, and Ctrl+C send zero velocity.\n"
        "  Forward, lateral, and yaw map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
        "  On PM01 lower_body_balance, linear_velocity[0] around 0.5 is where stepping starts.",
        file=sys.stderr,
    )


def send(session, x, y, yaw) -> bool:
    cmd = BodyVelocityCommand()
    cmd.x = x
    cmd.y = y
    cmd.yaw_rate = yaw
    return check_ok(session.base_velocity.send(cmd), "base_velocity.send")


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    max_x = max_y = max_yaw = 0.5
    deadman = 0.4
    rate = 20.0
    duration = 30.0
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--max-x" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "max-x")
            if parsed is None:
                return 2
            max_x = parsed
        elif arg == "--max-y" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "max-y")
            if parsed is None:
                return 2
            max_y = parsed
        elif arg == "--max-yaw-rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "max-yaw-rate")
            if parsed is None:
                return 2
            max_yaw = parsed
        elif arg == "--deadman" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "deadman")
            if parsed is None:
                return 2
            deadman = parsed
        elif arg == "--rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "rate")
            if parsed is None:
                return 2
            rate = parsed
        elif arg == "--duration" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "duration")
            if parsed is None:
                return 2
            duration = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if max_x < 0.0 or max_y < 0.0 or max_yaw < 0.0 or deadman <= 0.0 or rate < 5.0 or duration < 0.0:
        print("max values must be >= 0, deadman > 0, rate >= 5, duration >= 0", file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    if not wait_for_motion_sample(session):
        print_no_robot_data_hint()
        session.close()
        return 1
    terminal = TerminalRaw()
    if not terminal.active:
        print("stdin is not a terminal; keyboard control needs a local TTY", file=sys.stderr)
        session.close()
        return 2
    print(
        "Keyboard velocity control\n"
        "  W/S    forward / backward\n"
        "  A/D    strafe left / right\n"
        "  Q/E    turn left / right\n"
        "  Space  send zero velocity now\n"
        "  Esc or Ctrl+C  quit and send zero velocity\n"
        f"This run exits after {duration} s. Holding a key does not extend it. "
        "Pass --duration 0 to control until you quit.\n"
        f"The terminal repeats keys and has no key-up event. After {deadman} "
        "s without a repeat, the key counts as released and zero velocity is sent.\n"
        "kOk only means the command was sent, not that the robot is moving at that velocity.\n"
        f"motion={motion_or_none(session)} max_x={max_x} max_y={max_y} "
        f"max_yaw_rate={max_yaw} rate_hz={rate} duration_s={duration}\n"
        "The next line refreshes in place and keeps only the latest status."
    )
    catch_sigint()
    period = 1.0 / rate
    limited = duration > 0.0
    deadline = time.monotonic() + duration
    last_print = time.monotonic()
    hold_until = 0.0
    x = y = yaw = 0.0
    keys = "-"
    send_ok = True
    try:
        while send_ok and not stop_requested() and (not limited or time.monotonic() < deadline):
            saw_key = False
            next_x = next_y = next_yaw = 0.0
            next_keys = ""
            for ch in terminal.read_chars():
                if ch == "\x1b" or ch == "\x03":
                    request_stop()
                    break
                if ch == " ":
                    hold_until = 0.0
                    keys = "space"
                    saw_key = False
                    next_x = next_y = next_yaw = 0.0
                    next_keys = ""
                    continue
                bit = ""
                lower = ch.lower()
                if lower == "w":
                    next_x = max_x
                    bit = "W"
                elif lower == "s":
                    next_x = -max_x
                    bit = "S"
                elif lower == "a":
                    next_y = max_y
                    bit = "A"
                elif lower == "d":
                    next_y = -max_y
                    bit = "D"
                elif lower == "q":
                    next_yaw = max_yaw
                    bit = "Q"
                elif lower == "e":
                    next_yaw = -max_yaw
                    bit = "E"
                if bit:
                    saw_key = True
                    next_keys += bit
            if stop_requested():
                break
            now = time.monotonic()
            if saw_key:
                x, y, yaw = next_x, next_y, next_yaw
                keys = next_keys
                hold_until = now + deadman
            elif now > hold_until:
                x = y = yaw = 0.0
                keys = "-"
            send_ok = send(session, x, y, yaw)
            if now - last_print >= 0.1:
                print(
                    f"\rkeys={keys} x={x} y={y} yaw_rate={yaw} rate_hz={rate} "
                    f"motion={motion_or_none(session)}\033[K",
                    end="",
                    flush=True,
                )
                last_print = now
            time.sleep(period)
    finally:
        terminal.close()
    stopped = send(session, 0.0, 0.0, 0.0)
    print()
    if send_ok and not stop_requested() and limited:
        print(f"Reached duration={duration} s. Sending zero velocity and exiting.")
    print(f"stop_sent={'yes' if stopped else 'no'} motion={motion_or_none(session)}")
    pause_before_close()
    session.close()
    return 0 if send_ok and stopped else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
