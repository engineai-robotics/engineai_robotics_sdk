#!/usr/bin/env python3
"""safe_velocity_walk: send a short body-velocity command and always finish at zero."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import BodyVelocityCommand

from _cli import (
    catch_sigint,
    check_ok,
    create_session_or_exit,
    motion_or_none,
    parse_session_args,
    read_finite_float,
    pause_before_close,
    print_no_robot_data_hint,
    stop_requested,
    wait_for_motion_sample,
    wants_help,
)


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--x V] [--y V] [--yaw-rate W] [--duration SEC] [--rate HZ]\n"
        "  Defaults: x=0 y=0 yaw-rate=0 duration=2 rate=50.\n"
        "  --rate must be >= 20. --duration 0 runs until Ctrl+C.\n"
        "  Requires a motion sample first. Exit always sends zero velocity.\n"
        "  The robot releases an external velocity if it is not refreshed within 500 ms.\n"
        "  --x, --y, and --yaw-rate map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
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
    x = y = yaw = 0.0
    duration = 2.0
    rate = 50.0
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--x" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "x")
            if parsed is None:
                return 2
            x = parsed
        elif arg == "--y" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "y")
            if parsed is None:
                return 2
            y = parsed
        elif arg == "--yaw-rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "yaw-rate")
            if parsed is None:
                return 2
            yaw = parsed
        elif arg == "--duration" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "duration")
            if parsed is None:
                return 2
            duration = parsed
        elif arg == "--rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "rate")
            if parsed is None:
                return 2
            rate = parsed
        elif arg == "--stop-on-exit":
            pass
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if duration < 0.0 or rate < 20.0:
        print("duration must be >= 0 and rate must be >= 20", file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    if not wait_for_motion_sample(session):
        print_no_robot_data_hint()
        session.close()
        return 1
    print(
        f"motion={motion_or_none(session)} x={x} y={y} yaw_rate={yaw} "
        f"rate_hz={rate} duration_s={duration}"
    )
    print("note: send kOk means the command was sent, not that the robot applied the velocity")
    catch_sigint()
    period = 1.0 / rate
    deadline = None if duration == 0.0 else time.monotonic() + duration
    send_ok = True
    while send_ok and not stop_requested() and (deadline is None or time.monotonic() < deadline):
        send_ok = send(session, x, y, yaw)
        time.sleep(period)
    stopped = send(session, 0.0, 0.0, 0.0)
    print(f"stop_sent={'yes' if stopped else 'no'} motion={motion_or_none(session)}")
    pause_before_close()
    session.close()
    return 0 if send_ok and stopped else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
