#!/usr/bin/env python3
"""basic_body_velocity: base_velocity.send.

  python3 examples/basic_body_velocity.py --domain-id 0 [--vx V] [--vy V] [--yaw W] [--seconds N]
"""

import sys
import time

from engineai_sdkpy import BodyVelocityCommand

from _cli import (
    catch_sigint,
    check_ok,
    create_session_or_exit,
    parse_session_args,
    read_finite_float,
    read_strict_int,
    pause_before_close,
    stop_requested,
    wants_help,
)


def main(argv) -> int:
    usage = (
        f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR] "
        "[--vx V] [--vy V] [--yaw W] [--seconds N]\n"
        "  --vx, --vy, and --yaw map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
        "  On PM01 lower_body_balance, linear_velocity[0] around 0.5 is where stepping starts."
    )
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print(usage, file=sys.stderr)
        return 2
    vx = vy = yaw = 0.0
    seconds = 1
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--vx" and i + 1 < len(rest):
            parsed = read_finite_float(rest[i + 1], "vx")
            if parsed is None:
                return 2
            vx = parsed
            i += 2
        elif arg == "--vy" and i + 1 < len(rest):
            parsed = read_finite_float(rest[i + 1], "vy")
            if parsed is None:
                return 2
            vy = parsed
            i += 2
        elif arg == "--yaw" and i + 1 < len(rest):
            parsed = read_finite_float(rest[i + 1], "yaw")
            if parsed is None:
                return 2
            yaw = parsed
            i += 2
        elif arg == "--seconds" and i + 1 < len(rest):
            parsed = read_strict_int(rest[i + 1], "seconds")
            if parsed is None:
                return 2
            seconds = parsed
            i += 2
        else:
            print(usage, file=sys.stderr)
            return 2
    if seconds <= 0:
        print("seconds must be > 0", file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    cmd = BodyVelocityCommand()
    cmd.x = vx
    cmd.y = vy
    cmd.yaw_rate = yaw
    catch_sigint()
    deadline = time.monotonic() + seconds
    send_ok = True
    try:
        while send_ok and not stop_requested() and time.monotonic() < deadline:
            send_ok = check_ok(session.base_velocity.send(cmd), "base_velocity.send")
            if send_ok:
                time.sleep(0.02)
        stop = BodyVelocityCommand()
        stopped = check_ok(session.base_velocity.send(stop), "base_velocity.send(0,0,0)")
        if not send_ok or not stopped:
            return 1
        pause_before_close()
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
