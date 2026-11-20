#!/usr/bin/env python3
"""basic_joint_override: refresh a joint override and release it explicitly.

  python3 examples/basic_joint_override.py --domain-id 0 [--weight W] [--seconds N]
"""

import sys
import time

from engineai_sdkpy import JointCommand, JointOverrideCommand, ParallelParserType

from _cli import (
    catch_sigint,
    check_ok,
    create_session_or_exit,
    motion_or_none,
    parse_session_args,
    read_finite_float,
    read_strict_int,
    pause_before_close,
    stop_requested,
    wait_for_motion_sample,
    wants_help,
)


def main(argv) -> int:
    usage = (
        f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR] "
        "[--weight W] [--seconds N]"
    )
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print(usage, file=sys.stderr)
        return 2
    weight = 0.5
    seconds = 1
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--weight" and i + 1 < len(rest):
            parsed = read_finite_float(rest[i + 1], "weight")
            if parsed is None:
                return 2
            weight = parsed
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
        print(usage, file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    wait_for_motion_sample(session)
    print(f"current motion={motion_or_none(session)}")
    joint = session.joint_state.latest()
    if joint is None or not joint.position:
        print("No joint_state sample; abort", file=sys.stderr)
        session.close()
        return 1

    n = len(joint.position)
    command = JointCommand()
    command.position = list(joint.position)
    command.velocity = [0.0] * n
    command.feed_forward_torque = [0.0] * n
    command.torque = [0.0] * n
    command.stiffness = [200.0] * n
    command.damping = [5.0] * n
    command.parallel_parser_type = ParallelParserType.kClassic
    override = JointOverrideCommand()
    override.weight = weight
    override.joint_indices = list(range(n))
    override.command = command

    catch_sigint()
    deadline = time.monotonic() + seconds
    send_ok = True
    sent = False
    try:
        while send_ok and not stop_requested() and time.monotonic() < deadline:
            send_ok = check_ok(session.joint_override.send(override), "joint_override.send")
            if send_ok:
                sent = True
                time.sleep(0.1)
        print(f"joint_override refreshed for {seconds}s (weight={weight}, n={n})")
        released = True
        if sent:
            release = JointOverrideCommand()
            release.weight = 0.0
            released = check_ok(session.joint_override.send(release), "joint_override.release")
        if not send_ok or not released:
            return 1
        pause_before_close()
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
