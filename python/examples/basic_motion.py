#!/usr/bin/env python3
"""basic_motion: motion.current_motion / available_transitions / request_transition.

  python3 examples/basic_motion.py --domain-id 0
  python3 examples/basic_motion.py --domain-id 0 --motion joint_bridge [--timeout SEC]

Safety when the target is joint_bridge: this mode commands the joints
directly. The standing pose is not necessarily stable, so hoist the robot
before use. Both feet must be on the ground before entering walk.
"""

import sys
import time

from engineai_sdkpy import SessionError

from _cli import (
    create_session_or_exit,
    err_name,
    motion_is,
    motion_or_none,
    parse_session_args,
    read_strict_int,
    print_no_robot_data_hint,
    wait_for_motion_sample,
    wants_help,
)


def print_motion(session, label) -> None:
    print(f"==== {label} ====")
    print(f"current={motion_or_none(session)}")
    print("available_transitions:", end="")
    transitions = session.motion.available_transitions()
    if transitions is None:
        print(" (no sample)")
    else:
        print("".join(f" {name}" for name in transitions))


def main(argv) -> int:
    usage = (
        f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR] "
        "[--motion NAME] [--timeout SEC]"
    )
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print(usage, file=sys.stderr)
        return 2
    motion = ""
    timeout_sec = 8
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--motion" and i + 1 < len(rest):
            motion = rest[i + 1]
            i += 2
        elif arg == "--timeout" and i + 1 < len(rest):
            parsed = read_strict_int(rest[i + 1], "timeout")
            if parsed is None:
                return 2
            timeout_sec = parsed
            i += 2
        else:
            print(usage, file=sys.stderr)
            return 2
    session = create_session_or_exit(options)
    have_motion = wait_for_motion_sample(session)
    print_motion(session, "before")
    if not have_motion:
        print_no_robot_data_hint()
        session.close()
        return 1
    if not motion:
        session.close()
        return 0
    err = session.motion.request_transition(motion)
    print(f"motion.request_transition({motion}) -> {err_name(err)}")
    if err != SessionError.kOk:
        session.close()
        return 1
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if motion_is(session, motion):
            break
        time.sleep(0.2)
    print_motion(session, "after")
    ok = motion_is(session, motion)
    if not ok:
        print(f"timeout waiting for motion={motion}", file=sys.stderr)
    session.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
