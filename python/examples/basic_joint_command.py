#!/usr/bin/env python3
"""basic_joint_command: joint_command.send + latest_feedback.

Safety: joint_bridge commands the joints directly. The standing pose in this
mode is not necessarily stable, so hoist the robot before use. Both feet
must be on the ground before entering walk.

Requires current motion == joint_bridge (does not call request_transition).

  python3 examples/basic_joint_command.py --domain-id 0 [--hold-ms N]
"""

import sys
import time

from engineai_sdkpy import JointCommand, ParallelParserType

from _cli import (
    STATE_MAX_AGE,
    age_text,
    check_ok,
    create_session_or_exit,
    is_fresh,
    parse_session_args,
    read_strict_int,
    require_current_motion,
    wait_joint_state,
    wants_help,
)


def main(argv) -> int:
    usage = f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR] [--hold-ms N]"
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print(usage, file=sys.stderr)
        return 2
    hold_ms = 500
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--hold-ms" and i + 1 < len(rest):
            parsed = read_strict_int(rest[i + 1], "hold-ms")
            if parsed is None:
                return 2
            hold_ms = parsed
            i += 2
        else:
            print(usage, file=sys.stderr)
            return 2

    session = create_session_or_exit(options)
    if not require_current_motion(session, "joint_bridge"):
        session.close()
        return 1
    joint = wait_joint_state(session)
    if joint is None:
        print("No joint_state sample; abort", file=sys.stderr)
        session.close()
        return 1

    n = len(joint.position)
    cmd = JointCommand()
    cmd.position = list(joint.position)
    cmd.velocity = [0.0] * n
    cmd.feed_forward_torque = [0.0] * n
    cmd.torque = [0.0] * n
    cmd.stiffness = [200.0] * n
    cmd.damping = [5.0] * n
    cmd.parallel_parser_type = ParallelParserType.kClassic

    deadline = time.monotonic() + hold_ms / 1000.0
    while time.monotonic() < deadline:
        # Stop when the robot stops publishing; latest() alone would keep the last sample.
        received = session.joint_state.last_receive_time()
        if not is_fresh(received, STATE_MAX_AGE):
            print(f"joint_state stale ({age_text(received)}); stop sending", file=sys.stderr)
            session.close()
            return 1
        err = session.joint_command.send(cmd)
        if not check_ok(err, "joint_command.send"):
            session.close()
            return 1
        time.sleep(0.005)

    feedback = session.joint_command.latest_feedback()
    if feedback is not None:
        print(f"latest_feedback n={len(feedback.position)} age={age_text(session.joint_command.last_feedback_time())}")
    else:
        print("latest_feedback (no sample)")
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
