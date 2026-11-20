#!/usr/bin/env python3
"""basic_read_state: joint_state / motor_state / imu / gamepad.

latest() keeps the last sample after the robot stops publishing, so each line
also prints the sample age from last_receive_time().

  python3 examples/basic_read_state.py --domain-id 0 [--interface eth0]
"""

import sys

from _cli import (
    age_text,
    create_session_or_exit,
    parse_session_args,
    print_no_robot_data_hint,
    wait_joint_state,
    wants_help,
)


def main(argv) -> int:
    usage = f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR]"
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None or rest:
        print(usage, file=sys.stderr)
        return 2
    session = create_session_or_exit(options)
    have_joint = wait_joint_state(session) is not None

    joint = session.joint_state.latest()
    if joint is not None:
        line = f"joint_state n={len(joint.position)}"
        if joint.position:
            line += f" q0={joint.position[0]}"
        print(f"{line} age={age_text(session.joint_state.last_receive_time())}")
    else:
        print("joint_state (no sample)")

    motor = session.motor_state.latest()
    if motor is not None:
        print(f"motor_state n={len(motor.position)} age={age_text(session.motor_state.last_receive_time())}")
    else:
        print("motor_state (no sample)")

    imu = session.imu.latest()
    if imu is not None:
        print(f"imu rpy=({imu.rpy.x}, {imu.rpy.y}, {imu.rpy.z}) age={age_text(session.imu.last_receive_time())}")
    else:
        print("imu (no sample)")

    pad = session.gamepad.latest()
    if pad is not None:
        print(f"gamepad connected={int(pad.hardware_connected)} age={age_text(session.gamepad.last_receive_time())}")
    else:
        print("gamepad (no sample)")

    if (
        not have_joint
        and session.motor_state.latest() is None
        and session.imu.latest() is None
        and session.gamepad.latest() is None
    ):
        print_no_robot_data_hint()
        session.close()
        return 1
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
