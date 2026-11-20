#!/usr/bin/env python3
"""t800pro_hand_gesture_control: open, close, or relax the t800pro hands in joint_bridge.

Safety: joint_bridge commands the joints directly. The standing pose in this
mode is not necessarily stable, so hoist the robot before use. Both feet
must be on the ground before entering walk.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import JointCommand, ParallelParserType

from _cli import (
    catch_sigint,
    check_ok,
    create_session_or_exit,
    motion_is,
    motion_or_none,
    read_finite_float,
    parse_session_args,
    require_current_motion,
    stop_requested,
    wait_joint_state,
    wants_help,
)

NUM_JOINTS = 43
LEFT_HAND = (20, 21, 22, 23, 24, 25, 26)
RIGHT_HAND = (34, 35, 36, 37, 38, 39, 40)
LEFT_EXTEND = (0, 0, 0, 0, 0, 0, 0)
LEFT_FIST = (-0.85, 0.62, 0.48, -0.80, -0.58, -0.80, -0.58)
RIGHT_EXTEND = (0, 0, 0, 0, 0, 0, 0)
RIGHT_FIST = (0.85, 0.62, 0.48, 0.80, 0.58, 0.80, 0.58)
STIFFNESS = (
    1080, 480, 880, 1000, 800, 100, 1080, 480, 880, 1000, 800, 100, 200, 120, 120, 120, 120, 120, 10, 10, 5, 5,
    5, 5, 5, 5, 5, 120, 120, 120, 120, 120, 10, 10, 5, 5, 5, 5, 5, 5, 5, 100, 100,
)
DAMPING = (
    25, 25, 25, 25, 2, 2, 25, 25, 25, 25, 2, 2, 1, 1.8, 1.5, 1.5, 1.8, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2, 0.2,
    0.2, 0.2, 0.2, 1.8, 1.5, 1.5, 1.8, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1,
)


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR] --enable-low-level\n"
        "       [--hand left|right|both] [--gesture open|close|relax|both|pinch|custom-yaml]\n"
        "       [--pose extend|loose_fist|both] [--exec-time SEC] [--hold-time SEC] [--duration SEC] [--rate HZ]\n"
        "  T800 Pro joint_bridge only. Expects 43 joint_state values.\n"
        "  Each pose interpolates for --exec-time, then holds for --hold-time.\n"
        "  --pose both and --gesture both run loose fist, then extend. --duration sets --exec-time.\n"
        "  pinch and custom-yaml are refused: this tree has no verified angles for them.",
        file=sys.stderr,
    )


def hand_joints(hand):
    if hand == "left":
        return list(LEFT_HAND)
    if hand == "right":
        return list(RIGHT_HAND)
    return list(LEFT_HAND) + list(RIGHT_HAND)


def goal_hands(gesture, hand):
    closed = gesture == "close"
    left = LEFT_FIST if closed else LEFT_EXTEND
    right = RIGHT_FIST if closed else RIGHT_EXTEND
    if hand == "left":
        return list(left)
    if hand == "right":
        return list(right)
    return list(left) + list(right)


def make_command(positions):
    cmd = JointCommand()
    cmd.position = list(positions)
    cmd.velocity = [0.0] * NUM_JOINTS
    cmd.feed_forward_torque = [0.0] * NUM_JOINTS
    cmd.torque = [0.0] * NUM_JOINTS
    cmd.stiffness = list(STIFFNESS)
    cmd.damping = list(DAMPING)
    cmd.parallel_parser_type = ParallelParserType.kClassic
    return cmd


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    enable = False
    hand = "both"
    gesture = "open"
    gesture_set = False
    pose_set = False
    exec_time = 3.0
    hold_time = 2.5
    rate = 500.0
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--enable-low-level":
            enable = True
        elif arg == "--hand" and index + 1 < len(rest):
            index += 1
            hand = rest[index]
        elif arg == "--gesture" and index + 1 < len(rest):
            index += 1
            gesture = rest[index]
            gesture_set = True
        elif arg == "--pose" and index + 1 < len(rest):
            index += 1
            pose = rest[index]
            pose_set = True
            if pose == "extend":
                gesture = "open"
            elif pose == "loose_fist":
                gesture = "close"
            elif pose == "both":
                gesture = "both"
            else:
                print("--pose must be extend, loose_fist, or both", file=sys.stderr)
                return 2
        elif arg in ("--exec-time", "--duration") and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "exec-time")
            if parsed is None:
                return 2
            exec_time = parsed
        elif arg == "--hold-time" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "hold-time")
            if parsed is None:
                return 2
            hold_time = parsed
        elif arg == "--rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "rate")
            if parsed is None:
                return 2
            rate = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if not enable:
        print("refusing hand command without --enable-low-level", file=sys.stderr)
        return 2
    if gesture_set and pose_set:
        print("pass only one of --gesture or --pose", file=sys.stderr)
        return 2
    if hand not in ("left", "right", "both"):
        print("--hand must be left, right, or both", file=sys.stderr)
        return 2
    if gesture in ("pinch", "custom-yaml"):
        print(f"refusing gesture {gesture}: this tree has no verified angles for it", file=sys.stderr)
        return 2
    if gesture not in ("open", "close", "relax", "both"):
        print("--gesture must be open, close, relax, both, pinch, or custom-yaml", file=sys.stderr)
        return 2
    if exec_time <= 0.0 or hold_time < 0.0 or rate <= 0.0:
        print("exec-time and rate must be > 0, hold-time must be >= 0", file=sys.stderr)
        return 2
    if gesture == "relax":
        print("relax uses the existing extend pose")
    print("model_check=joint_count_43_only example=t800pro_hand_gesture_control")

    session = create_session_or_exit(options)
    if not require_current_motion(session, "joint_bridge"):
        session.close()
        return 1
    joint = wait_joint_state(session)
    if joint is None or len(joint.position) != NUM_JOINTS:
        got = 0 if joint is None else len(joint.position)
        print(f"need joint_state with {NUM_JOINTS} joints, got {got}", file=sys.stderr)
        session.close()
        return 1
    joints = hand_joints(hand)
    if gesture == "both":
        poses = ["close", "open"]
    elif gesture == "relax":
        poses = ["open"]
    else:
        poses = [gesture]
    pieces = " ".join(f"{item}={joint.position[item]}" for item in joints)
    print(f"hand={hand} gesture={gesture} indices: {pieces}")
    catch_sigint()
    period = 1.0 / rate
    hold = list(joint.position)
    ok = True
    for pose_name in poses:
        if not ok or stop_requested():
            break
        goal = goal_hands(pose_name, hand)
        start_hands = [hold[item] for item in joints]
        print(f"pose={pose_name} exec_time_s={exec_time} hold_time_s={hold_time}")
        started = time.monotonic()
        while ok and not stop_requested():
            if not motion_is(session, "joint_bridge"):
                print(f"left joint_bridge, current={motion_or_none(session)}", file=sys.stderr)
                ok = False
                break
            elapsed = time.monotonic() - started
            if elapsed >= exec_time:
                break
            alpha = elapsed / exec_time
            command_position = list(hold)
            for i, item in enumerate(joints):
                command_position[item] = start_hands[i] + (goal[i] - start_hands[i]) * alpha
            if not check_ok(session.joint_command.send(make_command(command_position)), "joint_command.send"):
                ok = False
                break
            time.sleep(period)
        if ok and not stop_requested():
            for i, item in enumerate(joints):
                hold[item] = goal[i]
            if not check_ok(session.joint_command.send(make_command(hold)), "joint_command.send"):
                ok = False
        hold_until = time.monotonic() + hold_time
        while ok and not stop_requested() and time.monotonic() < hold_until:
            if not motion_is(session, "joint_bridge"):
                print(f"left joint_bridge, current={motion_or_none(session)}", file=sys.stderr)
                ok = False
                break
            if not check_ok(session.joint_command.send(make_command(hold)), "joint_command.send"):
                ok = False
                break
            time.sleep(period)
    feedback = session.joint_command.latest_feedback()
    if feedback is not None:
        parts = []
        for item in joints:
            if item < len(feedback.position):
                parts.append(f"{item}={feedback.position[item]}")
        print("feedback " + " ".join(parts))
    else:
        print("feedback=(no sample)")
    session.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
