#!/usr/bin/env python3
"""joint_bridge_trajectory: interpolate joints from YAML in joint_bridge.

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
from _yaml import flatten_numbers, load_file


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR] "
        "--enable-low-level --config PATH.yaml\n"
        "       [--max-delta RAD]\n"
        "  Requires motion joint_bridge. Without --max-delta, large targets are sent.\n"
        "  With --max-delta, any joint whose total move exceeds the limit cancels the whole command.",
        file=sys.stderr,
    )


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    config_path = ""
    enable = False
    have_max_delta = False
    max_delta = 0.0
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--enable-low-level":
            enable = True
        elif arg == "--config" and index + 1 < len(rest):
            index += 1
            config_path = rest[index]
        elif arg == "--max-delta" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "max-delta")
            if parsed is None:
                return 2
            max_delta = parsed
            have_max_delta = True
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if not enable:
        print("refusing low-level joint command without --enable-low-level", file=sys.stderr)
        return 2
    if have_max_delta and max_delta < 0.0:
        print("max-delta must be >= 0", file=sys.stderr)
        return 2
    if not config_path:
        print("Missing required --config PATH.yaml", file=sys.stderr)
        print_usage(argv[0])
        return 2
    if not Path(config_path).is_file():
        print(f"config not found: {config_path}", file=sys.stderr)
        return 1
    try:
        root = load_file(config_path)
        target_positions = flatten_numbers(root.get("target_position"))
        kp = flatten_numbers(root.get("kp"))
        kd = flatten_numbers(root.get("kd"))
        num_steps = flatten_numbers(root.get("num_steps"), as_int=True)
    except (OSError, ValueError) as ex:
        print(f"failed to parse config: {ex}", file=sys.stderr)
        return 1

    session = create_session_or_exit(options)
    if not require_current_motion(session, "joint_bridge"):
        session.close()
        return 1
    joint = wait_joint_state(session)
    if joint is None:
        print("Timed out waiting for joint_state", file=sys.stderr)
        session.close()
        return 1
    count = len(joint.position)
    if not (len(target_positions) == len(kp) == len(kd) == len(num_steps) == count):
        print(
            f"config/joint_state length mismatch: joints={count} target={len(target_positions)} "
            f"kp={len(kp)} kd={len(kd)} num_steps={len(num_steps)}",
            file=sys.stderr,
        )
        session.close()
        return 1
    if have_max_delta:
        worst = 0.0
        worst_index = 0
        for i in range(count):
            delta = abs(target_positions[i] - joint.position[i])
            if delta > worst:
                worst = delta
                worst_index = i
        print(f"joints={count} max_delta={worst} at={worst_index} limit={max_delta}")
        if worst > max_delta:
            print("refusing command: joint delta is above --max-delta", file=sys.stderr)
            session.close()
            return 1

    trajectories = []
    for i in range(count):
        steps = num_steps[i]
        if steps < 2:
            print(f"num_steps[{i}] must be >= 2", file=sys.stderr)
            session.close()
            return 1
        step_size = (target_positions[i] - joint.position[i]) / (steps - 1)
        trajectories.append([joint.position[i] + step_size * s for s in range(steps)])

    cmd = JointCommand()
    cmd.position = [0.0] * count
    cmd.velocity = [0.0] * count
    cmd.feed_forward_torque = [0.0] * count
    cmd.torque = [0.0] * count
    cmd.stiffness = kp
    cmd.damping = kd
    cmd.parallel_parser_type = ParallelParserType.kClassic
    current_steps = [0] * count
    reached = [False] * count
    catch_sigint()
    all_reached = False
    while not stop_requested() and not all_reached:
        if not motion_is(session, "joint_bridge"):
            print(f"left joint_bridge, current={motion_or_none(session)}", file=sys.stderr)
            session.close()
            return 1
        all_reached = True
        for i in range(count):
            if not reached[i]:
                if current_steps[i] < num_steps[i]:
                    cmd.position[i] = trajectories[i][current_steps[i]]
                    current_steps[i] += 1
                    all_reached = False
                else:
                    cmd.position[i] = target_positions[i]
                    reached[i] = True
            else:
                cmd.position[i] = target_positions[i]
        if all_reached:
            break
        if not check_ok(session.joint_command.send(cmd), "joint_command.send"):
            session.close()
            return 1
        time.sleep(0.002)
    print("all joints reached target (or interrupted)")
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
