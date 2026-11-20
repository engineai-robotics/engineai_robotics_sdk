#!/usr/bin/env python3
"""upper_body_override: quintic upper-body override in lower_body_balance."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import JointOverrideCommand, ParallelParserType

from _cli import (
    catch_sigint,
    check_ok,
    create_session_or_exit,
    motion_is,
    motion_or_none,
    parse_session_args,
    read_strict_int,
    pause_before_close,
    require_current_motion,
    stop_requested,
    wants_help,
)
from _yaml import flatten_numbers, load_file


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR] "
        "--config PATH.yaml [--seconds N]\n"
        "  --seconds 0 means run until Ctrl+C",
        file=sys.stderr,
    )


def quintic_sample(q0, qf, duration, t):
    t = min(max(t, 0.0), duration)
    span = duration
    span2 = span * span
    span3 = span2 * span
    span4 = span3 * span
    span5 = span4 * span
    out = []
    for start, end in zip(q0, qf):
        delta = end - start
        a3 = (20.0 * delta) / (2.0 * span3)
        a4 = (30.0 * (start - end)) / (2.0 * span4)
        a5 = (12.0 * delta) / (2.0 * span5)
        out.append(start + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t)
    return out


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    config_path = ""
    seconds = 0
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--config" and index + 1 < len(rest):
            index += 1
            config_path = rest[index]
        elif arg == "--seconds" and index + 1 < len(rest):
            index += 1
            parsed = read_strict_int(rest[index], "seconds")
            if parsed is None:
                return 2
            seconds = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if not config_path:
        print("Missing required --config PATH.yaml", file=sys.stderr)
        print_usage(argv[0])
        return 2
    if not Path(config_path).is_file():
        print(f"config not found: {config_path}", file=sys.stderr)
        return 1
    if seconds < 0:
        print("seconds must be >= 0", file=sys.stderr)
        return 2
    try:
        root = load_file(config_path)
        freq = float(root["timer_freq"])
        duration = float(root["duration"])
        joint_indices = flatten_numbers(root.get("joint_indices"), as_int=True)
        start_pos = flatten_numbers(root.get("start_pos"))
        end_pos = flatten_numbers(root.get("end_pos"))
        velocity = flatten_numbers(root.get("velocity"))
        feed_forward_torque = flatten_numbers(root.get("feed_forward_torque"))
        torque = flatten_numbers(root.get("torque"))
        stiffness = flatten_numbers(root.get("stiffness"))
        damping = flatten_numbers(root.get("damping"))
    except (OSError, ValueError, KeyError, TypeError) as ex:
        print(f"failed to parse config: {ex}", file=sys.stderr)
        return 1
    count = len(joint_indices)
    lengths = [
        len(start_pos),
        len(end_pos),
        len(velocity),
        len(feed_forward_torque),
        len(torque),
        len(stiffness),
        len(damping),
    ]
    if count == 0 or any(length != count for length in lengths):
        print(f"config vector length mismatch vs joint_indices n={count}", file=sys.stderr)
        return 1
    if freq <= 0.0 or duration <= 0.0:
        print("timer_freq and duration must be > 0", file=sys.stderr)
        return 1
    steps = max(2, int(duration * freq))
    trajectory = [
        quintic_sample(start_pos, end_pos, duration, duration * step / (steps - 1)) for step in range(steps)
    ]

    session = create_session_or_exit(options)
    if not require_current_motion(session, "lower_body_balance"):
        session.close()
        return 1
    override = JointOverrideCommand()
    override.weight = 1.0
    override.joint_indices = joint_indices
    override.command.velocity = velocity
    override.command.feed_forward_torque = feed_forward_torque
    override.command.torque = torque
    override.command.stiffness = stiffness
    override.command.damping = damping
    override.command.parallel_parser_type = ParallelParserType.kClassic
    catch_sigint()
    period = 1.0 / freq
    deadline = None if seconds == 0 else time.monotonic() + seconds
    idx = 0
    direction = 1
    send_ok = True
    sent = False
    left_mode = False
    try:
        while send_ok and not left_mode and not stop_requested() and (deadline is None or time.monotonic() < deadline):
            if not motion_is(session, "lower_body_balance"):
                print(f"left lower_body_balance, current={motion_or_none(session)}", file=sys.stderr)
                left_mode = True
                break
            override.command.position = trajectory[idx]
            if not check_ok(session.joint_override.send(override), "joint_override.send"):
                send_ok = False
                break
            sent = True
            idx += direction
            if idx >= steps - 1:
                idx = steps - 1
                direction = -1
            elif idx <= 0:
                idx = 0
                direction = 1
            time.sleep(period)
        released = True
        if sent:
            release = JointOverrideCommand()
            release.weight = 0.0
            released = check_ok(session.joint_override.send(release), "joint_override.release")
        if send_ok and released and not left_mode:
            pause_before_close()
            return 0
        return 1
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
