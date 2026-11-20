#!/usr/bin/env python3
"""walk_and_motion_plan: walk with base_velocity while a joint-motion plan plays."""

import sys
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
    require_current_motion,
    request_stop,
    stop_requested,
    wants_help,
)
from _plan import RunCode, run_motion_plan
from _yaml import DEFAULT_ACTIONS_DIR, load_motion_plan, print_plan_preview, resolve_action_yaml


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       --model pm01|t800|t800pro [--interaction-action wave|handshake|reset]\n"
        "       [--x V] [--y V] [--yaw-rate W] [--actions DIR] [--timeout SEC] [--dry-run]\n"
        "  Default action is wave and --x is 0.5 (linear_velocity[0]).\n"
        "  --x, --y, and --yaw-rate map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
        "  Walking uses base_velocity. The action uses the existing motion-plan YAML.\n"
        "  --control-mode joint-override is refused and sends nothing.",
        file=sys.stderr,
    )


def send_velocity(session, x, y, yaw) -> bool:
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
    model = ""
    control_mode = "motion-plan"
    action = "wave"
    actions_dir = str(DEFAULT_ACTIONS_DIR)
    timeout_s = 0.0
    dry_run = False
    walk = {"x": 0.5, "y": 0.0, "yaw": 0.0, "stop": False, "send_ok": True}
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--model" and index + 1 < len(rest):
            index += 1
            model = rest[index]
        elif arg == "--control-mode" and index + 1 < len(rest):
            index += 1
            control_mode = rest[index]
        elif arg == "--interaction-action" and index + 1 < len(rest):
            index += 1
            action = rest[index]
        elif arg == "--actions" and index + 1 < len(rest):
            index += 1
            actions_dir = rest[index]
        elif arg == "--timeout" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "timeout")
            if parsed is None:
                return 2
            timeout_s = parsed
        elif arg == "--x" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "x")
            if parsed is None:
                return 2
            walk["x"] = parsed
        elif arg == "--y" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "y")
            if parsed is None:
                return 2
            walk["y"] = parsed
        elif arg == "--yaw-rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "yaw-rate")
            if parsed is None:
                return 2
            walk["yaw"] = parsed
        elif arg == "--dry-run":
            dry_run = True
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if control_mode == "joint-override":
        print(
            "refusing joint-override: this example walks with base_velocity and gestures with a motion plan.",
            file=sys.stderr,
        )
        return 2
    if control_mode != "motion-plan":
        print("control-mode must be motion-plan or joint-override", file=sys.stderr)
        return 2
    path, error = resolve_action_yaml(actions_dir, model, action)
    if path is None:
        print(error, file=sys.stderr)
        return 2
    plan, error = load_motion_plan(path)
    if plan is None:
        print(error, file=sys.stderr)
        return 1
    print(
        f"model={model} walking=base_velocity x={walk['x']} y={walk['y']} yaw_rate={walk['yaw']} "
        f"interaction=motion_plan action={action}"
    )
    print(
        "Keep sending velocity while the motion plan runs. Send zero velocity on completion, "
        "failure, or Ctrl+C. kOk only means the command was sent."
    )
    print_plan_preview(path, plan, 1.0)
    if dry_run:
        run_motion_plan(None, plan, {"speed_scale": 1.0, "timeout_s": 0.0, "cancel_after_ms": -1, "dry_run": True})
        return 0

    def poll(session, _user):
        if stop_requested():
            walk["stop"] = True
        x = 0.0 if walk["stop"] else walk["x"]
        y = 0.0 if walk["stop"] else walk["y"]
        yaw = 0.0 if walk["stop"] else walk["yaw"]
        if not send_velocity(session, x, y, yaw):
            walk["send_ok"] = False
            return True
        return walk["stop"]

    session = create_session_or_exit(options)
    if not require_current_motion(session, "lower_body_balance"):
        session.close()
        return 1
    print(f"motion={motion_or_none(session)}")
    catch_sigint()
    if not send_velocity(session, walk["x"], walk["y"], walk["yaw"]):
        session.close()
        return 1
    code = run_motion_plan(
        session,
        plan,
        {"speed_scale": 1.0, "timeout_s": timeout_s, "cancel_after_ms": -1, "dry_run": False},
        poll,
        None,
    )
    walk["stop"] = True
    stopped = send_velocity(session, 0.0, 0.0, 0.0)
    print(
        f"interaction_result={code} stop_sent={'yes' if stopped else 'no'} motion={motion_or_none(session)}"
    )
    pause_before_close()
    session.close()
    plan_ok = code in (RunCode.COMPLETED, RunCode.CANCELLED)
    return 0 if walk["send_ok"] and stopped and plan_ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
