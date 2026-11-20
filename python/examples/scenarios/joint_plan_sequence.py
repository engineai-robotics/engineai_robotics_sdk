#!/usr/bin/env python3
"""joint_plan_sequence: run the existing wave, handshake, and reset YAML files."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _cli import (
    catch_sigint,
    create_session_or_exit,
    motion_or_none,
    read_finite_float,
    parse_session_args,
    require_current_motion,
    wait_joint_state,
    wants_help,
)
from _plan import run_motion_plan
from _yaml import DEFAULT_ACTIONS_DIR, load_motion_plan, print_plan_preview, resolve_action_yaml


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       (--model pm01|t800|t800pro (--action wave|handshake|reset | --sequence a,b)\n"
        "        | --config PATH.yaml)\n"
        "       [--actions DIR] [--speed-scale S] [--timeout SEC] [--cancel-after SEC] [--dry-run]\n"
        "  execute requires current motion lower_body_balance.\n"
        "  --config runs one motion-plan YAML and does not need --model.\n"
        "  Default actions dir is the repository cpp/examples/config. --dry-run does not send.\n"
        "  Planner status/progress is the completion signal. Local kOk is only a send result.",
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
    model = ""
    config_path = ""
    action_set = False
    actions_dir = str(DEFAULT_ACTIONS_DIR)
    actions = []
    speed_scale = 1.0
    timeout_s = 0.0
    cancel_after_ms = -1
    dry_run = False
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--model" and index + 1 < len(rest):
            index += 1
            model = rest[index]
        elif arg == "--config" and index + 1 < len(rest):
            index += 1
            config_path = rest[index]
        elif arg == "--action" and index + 1 < len(rest):
            index += 1
            action_set = True
            actions.append(rest[index])
        elif arg == "--sequence" and index + 1 < len(rest):
            index += 1
            action_set = True
            actions.extend(part for part in rest[index].split(",") if part)
        elif arg == "--actions" and index + 1 < len(rest):
            index += 1
            actions_dir = rest[index]
        elif arg == "--speed-scale" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "speed-scale")
            if parsed is None:
                return 2
            speed_scale = parsed
        elif arg == "--timeout" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "timeout")
            if parsed is None:
                return 2
            timeout_s = parsed
        elif arg == "--cancel-after" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "cancel-after")
            if parsed is None:
                return 2
            cancel_after_ms = int(parsed * 1000.0)
        elif arg == "--dry-run":
            dry_run = True
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if config_path and action_set:
        print("pass either --config or --action/--sequence", file=sys.stderr)
        return 2
    if not config_path and not actions:
        actions.append("wave")
    if speed_scale <= 0.0 or timeout_s < 0.0:
        print("speed-scale must be > 0 and timeout must be >= 0", file=sys.stderr)
        return 2

    plans = []
    if config_path:
        plan, error = load_motion_plan(config_path)
        if plan is None:
            print(error, file=sys.stderr)
            return 1
        print(f"config={config_path}")
        print_plan_preview(config_path, plan, speed_scale)
        plans.append((config_path, plan))
    for action in actions:
        path, error = resolve_action_yaml(actions_dir, model, action)
        if path is None:
            print(error, file=sys.stderr)
            return 2
        plan, error = load_motion_plan(path)
        if plan is None:
            print(error, file=sys.stderr)
            return 1
        print(f"model={model} action={action}")
        print_plan_preview(path, plan, speed_scale)
        plans.append((path, plan))
    request = {
        "speed_scale": speed_scale,
        "timeout_s": timeout_s,
        "cancel_after_ms": cancel_after_ms,
        "dry_run": dry_run,
    }
    if dry_run:
        for _path, plan in plans:
            run_motion_plan(None, plan, request)
        return 0

    session = create_session_or_exit(options)
    if not require_current_motion(session, "lower_body_balance"):
        session.close()
        return 1
    joint = wait_joint_state(session)
    if joint is None:
        print("no joint_state sample", file=sys.stderr)
        session.close()
        return 1
    print(f"motion={motion_or_none(session)} joint_n={len(joint.position)}")
    catch_sigint()
    request["dry_run"] = False
    for path, plan in plans:
        code = run_motion_plan(session, plan, request)
        print(f"result={code} config={path}")
        if code != "completed":
            session.close()
            return 1
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
