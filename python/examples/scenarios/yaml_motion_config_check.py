#!/usr/bin/env python3
"""yaml_motion_config_check: local YAML check. This program does not send commands."""

import json
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _cli import create_session_or_exit, parse_session_args, wait_joint_state, wants_help
from _cli import ensure_finite_number, parse_yaml_bool
from _yaml import flatten_numbers, load_file


def print_usage(argv0):
    print(
        f"Usage: {argv0} --file PATH --check joint-plan|joint-command|override\n"
        "       [--model pm01|t800|t800pro] [--domain-id N] [--interface IF] [--strict] [--json]\n"
        "  Checks the file only. A failed check does not send a command.\n"
        "  --model joint counts: pm01=24, t800=25, t800pro=43, from joint_test.yaml.",
        file=sys.stderr,
    )


def model_joints(model):
    return {"pm01": 24, "t800": 25, "t800pro": 43}.get(model, -1)


def add(issues, field, reason):
    issues.append((field, reason))


def check_same_size(issues, field, actual, expected):
    if actual != expected:
        add(issues, field, f"length {actual} != {expected}")


def check_indices(issues, indices, joint_count):
    for index in indices:
        if index < 0:
            add(issues, "joint_indices", f"negative index {index}")
        elif joint_count >= 0 and index >= joint_count:
            add(issues, "joint_indices", f"index {index} >= joint count {joint_count}")


def check_joint_plan(root, strict, joint_count, issues):
    plan = root.get("motion_plan") if isinstance(root, dict) else None
    if not isinstance(plan, list) or len(plan) < 2:
        add(issues, "motion_plan", "need a sequence with joint_indices and at least one step")
        return
    indices = flatten_numbers(plan[0].get("joint_indices") if isinstance(plan[0], dict) else None, as_int=True)
    if not indices:
        add(issues, "joint_indices", "missing or empty")
    check_indices(issues, indices, joint_count)
    for i in range(1, len(plan)):
        item = plan[i] if isinstance(plan[i], dict) else {}
        name = str(item.get("motion") or f"step_{i}")
        request_type = str(item.get("request_type") or "").upper()
        if request_type == "RESET":
            continue
        targets = flatten_numbers(item.get("target_positions"))
        stiffness = flatten_numbers(item.get("stiffness"))
        damping = flatten_numbers(item.get("damping"))
        check_same_size(issues, f"{name}.target_positions", len(targets), len(indices))
        duration = -1.0
        duration_ok = False
        if item.get("duration") is not None:
            try:
                duration = ensure_finite_number(item["duration"])
                duration_ok = True
            except ValueError:
                duration_ok = False
        if not duration_ok or not duration > 0.0:
            add(issues, f"{name}.duration", "execution time must be a finite number > 0")
        if item.get("use_gravity_compensation") is not None:
            try:
                parse_yaml_bool(item["use_gravity_compensation"])
            except ValueError:
                add(issues, f"{name}.use_gravity_compensation", "must be true or false")
        if not stiffness:
            if strict:
                add(issues, f"{name}.stiffness", "missing")
        else:
            check_same_size(issues, f"{name}.stiffness", len(stiffness), len(indices))
        if not damping:
            if strict:
                add(issues, f"{name}.damping", "missing")
        else:
            check_same_size(issues, f"{name}.damping", len(damping), len(indices))


def check_joint_command(root, joint_count, issues):
    target = flatten_numbers(root.get("target_position") if isinstance(root, dict) else None)
    kp = flatten_numbers(root.get("kp") if isinstance(root, dict) else None)
    kd = flatten_numbers(root.get("kd") if isinstance(root, dict) else None)
    steps = flatten_numbers(root.get("num_steps") if isinstance(root, dict) else None, as_int=True)
    if not target:
        add(issues, "target_position", "missing or empty")
    check_same_size(issues, "kp", len(kp), len(target))
    check_same_size(issues, "kd", len(kd), len(target))
    check_same_size(issues, "num_steps", len(steps), len(target))
    if joint_count >= 0:
        check_same_size(issues, "target_position", len(target), joint_count)
    for i, step in enumerate(steps):
        if step < 2:
            add(issues, "num_steps", f"index {i} must be >= 2")


def check_override(root, joint_count, issues):
    root = root if isinstance(root, dict) else {}
    indices = flatten_numbers(root.get("joint_indices"), as_int=True)
    start_pos = flatten_numbers(root.get("start_pos"))
    end_pos = flatten_numbers(root.get("end_pos"))
    velocity = flatten_numbers(root.get("velocity"))
    feed_forward = flatten_numbers(root.get("feed_forward_torque"))
    torque = flatten_numbers(root.get("torque"))
    stiffness = flatten_numbers(root.get("stiffness"))
    damping = flatten_numbers(root.get("damping"))
    if not indices:
        add(issues, "joint_indices", "missing or empty")
    check_indices(issues, indices, joint_count)
    count = len(indices)
    check_same_size(issues, "start_pos", len(start_pos), count)
    check_same_size(issues, "end_pos", len(end_pos), count)
    check_same_size(issues, "velocity", len(velocity), count)
    check_same_size(issues, "feed_forward_torque", len(feed_forward), count)
    check_same_size(issues, "torque", len(torque), count)
    if not stiffness:
        add(issues, "stiffness", "missing")
    else:
        check_same_size(issues, "stiffness", len(stiffness), count)
    if not damping:
        add(issues, "damping", "missing")
    else:
        check_same_size(issues, "damping", len(damping), count)
    duration = -1.0
    duration_ok = False
    if root.get("duration") is not None:
        try:
            duration = ensure_finite_number(root["duration"])
            duration_ok = True
        except ValueError:
            duration_ok = False
    if not duration_ok or not duration > 0.0:
        add(issues, "duration", "execution time must be a finite number > 0")
    freq = -1.0
    freq_ok = False
    if root.get("timer_freq") is not None:
        try:
            freq = ensure_finite_number(root["timer_freq"])
            freq_ok = True
        except ValueError:
            freq_ok = False
    if not freq_ok or not freq > 0.0:
        add(issues, "timer_freq", "must be a finite number > 0")


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    file_path = ""
    model = ""
    check = ""
    strict = False
    as_json = False
    # --domain-id is optional here, unlike the other examples.
    filtered = [argv[0]]
    index = 1
    while index < len(argv):
        arg = argv[index]
        if arg == "--file" and index + 1 < len(argv):
            index += 1
            file_path = argv[index]
        elif arg == "--model" and index + 1 < len(argv):
            index += 1
            model = argv[index]
        elif arg == "--check" and index + 1 < len(argv):
            index += 1
            check = argv[index]
        elif arg == "--strict":
            strict = True
        elif arg == "--json":
            as_json = True
        elif arg in ("--domain-id", "--interface", "--network-address") and index + 1 < len(argv):
            filtered.extend((arg, argv[index + 1]))
            index += 1
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if not file_path or check not in ("joint-plan", "joint-command", "override"):
        print_usage(argv[0])
        return 2
    if model and model_joints(model) < 0:
        print("model must be pm01, t800, or t800pro", file=sys.stderr)
        return 2
    have_domain = any(item == "--domain-id" for item in filtered)
    options = None
    if have_domain:
        options, rest = parse_session_args(filtered)
        if options is None or rest:
            if options is not None and options.dds_domain_id < 0:
                print("domain-id must be >= 0", file=sys.stderr)
            else:
                print_usage(argv[0])
            return 2

    issues = []
    joint_count = -1 if not model else model_joints(model)
    if options is not None:
        session = create_session_or_exit(options)
        joint = wait_joint_state(session)
        if joint is None:
            add(issues, "joint_state", "no sample")
        elif joint_count >= 0 and len(joint.position) != joint_count:
            add(
                issues,
                "joint_state",
                f"sample length {len(joint.position)} != model {model} count {joint_count}",
            )
        else:
            joint_count = len(joint.position)
        session.close()

    if not Path(file_path).is_file():
        add(issues, "file", f"not found: {file_path}")
    else:
        try:
            root = load_file(file_path)
            if check == "joint-plan":
                check_joint_plan(root, strict, joint_count, issues)
            elif check == "joint-command":
                check_joint_command(root, joint_count, issues)
            else:
                check_override(root, joint_count, issues)
        except (OSError, ValueError, TypeError, yaml.YAMLError) as ex:
            add(issues, "yaml", str(ex))

    if as_json:
        print(
            json.dumps(
                {
                    "ok": not issues,
                    "check": check,
                    "errors": [{"field": field, "reason": reason} for field, reason in issues],
                },
                ensure_ascii=False,
            )
        )
    elif not issues:
        line = f"check=ok file={file_path} kind={check}"
        if joint_count >= 0:
            line += f" joint_count={joint_count}"
        print(line)
    else:
        print(f"check=failed file={file_path} kind={check}")
        for field, reason in issues:
            print(f"error field={field} reason={reason}")
        print("no command sent")
    return 0 if not issues else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
