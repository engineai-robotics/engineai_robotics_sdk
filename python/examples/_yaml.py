"""YAML helpers for scenario examples. The SDK package does not depend on PyYAML."""

import sys
from pathlib import Path

from _cli import ensure_finite_number, ensure_int, parse_yaml_bool

try:
    import yaml
except ImportError:
    print(
        "PyYAML is required for this example. Install python3-yaml.",
        file=sys.stderr,
    )
    raise SystemExit(1)


# python/examples/_yaml.py -> repository root -> cpp/examples/config
DEFAULT_ACTIONS_DIR = Path(__file__).resolve().parents[2] / "cpp" / "examples" / "config"


def flatten_numbers(node, as_int=False):
    out = []

    def walk(value):
        if value is None:
            return
        if isinstance(value, bool):
            raise ValueError("expected YAML sequence or scalar of numbers")
        if isinstance(value, (int, float)):
            if as_int:
                out.append(ensure_int(value))
            else:
                out.append(ensure_finite_number(value))
            return
        if isinstance(value, list):
            for child in value:
                walk(child)
            return
        raise ValueError("expected YAML sequence or scalar of numbers")

    walk(node)
    return out


def load_file(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def load_motion_plan(path):
    if not Path(path).is_file():
        return None, f"config not found: {path}"
    try:
        root = load_file(path)
    except (OSError, yaml.YAMLError, ValueError) as ex:
        return None, str(ex)
    try:
        return _load_motion_plan_root(root), None
    except ValueError as ex:
        return None, str(ex)


def _load_motion_plan_root(root):
    plan = root.get("motion_plan") if isinstance(root, dict) else None
    if not isinstance(plan, list) or len(plan) < 2:
        raise ValueError("config missing motion_plan sequence")
    indices = flatten_numbers(plan[0].get("joint_indices") if isinstance(plan[0], dict) else None, as_int=True)
    if not indices:
        raise ValueError("config missing joint_indices")
    tasks = []
    for index in range(1, len(plan)):
        item = plan[index] if isinstance(plan[index], dict) else {}
        name = item.get("motion") or f"motion_{index}"
        request_type = str(item.get("request_type") or "").upper()
        task = {
            "name": str(name),
            "reset": request_type == "RESET",
            "target_positions": [],
            "duration": 2.0,
            "use_gravity_compensation": True,
            "stiffness": [],
            "damping": [],
        }
        if task["reset"]:
            tasks.append(task)
            continue
        task["target_positions"] = flatten_numbers(item.get("target_positions"))
        if "duration" in item and item["duration"] is not None:
            duration = ensure_finite_number(item["duration"])
            if not duration > 0.0:
                raise ValueError(f"task '{task['name']}' duration must be > 0")
            task["duration"] = duration
        if "use_gravity_compensation" in item and item["use_gravity_compensation"] is not None:
            try:
                task["use_gravity_compensation"] = parse_yaml_bool(item["use_gravity_compensation"])
            except ValueError as exc:
                raise ValueError(
                    f"task '{task['name']}' use_gravity_compensation must be true or false"
                ) from exc
        task["stiffness"] = flatten_numbers(item.get("stiffness"))
        task["damping"] = flatten_numbers(item.get("damping"))
        if len(task["target_positions"]) != len(indices):
            raise ValueError(f"task '{task['name']}' target_positions size != joint_indices")
        tasks.append(task)
    if not tasks:
        raise ValueError("no motion tasks in config")
    return {"joint_indices": indices, "tasks": tasks}


def resolve_action_yaml(actions_dir, model, action):
    if model not in ("pm01", "t800", "t800pro"):
        return None, "model must be pm01, t800, or t800pro"
    root = actions_dir or str(DEFAULT_ACTIONS_DIR)
    if action == "wave":
        if model == "t800pro":
            return None, "t800pro has no motion_plan_wave_hands.yaml; use handshake or reset"
        filename = "motion_plan_wave_hands.yaml"
    elif action == "handshake":
        filename = "motion_plan_shake_hand.yaml"
    elif action == "reset":
        filename = "motion_plan_reset.yaml"
    else:
        return None, "action must be wave, handshake, or reset"
    return str(Path(root) / model / filename), None


def num(value):
    return f"{value:.10g}"


def print_plan_preview(path, plan, speed_scale):
    print(f"config={path}")
    print("joint_indices:", " ".join(str(index) for index in plan["joint_indices"]))
    total = 0.0
    for task in plan["tasks"]:
        seconds = 0.0 if task["reset"] else task["duration"] / speed_scale
        total += seconds
        kind = "RESET" if task["reset"] else "PLAN_EXECUTE"
        print(
            f"step name={task['name']} type={kind} joints={len(plan['joint_indices'])} duration_s={num(seconds)}"
        )
    print(f"estimated_duration_s={num(total)}")
