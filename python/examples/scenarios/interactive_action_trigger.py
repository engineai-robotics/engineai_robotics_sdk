#!/usr/bin/env python3
"""interactive_action_trigger: one key starts one existing motion-plan action."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import GamepadButton, JointMotionPlanStatus

from _cli import (
    MOTION_MAX_AGE,
    STATE_MAX_AGE,
    TerminalRaw,
    catch_sigint,
    create_session_or_exit,
    is_fresh,
    motion_is,
    motion_or_none,
    parse_session_args,
    read_finite_float,
    require_current_motion,
    stop_requested,
    wants_help,
)
from _plan import plan_status_name, run_motion_plan
from _yaml import DEFAULT_ACTIONS_DIR, load_file, load_motion_plan, print_plan_preview, resolve_action_yaml

GAMEPAD_CODES = {
    "lb": int(GamepadButton.kLb),
    "rb": int(GamepadButton.kRb),
    "a": int(GamepadButton.kA),
    "b": int(GamepadButton.kB),
    "x": int(GamepadButton.kX),
    "y": int(GamepadButton.kY),
    "back": int(GamepadButton.kBack),
    "start": int(GamepadButton.kStart),
}


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       --model pm01|t800|t800pro --input keyboard|gamepad\n"
        "       [--bindings PATH] [--actions DIR] [--timeout SEC] [--poll-rate HZ] [--list-bindings]\n"
        "  Default bindings: cpp/examples/config/interactive_bindings.yaml\n"
        "  Requires current motion lower_body_balance. A new action is refused while a plan is still running.",
        file=sys.stderr,
    )


def default_bindings(mode):
    if mode == "gamepad":
        return [
            (int(GamepadButton.kA), "wave"),
            (int(GamepadButton.kB), "handshake"),
            (int(GamepadButton.kX), "reset"),
        ]
    return [(ord("1"), "wave"), (ord("2"), "handshake"), (ord("3"), "reset")]


def load_bindings(path, mode, bindings):
    if not Path(path).is_file():
        return bindings, False
    try:
        root = load_file(path)
    except (OSError, ValueError):
        return bindings, False
    table = root.get(mode) if isinstance(root, dict) else None
    if not isinstance(table, dict):
        return bindings, False
    loaded = []
    for key, action in table.items():
        if action not in ("wave", "handshake", "reset"):
            continue
        code = -1
        if mode == "keyboard" and len(str(key)) == 1:
            code = ord(str(key))
        elif mode == "gamepad":
            code = GAMEPAD_CODES.get(str(key), -1)
        if code >= 0:
            loaded.append((code, action))
    if not loaded:
        return bindings, False
    return loaded, True


def print_bindings(mode, bindings):
    print(f"input={mode}")
    for code, action in bindings:
        if mode == "keyboard":
            print(f"key={chr(code)} action={action}")
        else:
            print(f"button={code} action={action}")
    if mode == "keyboard":
        print("key=space action=cancel")
        print("key=esc action=quit")
    else:
        print("button=Y action=cancel")
        print("button=Start action=quit")
        print("note: gamepad mapping is a local example and is not verified against onboard button functions")


def find_binding(bindings, code):
    for item in bindings:
        if item[0] == code:
            return item[1]
    return None


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    mode = "keyboard"
    model = ""
    bindings_path = str(DEFAULT_ACTIONS_DIR / "interactive_bindings.yaml")
    actions_dir = str(DEFAULT_ACTIONS_DIR)
    timeout_s = 30.0
    poll_rate = 20.0
    list_bindings = False
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--model" and index + 1 < len(rest):
            index += 1
            model = rest[index]
        elif arg == "--input" and index + 1 < len(rest):
            index += 1
            mode = rest[index]
        elif arg == "--bindings" and index + 1 < len(rest):
            index += 1
            bindings_path = rest[index]
        elif arg == "--actions" and index + 1 < len(rest):
            index += 1
            actions_dir = rest[index]
        elif arg == "--timeout" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "timeout")
            if parsed is None:
                return 2
            timeout_s = parsed
        elif arg == "--poll-rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "poll-rate")
            if parsed is None:
                return 2
            poll_rate = parsed
        elif arg == "--list-bindings":
            list_bindings = True
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if mode not in ("keyboard", "gamepad"):
        print("--input must be keyboard or gamepad", file=sys.stderr)
        return 2
    if timeout_s <= 0.0 or poll_rate <= 0.0:
        print("timeout and poll-rate must be > 0", file=sys.stderr)
        return 2
    bindings = default_bindings(mode)
    bindings, loaded = load_bindings(bindings_path, mode, bindings)
    if not loaded:
        print("bindings file not loaded, using built-in map")
    print_bindings(mode, bindings)
    if list_bindings:
        return 0
    if not model:
        print("Missing required --model", file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    if not require_current_motion(session, "lower_body_balance"):
        session.close()
        return 1
    terminal = TerminalRaw(mode == "keyboard")
    if mode == "keyboard" and not terminal.active:
        print("stdin is not a terminal", file=sys.stderr)
        session.close()
        return 2
    print(f"ready model={model} motion={motion_or_none(session)}")
    catch_sigint()
    state = {
        "quit": False,
        "buttons": [0] * 12,
        "have_buttons": False,
        "was_connected": False,
        "wait_release": True,
        "last_key": "",
        "last_time": 0.0,
    }

    def poll_input(active_session, _user):
        if mode == "keyboard":
            for ch in terminal.read_chars():
                if ch in (" ", "\x1b", "\x03"):
                    if ch != " ":
                        state["quit"] = True
                    print("cancel requested")
                    return True
                action = find_binding(bindings, ord(ch))
                if action:
                    print(f"rejected action={action} reason=planner busy")
            return False
        pad = active_session.gamepad.latest()
        if pad is None:
            return False
        if not pad.hardware_connected:
            cancel = False
            if state["was_connected"]:
                print("gamepad disconnected; cancel requested")
                cancel = True
            state["was_connected"] = False
            state["wait_release"] = True
            return cancel
        if not state["was_connected"]:
            state["wait_release"] = True
            state["was_connected"] = True
        y = int(GamepadButton.kY)
        start = int(GamepadButton.kStart)
        cancel = pad.digital_states[y] != 0 or pad.digital_states[start] != 0
        if cancel:
            if pad.digital_states[start] != 0:
                state["quit"] = True
            print("cancel requested")
        for code, action in bindings:
            if pad.digital_states[code] != 0:
                print(f"rejected action={action} reason=planner busy")
        state["buttons"] = list(pad.digital_states)
        state["have_buttons"] = True
        return cancel

    def pressed_action():
        if mode == "keyboard":
            action = ""
            for ch in terminal.read_chars():
                if ch in ("\x1b", "\x03"):
                    state["quit"] = True
                    return ""
                if ch == " ":
                    print("cancel ignored reason=no active request")
                    continue
                now = time.monotonic()
                repeat = ch == state["last_key"] and now - state["last_time"] < 0.25
                state["last_key"] = ch
                state["last_time"] = now
                if repeat:
                    continue
                found = find_binding(bindings, ord(ch))
                if found:
                    action = found
            return action
        pad = session.gamepad.latest()
        if pad is None:
            print("gamepad=(no sample). Check that the robot is publishing gamepad data.")
            return ""
        if not pad.hardware_connected:
            print("gamepad hardware_connected=false. Connect the pad and confirm the robot publishes it.")
            state["was_connected"] = False
            state["wait_release"] = True
            state["have_buttons"] = False
            return ""
        if not state["was_connected"]:
            state["wait_release"] = True
            state["was_connected"] = True
            state["buttons"] = list(pad.digital_states)
            state["have_buttons"] = True
            print("gamepad connected; release held buttons before a new action")
            return ""
        any_down = False
        action = ""
        for code, name in bindings:
            now = pad.digital_states[code]
            prev = state["buttons"][code] if state["have_buttons"] else 0
            if now != 0:
                any_down = True
                if not state["wait_release"] and prev == 0 and not action:
                    action = name
        if pad.digital_states[int(GamepadButton.kStart)] != 0:
            state["quit"] = True
        if not any_down:
            state["wait_release"] = False
        state["buttons"] = list(pad.digital_states)
        state["have_buttons"] = True
        return action

    try:
        while not state["quit"] and not stop_requested():
            action = pressed_action()
            if state["quit"] or stop_requested():
                break
            if action:
                if not motion_is(session, "lower_body_balance"):
                    print(f"rejected action={action} reason=current motion is not lower_body_balance")
                elif not is_fresh(session.motion.last_receive_time(), MOTION_MAX_AGE) or not is_fresh(
                    session.joint_state.last_receive_time(), STATE_MAX_AGE
                ):
                    print(f"rejected action={action} reason=missing or stale motion / joint_state sample")
                else:
                    planner = session.joint_motion_planner.latest_state()
                    if planner is not None and planner.status != JointMotionPlanStatus.kIdle:
                        print(f"rejected action={action} reason=planner {plan_status_name(planner.status)}")
                    else:
                        path, error = resolve_action_yaml(actions_dir, model, action)
                        if path is None:
                            print(f"rejected action={action} reason={error}")
                        else:
                            plan, error = load_motion_plan(path)
                            if plan is None:
                                print(f"rejected action={action} reason={error}")
                            else:
                                print(f"trigger action={action} motion={motion_or_none(session)}")
                                print_plan_preview(path, plan, 1.0)
                                code = run_motion_plan(
                                    session,
                                    plan,
                                    {
                                        "speed_scale": 1.0,
                                        "timeout_s": timeout_s,
                                        "cancel_after_ms": -1,
                                        "dry_run": False,
                                    },
                                    poll_input,
                                    None,
                                )
                                print(f"result={code} action={action}")
            time.sleep(1.0 / poll_rate)
    finally:
        terminal.close()
    print("quit")
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
