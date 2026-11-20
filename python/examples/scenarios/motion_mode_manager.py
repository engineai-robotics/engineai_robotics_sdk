#!/usr/bin/env python3
"""motion_mode_manager: list the current motion and request a listed transition.

Safety when the target is joint_bridge: this mode commands the joints
directly. The standing pose is not necessarily stable, so hoist the robot
before use. Both feet must be on the ground before entering walk.
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _cli import (
    create_session_or_exit,
    err_name,
    motion_is,
    motion_or_none,
    parse_session_args,
    read_strict_int,
    print_no_robot_data_hint,
    wait_for_motion_sample,
    wants_help,
)
from engineai_sdkpy import SessionError


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--list] [--target-motion NAME] [--timeout SEC] [--json]\n"
        "  --list is the default when --target-motion is omitted.\n"
        "  A target outside available_transitions is not sent.",
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
    as_json = False
    target = ""
    timeout_sec = 8
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--list":
            pass
        elif arg == "--json":
            as_json = True
        elif arg == "--target-motion" and index + 1 < len(rest):
            index += 1
            target = rest[index]
        elif arg == "--timeout" and index + 1 < len(rest):
            index += 1
            parsed = read_strict_int(rest[index], "timeout")
            if parsed is None:
                return 2
            timeout_sec = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if timeout_sec <= 0:
        print("timeout must be > 0", file=sys.stderr)
        return 2

    session = create_session_or_exit(options)
    if not wait_for_motion_sample(session, timeout_sec * 1000):
        print_no_robot_data_hint()
        session.close()
        return 1
    current = motion_or_none(session)
    transitions = session.motion.available_transitions()
    names = list(transitions) if transitions is not None else []

    def print_state(result, detail):
        if not as_json:
            print(f"current={current}")
            line = "available_transitions:"
            if not names:
                line += " (none)"
            for name in names:
                line += f" {name}"
            print(line)
            text = f"result={result}"
            if detail:
                text += f" {detail}"
            print(text)
            return
        payload = {
            "current": current,
            "available": names,
            "target": target or None,
            "result": result,
            "detail": detail,
        }
        print(json.dumps(payload, separators=(", ", ": ")).replace(": ", ": ").replace(", ", ", "))

    # Match the C++ field order and spacing more closely than json.dumps.
    def print_state_json(result, detail):
        available = ", ".join(json.dumps(name) for name in names)
        target_text = "null" if not target else json.dumps(target)
        print(
            '{"current": '
            + json.dumps(current)
            + ', "available": ['
            + available
            + '], "target": '
            + target_text
            + ', "result": '
            + json.dumps(result)
            + ', "detail": '
            + json.dumps(detail)
            + "}"
        )

    def emit(result, detail):
        if as_json:
            print_state_json(result, detail)
        else:
            print_state(result, detail)

    if not target:
        emit("listed", "")
        session.close()
        return 0 if names else 1
    if transitions is None:
        emit("no_transitions", "available_transitions has no sample")
        session.close()
        return 1
    if motion_is(session, target):
        emit("already_active", "current motion already matches")
        session.close()
        return 0
    if target not in names:
        emit("unreachable", "target is not in available_transitions; request was not sent")
        session.close()
        return 1
    err = session.motion.request_transition(target)
    if not as_json:
        print(f"request_transition -> {err_name(err)}")
    if err != SessionError.kOk:
        emit("send_failed", err_name(err))
        session.close()
        return 1
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if motion_is(session, target):
            emit("switched", "current motion matches target")
            session.close()
            return 0
        if not as_json:
            print(f"waiting current={motion_or_none(session)}")
        time.sleep(0.2)
    emit("timeout", "current=" + motion_or_none(session))
    session.close()
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
