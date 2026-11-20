#!/usr/bin/env python3
"""connectivity_doctor: local session and whether robot samples arrived."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import Robot

from _cli import err_name, motion_or_none, print_no_robot_data_hint, read_strict_int, wants_help

CHECK_NAMES = ("heartbeat", "motion", "joint", "imu", "power")


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--timeout SEC] [--check heartbeat,motion,joint,imu,power] [--json]\n"
        "  Default timeout is 5 s and every check is enabled.\n"
        "  Domain and interface are the values passed in, not a discovered binding.",
        file=sys.stderr,
    )


def parse_checks(text):
    checks = {name: False for name in CHECK_NAMES}
    for item in text.split(","):
        if item not in checks:
            return None
        checks[item] = True
    if not any(checks.values()):
        return None
    return checks


def receive_time(session, name):
    if name == "heartbeat":
        return session.node_heartbeat.last_receive_time()
    if name == "motion":
        return session.motion.last_receive_time()
    if name == "joint":
        return session.joint_state.last_receive_time()
    if name == "imu":
        return session.imu.last_receive_time()
    return session.power.last_receive_time()


def since_text(received):
    if received is None:
        return "(no sample)"
    import time

    elapsed_ms = int((time.monotonic() - received) * 1000)
    return f"{elapsed_ms} ms"


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    from _cli import parse_session_args

    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    timeout_sec = 5
    as_json = False
    checks = {name: True for name in CHECK_NAMES}
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--json":
            as_json = True
        elif arg == "--timeout" and index + 1 < len(rest):
            index += 1
            parsed = read_strict_int(rest[index], "timeout")
            if parsed is None:
                return 2
            timeout_sec = parsed
        elif arg == "--check" and index + 1 < len(rest):
            index += 1
            checks = parse_checks(rest[index])
            if checks is None:
                print("--check must be a comma list of heartbeat,motion,joint,imu,power", file=sys.stderr)
                return 2
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if timeout_sec <= 0:
        print("timeout must be > 0", file=sys.stderr)
        return 2

    interface = options.network_interface_name or "(empty)"
    address = options.network_address or "(empty)"
    result = Robot.create(options)
    if not result:
        message = "local DDS session was not created. Check the domain id and interface name."
        if as_json:
            import json

            print(
                json.dumps(
                    {
                        "session": False,
                        "robot_sample": False,
                        "error": err_name(result.error),
                        "dds_domain_id": options.dds_domain_id,
                        "network_interface": interface,
                        "network_address": address,
                        "message": message,
                    },
                    ensure_ascii=False,
                )
            )
            print(message, file=sys.stderr)
        else:
            print(f"session=failed error={err_name(result.error)}")
            print(
                f"stage=session_create dds_domain_id={options.dds_domain_id} "
                f"network_interface={interface} network_address={address}"
            )
            print(f"hint: {message}")
        return 1

    session = result.session
    import time

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not session.is_open():
            break
        if all(not checks[name] or receive_time(session, name) is not None for name in CHECK_NAMES):
            break
        time.sleep(0.02)

    def present(name):
        return receive_time(session, name) is not None

    robot_sample = any(checks[name] and present(name) for name in CHECK_NAMES)
    ready = all(not checks[name] or present(name) for name in CHECK_NAMES)
    if as_json:
        import json

        payload = {
            "session": session.is_open(),
            "robot_sample": robot_sample,
            "dds_domain_id": options.dds_domain_id,
            "network_interface": interface,
            "checks": {name: present(name) for name in CHECK_NAMES if checks[name]},
        }
        if not ready:
            missing = " ".join(name for name in CHECK_NAMES if checks[name] and not present(name))
            payload["message"] = f"missing {missing}".rstrip()
        print(json.dumps(payload, ensure_ascii=False))
    else:
        print(f"session={'ok' if session.is_open() else 'failed'}")
        print(f"robot_sample={'yes' if robot_sample else 'no'}")
        print(f"dds_domain_id={options.dds_domain_id}")
        print(f"network_interface={interface}")
        print(f"network_address={address}")
        if checks["heartbeat"]:
            hb = session.node_heartbeat.latest()
            if hb is None:
                print("node_heartbeat (no sample)")
            else:
                line = f"node_heartbeat node={hb.node_name} status={hb.node_status} err={hb.error_code}"
                if hb.error_message:
                    line += f" message={hb.error_message}"
                print(line)
        if checks["motion"]:
            print(f"current_motion={motion_or_none(session)}")
        if checks["joint"]:
            joint = session.joint_state.latest()
            if joint is None:
                print("joint_state (no sample)")
            else:
                print(f"joint_state n={len(joint.position)}")
        if checks["imu"]:
            imu = session.imu.latest()
            if imu is None:
                print("imu (no sample)")
            else:
                print(f"imu rpy=({imu.rpy.x}, {imu.rpy.y}, {imu.rpy.z})")
        if checks["power"]:
            power = session.power.latest()
            if power is None:
                print("power (no sample)")
            else:
                board = power.power_info
                line = (
                    f"power enable={int(bool(board.enable))} pct={board.percentage} "
                    f"V={board.voltage} A={board.current} err={board.error_code}"
                )
                if power.battery_info is not None:
                    battery = power.battery_info
                    line += f" bms_soc={battery.soc} bms_temp={battery.temp} bms_current_ma={battery.bms_current}"
                else:
                    line += " bms=(unavailable)"
                print(line)
        for name in CHECK_NAMES:
            if checks[name]:
                print(f"{name}_since_receive={since_text(receive_time(session, name))}")

    if not ready:
        missing = " ".join(name for name in CHECK_NAMES if checks[name] and not present(name))
        stage = f"stage=sample missing={missing}".rstrip()
        print(stage, file=sys.stderr if as_json else sys.stdout)
        print_no_robot_data_hint()
        session.close()
        return 1
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
