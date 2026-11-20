#!/usr/bin/env python3
"""robot_status_snapshot: one readable status table, or JSON with --json."""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import Robot

from _cli import (
    HEARTBEAT_MAX_AGE,
    MOTION_MAX_AGE,
    STATE_MAX_AGE,
    catch_sigint,
    err_name,
    is_fresh,
    parse_session_args,
    print_no_robot_data_hint,
    read_finite_float,
    stop_requested,
    wants_help,
)

LOW_SOC = 20.0
CRITICAL_SOC = 5.0
MAX_BATTERY_TEMP_C = 55.0
MOS_OVER_TEMPERATURE = 0x3401
MOTOR_OVER_TEMPERATURE = 0x3403
DRIVER_ERROR_BIT = 0x0800


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--json] [--once] [--watch] [--interval SEC] [--show-diagnostics]\n"
        "  Prints one status table. --watch repeats until Ctrl+C. --once is the default.\n"
        "  Exit 1 on a single snapshot when the session fails, no robot sample arrives, or any alert is set.\n"
        "  A cached sample older than its threshold is marked stale and counts as an alert.",
        file=sys.stderr,
    )


def json_escape(text):
    out = []
    for ch in text:
        code = ord(ch)
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\n":
            out.append("\\n")
        elif ch == "\r":
            out.append("\\r")
        elif ch == "\t":
            out.append("\\t")
        elif code < 0x20:
            out.append(f"\\u{code:04x}")
        else:
            out.append(ch)
    return "".join(out)


def json_string(text):
    return '"' + json_escape(text) + '"'


def json_number(value):
    if not math.isfinite(value):
        return "null"
    return str(value)


def index_list(indices):
    return ",".join(str(index) for index in indices)


def driver_code(code):
    return int(code) & 0xFFFF


def hex_code(code):
    return hex(code)


def classify_motors(motors):
    offline = [i for i, flag in enumerate(motors.offline) if flag != 0]
    over_temp = []
    fault = []
    warning = []
    for i, code in enumerate(motors.error_code):
        value = driver_code(code)
        if value == 0:
            continue
        if value in (MOS_OVER_TEMPERATURE, MOTOR_OVER_TEMPERATURE):
            over_temp.append(i)
        elif value & DRIVER_ERROR_BIT:
            fault.append(i)
        else:
            warning.append(i)
    max_motor = max(motors.motor_temperature) if motors.motor_temperature else None
    max_mos = max(motors.mos_temperature) if motors.mos_temperature else None
    return offline, over_temp, fault, warning, max_motor, max_mos


def add(alerts, item, code, detail):
    alerts.append((item, code, detail))


def age_ms(received):
    if received is None:
        return -1
    return int((time.monotonic() - received) * 1000)


def note_stale(item, received, max_age, alerts, detail):
    if is_fresh(received, max_age):
        return detail
    note = f"since_receive_ms={age_ms(received)}"
    add(alerts, item, "stale", note)
    return f"{detail} {note}"


def freshness_json(received, max_age):
    stale = "true" if not is_fresh(received, max_age) else "false"
    return f', "stale": {stale}, "since_receive_ms": {age_ms(received)}'


def item_alert(alerts, item):
    return any(alert_item == item for alert_item, _code, _detail in alerts)


def collect_heartbeat(session, alerts):
    hb = session.node_heartbeat.latest()
    if hb is None:
        add(alerts, "heartbeat", "no_sample", "no sample")
        return "no sample"
    detail = f"node={hb.node_name} status={hb.node_status} err={hb.error_code}"
    if hb.error_message:
        detail += f" message={hb.error_message}"
    if hb.node_status == "error" or hb.error_code != 0 or hb.error_message:
        add(alerts, "heartbeat", "node_error", detail)
    return note_stale("heartbeat", session.node_heartbeat.last_receive_time(), HEARTBEAT_MAX_AGE, alerts, detail)


def collect_motion(session, alerts):
    motion = session.motion.current_motion()
    if motion is None:
        add(alerts, "motion", "no_sample", "no sample")
        return "no sample"
    return note_stale("motion", session.motion.last_receive_time(), MOTION_MAX_AGE, alerts, motion)


def collect_imu(session, alerts):
    imu = session.imu.latest()
    if imu is None:
        add(alerts, "imu", "no_sample", "no sample")
        return "no sample"
    detail = f"rpy=({imu.rpy.x}, {imu.rpy.y}, {imu.rpy.z})"
    return note_stale("imu", session.imu.last_receive_time(), STATE_MAX_AGE, alerts, detail)


def collect_power(session, alerts):
    power = session.power.latest()
    if power is None:
        add(alerts, "power", "no_sample", "no sample")
        return "no sample"
    board = power.power_info
    detail = (
        f"enable={int(bool(board.enable))} pct={board.percentage} V={board.voltage} "
        f"A={board.current} err={board.error_code}"
    )
    if board.error_code != 0:
        add(alerts, "power", "power_fault", f"error_code={board.error_code}")
    if math.isfinite(board.percentage) and 0.0 <= board.percentage <= 100.0:
        if board.percentage <= CRITICAL_SOC:
            add(alerts, "power", "critical_battery", f"percentage={board.percentage}")
        elif board.percentage <= LOW_SOC:
            add(alerts, "power", "low_battery", f"percentage={board.percentage}")
    if power.battery_info is not None:
        battery = power.battery_info
        detail += f" soc={battery.soc} temp_c={battery.temp}"
        if math.isfinite(battery.temp) and battery.temp >= MAX_BATTERY_TEMP_C:
            add(alerts, "power", "battery_over_temperature", f"temp_c={battery.temp}")
        if battery.error_word != 0:
            add(alerts, "power", "battery_error", f"error_word={battery.error_word}")
    else:
        detail += " bms=unavailable"
    return note_stale("power", session.power.last_receive_time(), STATE_MAX_AGE, alerts, detail)


def collect_motors(session, alerts):
    motors = session.motor_diagnostics.latest()
    if motors is None:
        add(alerts, "motors", "no_sample", "no sample")
        return "no sample"
    offline, over_temp, fault, warning, max_motor, max_mos = classify_motors(motors)
    if offline:
        add(alerts, "motors", "motor_offline", "indices=" + index_list(offline))
    if over_temp:
        add(alerts, "motors", "motor_over_temperature", "indices=" + index_list(over_temp))
    if fault:
        add(alerts, "motors", "motor_fault", "indices=" + index_list(fault))
    if warning:
        add(alerts, "motors", "motor_warning", "indices=" + index_list(warning))
    detail = (
        f"n={len(motors.offline)} offline={len(offline)} over_temp={len(over_temp)} "
        f"fault={len(fault)} warning={len(warning)}"
    )
    if max_motor is not None:
        detail += f" max_motor_c={max_motor}"
    if max_mos is not None:
        detail += f" max_mos_c={max_mos}"
    return note_stale("motors", session.motor_diagnostics.last_receive_time(), STATE_MAX_AGE, alerts, detail)


def print_table(rows, alerts):
    color = sys.stdout.isatty()
    print(f"{'item':<12}{'status':<8}detail")
    for item, alert, detail in rows:
        status = "ALERT" if alert else "ok"
        if alert and color:
            status_text = f"\033[31m{status:<8}\033[0m"
        else:
            status_text = f"{status:<8}"
        print(f"{item:<12}{status_text}{detail}")
    print(f"alerts={len(alerts)}")
    for item, code, detail in alerts:
        print(f"alert item={item} code={code} {detail}")


def print_json(session_ok, session_error, domain_id, interface_name, session, alerts):
    print("{")
    print(f'  "session_ok": {"true" if session_ok else "false"},')
    print(f'  "session_error": {"null" if not session_error else json_string(session_error)},')
    print(f'  "dds_domain_id": {domain_id},')
    print(f'  "network_interface": {json_string(interface_name)},')
    print(f'  "alert_count": {len(alerts)},')
    print('  "alerts": [', end="")
    for i, (item, code, detail) in enumerate(alerts):
        if i:
            print(",", end="")
        print(f'\n    {{"item": {json_string(item)}, "code": {json_string(code)}, "detail": {json_string(detail)}}}', end="")
    if alerts:
        print("\n  ", end="")
    print("],")
    if session is None:
        print('  "heartbeat": null,\n  "motion": null,\n  "imu": null,\n  "power": null,\n  "motors": null\n}')
        return
    hb = session.node_heartbeat.latest()
    if hb is not None:
        print(
            '  "heartbeat": {"sample": true, "node": '
            + json_string(hb.node_name)
            + ', "status": '
            + json_string(hb.node_status)
            + f', "error_code": {hb.error_code}, "error_message": {json_string(hb.error_message)}'
            + freshness_json(session.node_heartbeat.last_receive_time(), HEARTBEAT_MAX_AGE)
            + "},"
        )
    else:
        print('  "heartbeat": {"sample": false},')
    motion = session.motion.current_motion()
    if motion is not None:
        print(
            '  "motion": {"sample": true, "name": '
            + json_string(motion)
            + freshness_json(session.motion.last_receive_time(), MOTION_MAX_AGE)
            + "},"
        )
    else:
        print('  "motion": {"sample": false},')
    imu = session.imu.latest()
    if imu is not None:
        print(
            '  "imu": {"sample": true, "roll": '
            + json_number(imu.rpy.x)
            + ', "pitch": '
            + json_number(imu.rpy.y)
            + ', "yaw": '
            + json_number(imu.rpy.z)
            + freshness_json(session.imu.last_receive_time(), STATE_MAX_AGE)
            + "},"
        )
    else:
        print('  "imu": {"sample": false},')
    power = session.power.latest()
    if power is not None:
        board = power.power_info
        print(
            '  "power": {"sample": true, "enable": '
            + ("true" if board.enable else "false")
            + ', "percentage": '
            + json_number(board.percentage)
            + ', "voltage": '
            + json_number(board.voltage)
            + ', "current": '
            + json_number(board.current)
            + f', "error_code": {board.error_code}',
            end="",
        )
        if power.battery_info is not None:
            battery = power.battery_info
            print(
                ', "battery": {"soc": '
                + str(battery.soc)
                + ', "temp_c": '
                + json_number(battery.temp)
                + ', "error_word": '
                + str(battery.error_word)
                + "}"
                + freshness_json(session.power.last_receive_time(), STATE_MAX_AGE)
                + "},"
            )
        else:
            print(', "battery": null' + freshness_json(session.power.last_receive_time(), STATE_MAX_AGE) + "},")
    else:
        print('  "power": {"sample": false},')
    motors = session.motor_diagnostics.latest()
    if motors is not None:
        offline, over_temp, fault, warning, max_motor, max_mos = classify_motors(motors)

        def print_codes(indices):
            parts = []
            for index in indices:
                code = driver_code(motors.error_code[index])
                parts.append('{"index": ' + str(index) + ', "code": ' + json_string(hex_code(code)) + "}")
            return ", ".join(parts)

        print(
            '  "motors": {"sample": true, "count": '
            + str(len(motors.offline))
            + ', "offline": ['
            + ", ".join(str(index) for index in offline)
            + '], "over_temperature": ['
            + print_codes(over_temp)
            + '], "fault": ['
            + print_codes(fault)
            + '], "warning": ['
            + print_codes(warning)
            + "]",
            end="",
        )
        if max_motor is not None:
            print(f', "max_motor_c": {json_number(max_motor)}', end="")
        if max_mos is not None:
            print(f', "max_mos_c": {json_number(max_mos)}', end="")
        print(freshness_json(session.motor_diagnostics.last_receive_time(), STATE_MAX_AGE) + "}")
    else:
        print('  "motors": {"sample": false}')
    print("}")


def has_source(session):
    return any(
        (
            session.node_heartbeat.latest(),
            session.motion.current_motion(),
            session.imu.latest(),
            session.power.latest(),
            session.motor_diagnostics.latest(),
        )
    )


def has_all_sources(session):
    return all(
        (
            session.node_heartbeat.latest(),
            session.motion.current_motion(),
            session.imu.latest(),
            session.power.latest(),
            session.motor_diagnostics.latest(),
        )
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
    watch = False
    once = False
    show_diagnostics = False
    interval = 1.0
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--json":
            as_json = True
        elif arg == "--watch":
            watch = True
        elif arg == "--once":
            once = True
        elif arg == "--show-diagnostics":
            show_diagnostics = True
        elif arg == "--interval" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "interval")
            if parsed is None:
                return 2
            interval = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if watch and once:
        print("pass only one of --watch and --once", file=sys.stderr)
        return 2
    if interval <= 0.0:
        print("interval must be > 0", file=sys.stderr)
        return 2
    interface = options.network_interface_name or "(empty)"
    result = Robot.create(options)
    if not result:
        error = err_name(result.error)
        alerts = [("session", "session_failed", error)]
        if as_json:
            print_json(False, error, options.dds_domain_id, interface, None, alerts)
        else:
            print_table([("session", True, "failed error=" + error)], alerts)
        return 1
    session = result.session
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not session.is_open() or has_all_sources(session):
            break
        time.sleep(0.02)
    catch_sigint()
    clean = True
    while True:
        any_sample = has_source(session)
        alerts = []
        heartbeat = collect_heartbeat(session, alerts)
        motion = collect_motion(session, alerts)
        joint = session.joint_state.latest()
        if joint is not None:
            joints = note_stale(
                "joints",
                session.joint_state.last_receive_time(),
                STATE_MAX_AGE,
                alerts,
                f"n={len(joint.position)}",
            )
        else:
            add(alerts, "joints", "no_sample", "no sample")
            joints = "no sample"
        imu = collect_imu(session, alerts)
        pad = session.gamepad.latest()
        if pad is not None:
            gamepad = note_stale(
                "gamepad",
                session.gamepad.last_receive_time(),
                STATE_MAX_AGE,
                alerts,
                f"connected={'true' if pad.hardware_connected else 'false'}",
            )
            if not pad.hardware_connected:
                add(alerts, "gamepad", "not_connected", "hardware_connected=false")
        else:
            add(alerts, "gamepad", "no_sample", "no sample")
            gamepad = "no sample"
        power = collect_power(session, alerts)
        motors = collect_motors(session, alerts)
        if not any_sample and session.joint_state.latest() is None and session.gamepad.latest() is None:
            add(alerts, "session", "no_robot_sample", "no heartbeat, motion, imu, power, motor, joint, or gamepad sample")
        rows = [
            ("session", item_alert(alerts, "session"), "ok" if session.is_open() else "not open"),
            ("heartbeat", item_alert(alerts, "heartbeat"), heartbeat),
            ("motion", item_alert(alerts, "motion"), motion),
            ("joints", item_alert(alerts, "joints"), joints),
            ("imu", item_alert(alerts, "imu"), imu),
            ("gamepad", item_alert(alerts, "gamepad"), gamepad),
            ("power", item_alert(alerts, "power"), power),
            ("motors", item_alert(alerts, "motors"), motors),
        ]
        if show_diagnostics:
            diag = session.motor_diagnostics.latest()
            if diag is not None:
                offline, over_temp, _fault, _warning, _max_motor, _max_mos = classify_motors(diag)
                rows.append(("diag", False, "offline=" + index_list(offline) + " over_temp=" + index_list(over_temp)))
        if not as_json:
            print(f"dds_domain_id={options.dds_domain_id}")
            print(f"network_interface={interface}")
            print_table(rows, alerts)
        else:
            print_json(session.is_open(), "", options.dds_domain_id, interface, session, alerts)
        if not any_sample:
            print_no_robot_data_hint()
        clean = not alerts
        if not watch or stop_requested():
            break
        time.sleep(interval)
    session.close()
    return 0 if clean else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
