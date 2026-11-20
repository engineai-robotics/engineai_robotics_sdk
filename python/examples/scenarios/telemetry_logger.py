#!/usr/bin/env python3
"""telemetry_logger: write polled robot samples to CSV or JSONL."""

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _cli import (
    catch_sigint,
    create_session_or_exit,
    read_finite_float,
    parse_session_args,
    print_no_robot_data_hint,
    stop_requested,
    wait_for_robot_sample,
    wants_help,
)

FIELD_NAMES = ("joint", "motor", "imu", "motion", "power", "diagnostics")


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       --duration SEC --output PATH [--rate HZ] [--format csv|jsonl]\n"
        "       [--fields joint,motor,imu,motion,power,diagnostics]\n"
        "  Default rate=10 format=csv fields=all.",
        file=sys.stderr,
    )


def join_values(values):
    return "|".join(str(value) for value in values)


def json_number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return (repr(value) if isinstance(value, float) else str(value)), False
    return "null", True


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    duration = 0.0
    rate = 10.0
    output = ""
    fmt = "csv"
    fields = {name: True for name in FIELD_NAMES}
    fields_set = False
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--duration" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "duration")
            if parsed is None:
                return 2
            duration = parsed
        elif arg == "--rate" and index + 1 < len(rest):
            index += 1
            parsed = read_finite_float(rest[index], "rate")
            if parsed is None:
                return 2
            rate = parsed
        elif arg == "--output" and index + 1 < len(rest):
            index += 1
            output = rest[index]
        elif arg == "--format" and index + 1 < len(rest):
            index += 1
            fmt = rest[index]
        elif arg == "--fields" and index + 1 < len(rest):
            index += 1
            fields = {name: False for name in FIELD_NAMES}
            fields_set = True
            for item in rest[index].split(","):
                if item not in fields:
                    print(f"unknown field {item}", file=sys.stderr)
                    return 2
                if item:
                    fields[item] = True
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if not fields_set:
        fields = {name: True for name in FIELD_NAMES}
    if duration <= 0.0 or rate <= 0.0 or not output or fmt not in ("csv", "jsonl"):
        print_usage(argv[0])
        return 2

    session = create_session_or_exit(options)
    if not wait_for_robot_sample(session):
        print_no_robot_data_hint()
        session.close()
        return 1
    try:
        handle = open(output, "w", encoding="utf-8")
    except OSError:
        print(f"cannot open {output}", file=sys.stderr)
        session.close()
        return 1
    if fmt == "csv":
        header = "timestamp_ms"
        if fields["joint"]:
            header += ",missing_joint,joint_since_receive_ms,joint_new,joint_n,joint_q"
        if fields["motor"]:
            header += ",missing_motor,motor_n"
        if fields["imu"]:
            header += ",missing_imu,roll,pitch,yaw"
        if fields["motion"]:
            header += ",missing_motion,motion"
        if fields["power"]:
            header += ",missing_power,power_pct"
        if fields["diagnostics"]:
            header += ",missing_diagnostics,motor_offline_n"
        handle.write(header + "\n")

    rows = 0
    invalid_numbers = 0
    write_failed = False
    missing = {name: 0 for name in FIELD_NAMES}
    have_power = False
    have_roll = False
    min_power = max_power = 0.0
    min_roll = max_roll = 0.0
    deadline = time.monotonic() + duration
    period = 1.0 / rate
    catch_sigint()
    prev_joint_time = object()
    have_prev = False
    with handle:
        while not stop_requested() and time.monotonic() < deadline:
            stamp = int(time.time() * 1000)
            joint_time = session.joint_state.last_receive_time() if fields["joint"] else None
            joint_new = joint_time is not None and (not have_prev or joint_time != prev_joint_time)
            if fields["joint"]:
                prev_joint_time = joint_time
                have_prev = True
            joint_since = int((time.monotonic() - joint_time) * 1000) if joint_time is not None else -1
            joint = session.joint_state.latest() if fields["joint"] else None
            motor = session.motor_state.latest() if fields["motor"] else None
            imu = session.imu.latest() if fields["imu"] else None
            motion = session.motion.current_motion() if fields["motion"] else None
            power = session.power.latest() if fields["power"] else None
            diag = session.motor_diagnostics.latest() if fields["diagnostics"] else None
            got_joint = joint is not None
            got_motor = motor is not None
            if fields["joint"] and not got_joint:
                missing["joint"] += 1
            if fields["motor"] and not got_motor:
                missing["motor"] += 1
            if fields["imu"] and imu is None:
                missing["imu"] += 1
            if fields["motion"] and motion is None:
                missing["motion"] += 1
            if fields["power"] and power is None:
                missing["power"] += 1
            if fields["diagnostics"] and diag is None:
                missing["diagnostics"] += 1
            if power is not None and math.isfinite(power.power_info.percentage):
                pct = power.power_info.percentage
                if not have_power or pct < min_power:
                    min_power = pct
                if not have_power or pct > max_power:
                    max_power = pct
                have_power = True
            if imu is not None and math.isfinite(imu.rpy.x):
                if not have_roll or imu.rpy.x < min_roll:
                    min_roll = imu.rpy.x
                if not have_roll or imu.rpy.x > max_roll:
                    max_roll = imu.rpy.x
                have_roll = True
            offline = 0
            if diag is not None:
                offline = sum(1 for flag in diag.offline if flag != 0)
            if fmt == "csv":
                cells = [str(stamp)]
                if fields["joint"]:
                    cells.extend(
                        [
                            "0" if got_joint else "1",
                            str(joint_since),
                            "1" if joint_new else "0",
                            str(len(joint.position) if got_joint else 0),
                            join_values(joint.position) if got_joint else "",
                        ]
                    )
                if fields["motor"]:
                    cells.extend(["0" if got_motor else "1", str(len(motor.position) if got_motor else 0)])
                if fields["imu"]:
                    cells.append("0" if imu is not None else "1")
                    if imu is not None:
                        cells.extend([str(imu.rpy.x), str(imu.rpy.y), str(imu.rpy.z)])
                    else:
                        cells.extend(["", "", ""])
                if fields["motion"]:
                    cells.extend(["0" if motion is not None else "1", motion or ""])
                if fields["power"]:
                    cells.extend(
                        [
                            "0" if power is not None else "1",
                            str(power.power_info.percentage if power is not None else 0.0),
                        ]
                    )
                if fields["diagnostics"]:
                    cells.extend(["0" if diag is not None else "1", str(offline)])
                handle.write(",".join(cells) + "\n")
            else:
                parts = [f'"timestamp_ms": {stamp}']
                if fields["joint"]:
                    parts.append(f'"missing_joint": {"false" if got_joint else "true"}')
                    parts.append(f'"joint_since_receive_ms": {joint_since}')
                    parts.append(f'"joint_new": {"true" if joint_new else "false"}')
                    parts.append(f'"joint_n": {len(joint.position) if got_joint else 0}')
                if fields["motor"]:
                    parts.append(f'"missing_motor": {"false" if got_motor else "true"}')
                    parts.append(f'"motor_n": {len(motor.position) if got_motor else 0}')
                if fields["imu"]:
                    parts.append(f'"missing_imu": {"false" if imu is not None else "true"}')
                    if imu is not None:
                        for axis, sample in (("roll", imu.rpy.x), ("pitch", imu.rpy.y), ("yaw", imu.rpy.z)):
                            text, bad = json_number(sample)
                            if bad:
                                invalid_numbers += 1
                            parts.append(f'"{axis}": {text}')
                if fields["motion"]:
                    parts.append(f'"missing_motion": {"false" if motion is not None else "true"}')
                    parts.append('"motion": ' + json.dumps(motion or ""))
                if fields["power"]:
                    pct = power.power_info.percentage if power is not None else 0.0
                    parts.append(f'"missing_power": {"false" if power is not None else "true"}')
                    text, bad = json_number(pct)
                    if bad:
                        invalid_numbers += 1
                    parts.append(f'"power_pct": {text}')
                if fields["diagnostics"]:
                    parts.append(f'"missing_diagnostics": {"false" if diag is not None else "true"}')
                    parts.append(f'"motor_offline_n": {offline}')
                handle.write("{" + ", ".join(parts) + "}\n")
            try:
                handle.flush()
            except OSError as ex:
                print(f"write failed saved={rows}: {ex}", file=sys.stderr)
                write_failed = True
                break
            rows += 1
            time.sleep(period)
    session.close()
    if write_failed:
        return 1
    print(f"output={output} format={fmt} rows={rows} invalid_numbers={invalid_numbers}")
    selected = [name for name in FIELD_NAMES if fields[name]]
    print("fields=" + ",".join(selected))
    print(
        "missing joint="
        + str(missing["joint"])
        + " motor="
        + str(missing["motor"])
        + " imu="
        + str(missing["imu"])
        + " motion="
        + str(missing["motion"])
        + " power="
        + str(missing["power"])
        + " diagnostics="
        + str(missing["diagnostics"])
    )
    if have_power:
        print(f"power_pct min={min_power} max={max_power}")
    if have_roll:
        print(f"imu_roll min={min_roll} max={max_roll}")
    return 0 if rows > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
