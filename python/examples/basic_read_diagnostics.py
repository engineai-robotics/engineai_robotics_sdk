#!/usr/bin/env python3
"""basic_read_diagnostics: power / motor_diagnostics / node_heartbeat / motor_command.

  python3 examples/basic_read_diagnostics.py --domain-id 0 [--interface eth0]
"""

import sys
import time

from _cli import create_session_or_exit, parse_session_args, print_no_robot_data_hint, wants_help


def main(argv) -> int:
    usage = f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR]"
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None or rest:
        print(usage, file=sys.stderr)
        return 2
    session = create_session_or_exit(options)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if (
            session.power.latest() is not None
            or session.motor_diagnostics.latest() is not None
            or session.node_heartbeat.latest() is not None
            or session.motor_command.latest() is not None
        ):
            break
        time.sleep(0.02)

    power = session.power.latest()
    if power is not None:
        board = power.power_info
        line = (
            f"power enable={int(board.enable)} pct={board.percentage} V={board.voltage} "
            f"A={board.current} err={board.error_code}"
        )
        if power.battery_info is not None:
            battery = power.battery_info
            line += (
                f" bms_soc={int(battery.soc)} bms_temp={battery.temp} "
                f"bms_current_ma={battery.bms_current}"
            )
        else:
            line += " bms=(unavailable)"
        print(line)
    else:
        print("power (no sample)")

    dbg = session.motor_diagnostics.latest()
    if dbg is not None:
        print(
            f"motor_diagnostics n_temp={len(dbg.motor_temperature)} n_offline={len(dbg.offline)}"
        )
    else:
        print("motor_diagnostics (no sample)")

    hb = session.node_heartbeat.latest()
    if hb is not None:
        print(f"node_heartbeat node={hb.node_name} status={hb.node_status} err={hb.error_message}")
    else:
        print("node_heartbeat (no sample)")

    motor_cmd = session.motor_command.latest()
    if motor_cmd is not None:
        line = f"motor_command n_pos={len(motor_cmd.position)}"
        if motor_cmd.position:
            line += f" q0={motor_cmd.position[0]}"
        print(line)
    else:
        print("motor_command (no sample)")

    if (
        session.power.latest() is None
        and session.motor_diagnostics.latest() is None
        and session.node_heartbeat.latest() is None
        and session.motor_command.latest() is None
    ):
        print_no_robot_data_hint()
        session.close()
        return 1
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
