#!/usr/bin/env python3
"""led_control: set lamp patterns by name or value."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engineai_sdkpy import LedColor, SessionError

from _cli import (
    catch_sigint,
    create_session_or_exit,
    err_name,
    parse_session_args,
    read_strict_int,
    pause_before_close,
    print_no_robot_data_hint,
    stop_requested,
    wait_for_robot_sample,
    wants_help,
)

MODES = (
    ("blink_red", LedColor.kBlinkRed, "blink red"),
    ("blink_green", LedColor.kBlinkGreen, "blink green"),
    ("blink_blue", LedColor.kBlinkBlue, "blink blue"),
    ("blink_white", LedColor.kBlinkWhite, "blink white"),
    ("constant_on_white", LedColor.kConstantOnWhite, "constant white"),
    ("constant_on_green", LedColor.kConstantOnGreen, "constant green"),
    ("breathe_white", LedColor.kBreatheWhite, "breathe white"),
    ("water_white", LedColor.kWaterWhite, "water white"),
    ("breathe_red", LedColor.kBreatheRed, "breathe red"),
    ("blink_orange", LedColor.kBlinkOrange, "blink orange"),
    ("constant_on_orange", LedColor.kConstantOnOrange, "constant orange"),
    ("blink_red_and_white", LedColor.kBlinkRedAndWhite, "blink red and white"),
    ("constant_on_red", LedColor.kConstantOnRed, "constant red"),
)


def print_usage(argv0):
    print(
        f"Usage: {argv0} --domain-id N [--interface IF] [--network-address ADDR]\n"
        "       [--list] [--color NAME|N] [--sequence NAME,NAME|all] [--cycles N] [--period-ms MS]\n"
        "  --sequence all or --cycles plays every lamp mode. --cycles 0 runs until Ctrl+C.\n"
        "  Sets a lamp request. The SDK cannot read the current lamp.\n"
        "  Fault and safety indications on the robot may hide the request.",
        file=sys.stderr,
    )


def print_modes():
    for name, color, meaning in MODES:
        print(f"{name} value={int(color)} {meaning}")


def parse_color(text):
    for name, color, _meaning in MODES:
        if text == name:
            return color
    try:
        value = int(text, 10)
    except ValueError:
        return None
    for _name, color, _meaning in MODES:
        if int(color) == value:
            return color
    return None


def color_name(color):
    for name, mode, _meaning in MODES:
        if mode == color:
            return name
    return "unknown"


def parse_sequence(text):
    if text == "all":
        return [color for _name, color, _meaning in MODES]
    out = []
    for token in text.replace(" ", "").split(","):
        if not token:
            return None
        color = parse_color(token)
        if color is None:
            return None
        out.append(color)
    return out or None


def main(argv) -> int:
    if wants_help(argv):
        print_usage(argv[0])
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print_usage(argv[0])
        return 2
    list_modes = False
    color_text = ""
    sequence_text = ""
    cycles = 1
    cycles_set = False
    period_ms = 1000
    index = 0
    while index < len(rest):
        arg = rest[index]
        if arg == "--list":
            list_modes = True
        elif arg == "--color" and index + 1 < len(rest):
            index += 1
            color_text = rest[index]
        elif arg == "--sequence" and index + 1 < len(rest):
            index += 1
            sequence_text = rest[index]
        elif arg == "--cycles" and index + 1 < len(rest):
            index += 1
            parsed = read_strict_int(rest[index], "cycles")
            if parsed is None:
                return 2
            cycles = parsed
            cycles_set = True
        elif arg == "--period-ms" and index + 1 < len(rest):
            index += 1
            parsed = read_strict_int(rest[index], "period-ms")
            if parsed is None:
                return 2
            period_ms = parsed
        else:
            print_usage(argv[0])
            return 2
        index += 1
    if list_modes:
        print_modes()
    if not color_text and not sequence_text:
        if list_modes and not cycles_set:
            return 0
        if not cycles_set:
            print("pass --list, --color, --sequence, or --cycles", file=sys.stderr)
            print_usage(argv[0])
            return 2
        sequence_text = "all"
    if cycles < 0 or period_ms <= 0:
        print("period-ms must be > 0", file=sys.stderr)
        return 2

    modes = []
    if color_text:
        color = parse_color(color_text)
        if color is None:
            print(f"unknown color '{color_text}'", file=sys.stderr)
            print_modes()
            return 2
        modes.append(color)
    if sequence_text:
        sequence = parse_sequence(sequence_text)
        if sequence is None:
            print(f"unknown sequence '{sequence_text}'", file=sys.stderr)
            print_modes()
            return 2
        modes.extend(sequence)

    session = create_session_or_exit(options)
    if not wait_for_robot_sample(session):
        print_no_robot_data_hint()
        session.close()
        return 1
    print(
        "note: set_mode reports only that this process sent the request. "
        "There is no API to read the current lamp. "
        "Fault and safety indications may hide this request until they clear."
    )
    catch_sigint()
    cycle = 0
    while not stop_requested() and (cycles == 0 or cycle < cycles):
        for mode in modes:
            if stop_requested():
                break
            err = session.led.set_mode(mode)
            print(f"led.set_mode name={color_name(mode)} value={int(mode)} -> {err_name(err)}")
            if err != SessionError.kOk:
                session.close()
                return 1
            if len(modes) > 1 or cycles != 1:
                time.sleep(period_ms / 1000.0)
        cycle += 1
    pause_before_close()
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
