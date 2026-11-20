#!/usr/bin/env python3
"""basic_led: led.set_mode.

  python3 examples/basic_led.py --domain-id 0
  python3 examples/basic_led.py --domain-id 0 --color 2
"""

import sys

from engineai_sdkpy import LedColor

from _cli import (
    check_ok,
    create_session_or_exit,
    parse_session_args,
    read_strict_int,
    print_no_robot_data_hint,
    wait_for_robot_sample,
    wants_help,
)


def main(argv) -> int:
    usage = f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR] [--color N]"
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print(usage, file=sys.stderr)
        return 2
    color = int(LedColor.kBlinkGreen)
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--color" and i + 1 < len(rest):
            parsed = read_strict_int(rest[i + 1], "color")
            if parsed is None:
                return 2
            color = parsed
            i += 2
        else:
            print(usage, file=sys.stderr)
            return 2
    session = create_session_or_exit(options)
    if not wait_for_robot_sample(session):
        print_no_robot_data_hint()
        session.close()
        return 1
    err = session.led.set_mode(LedColor(color))
    if not check_ok(err, "led.set_mode"):
        session.close()
        return 1
    print(f"led.set_mode({color}) ok")
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
