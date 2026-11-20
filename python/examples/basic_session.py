#!/usr/bin/env python3
"""basic_session: Robot.create / is_open / close.

create() only creates local DDS resources. It does not mean a robot was discovered.

  python3 examples/basic_session.py --domain-id 0 [--interface eth0]
"""

import sys

from _cli import create_session_or_exit, parse_session_args, wants_help


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
    print(f"is_open={int(session.is_open())}")
    print("Local session created. This is not a handshake with the robot.")
    print("Run basic_motion or basic_read_state to see if samples arrive.")
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
