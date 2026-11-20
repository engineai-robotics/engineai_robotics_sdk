"""Shared argument parsing for the examples."""

import math
import os
import signal
import sys
import termios
import time

from engineai_sdkpy import Robot, SessionError, SessionOptions


def err_name(err) -> str:
    names = {
        SessionError.kOk: "Ok",
        SessionError.kSessionClosed: "SessionClosed",
        SessionError.kTransportUnavailable: "TransportUnavailable",
        SessionError.kPublishFailed: "PublishFailed",
        SessionError.kInvalidArgument: "InvalidArgument",
    }
    return names.get(err, "Unknown")


def wants_help(argv) -> bool:
    return any(arg in ("-h", "--help") for arg in argv[1:])


def parse_strict_int(text: str) -> int:
    """Accept only a complete base-10 integer token. '12abc' and 'abc' fail."""
    if isinstance(text, bool) or not isinstance(text, str) or not text:
        raise ValueError("not an integer")
    body = text[1:] if text[0] == "-" else text
    if not body or not body.isascii() or not body.isdigit():
        raise ValueError("not an integer")
    return int(text)


def parse_finite_float(text: str) -> float:
    """Accept only a complete finite number. 'nan', 'inf', and '1abc' fail."""
    if isinstance(text, bool) or not isinstance(text, str) or not text or text.strip() != text:
        raise ValueError("not a finite number")
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError("not a finite number") from exc
    if not math.isfinite(value):
        raise ValueError("not a finite number")
    return value


def ensure_finite_number(value) -> float:
    """YAML numbers must already be ints or floats, and must be finite."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("expected a finite number")
    return number


def ensure_int(value) -> int:
    """YAML integers must be real ints. 0.9 is not truncated."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("expected an integer")
    return value


def read_strict_int(text, name):
    try:
        return parse_strict_int(text)
    except ValueError:
        print(f"{name} must be an integer", file=sys.stderr)
        return None


def read_finite_float(text, name):
    try:
        return parse_finite_float(text)
    except ValueError:
        print(f"{name} must be a finite number", file=sys.stderr)
        return None


def parse_yaml_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.lower() in ("true", "false"):
        return value.lower() == "true"
    raise ValueError("expected true or false")


def parse_session_args(argv):
    options = SessionOptions()
    rest = []
    have_domain = False
    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == "--domain-id" and i + 1 < len(argv):
            domain_id = read_strict_int(argv[i + 1], "domain-id")
            if domain_id is None:
                return None, rest
            options.dds_domain_id = domain_id
            have_domain = True
            i += 2
        elif arg == "--interface" and i + 1 < len(argv):
            options.network_interface_name = argv[i + 1]
            i += 2
        elif arg == "--network-address" and i + 1 < len(argv):
            options.network_address = argv[i + 1]
            i += 2
        else:
            rest.append(arg)
            i += 1
    if not have_domain:
        print(
            "Missing required --domain-id <int> (robot ROS_DOMAIN_ID; 0 if unset)",
            file=sys.stderr,
        )
        return None, rest
    if options.dds_domain_id < 0:
        print("InvalidArgument: dds_domain_id must be >= 0", file=sys.stderr)
        return None, rest
    return options, rest


def create_session_or_exit(options) -> Robot:
    created = Robot.create(options)
    if not created:
        print(f"local session creation failed: {err_name(created.error)}", file=sys.stderr)
        raise SystemExit(1)
    return created.session


def check_ok(err, what) -> bool:
    if err != SessionError.kOk:
        print(f"{what} -> {err_name(err)}", file=sys.stderr)
        return False
    return True


# latest() keeps returning the last sample after the robot stops publishing.
# Freshness comes from last_receive_time(). Thresholds follow the robot publish
# periods: heartbeat 1 s, motion_state 200 ms, joint/imu 2 ms.
HEARTBEAT_MAX_AGE = 3.0
MOTION_MAX_AGE = 1.0
STATE_MAX_AGE = 0.1


# Read the time before latest(): the value read afterwards is at least as new,
# so the age can only be overestimated.
def is_fresh(received, max_age) -> bool:
    return received is not None and time.monotonic() - received <= max_age


def age_text(received) -> str:
    if received is None:
        return "(no sample)"
    return f"{(time.monotonic() - received) * 1000:.0f} ms"


# The heartbeat is the robot liveness signal.
def robot_online(session) -> bool:
    return is_fresh(session.node_heartbeat.last_receive_time(), HEARTBEAT_MAX_AGE)


def motion_or_none(session) -> str:
    current = session.motion.current_motion()
    return current if current is not None else "(no sample)"


def print_no_robot_data_hint() -> None:
    print(
        "No sample from the robot yet. A successfully created local session "
        "does not mean that a robot was found.\n"
        "Check: robot is running, and --domain-id equals its ROS_DOMAIN_ID "
        "(0 when unset; run_robot.sh exports 69). On the same machine with "
        "ROS_LOCALHOST_ONLY=1, export that variable and do not pass "
        "--interface or --network-address. Otherwise select this PC's NIC "
        "with --interface or --network-address (ip -br addr).",
        file=sys.stderr,
    )


# A stale motion sample does not count: the robot may have left that motion.
def motion_is(session, name) -> bool:
    if not is_fresh(session.motion.last_receive_time(), MOTION_MAX_AGE):
        return False
    return session.motion.current_motion() == name


def wait_for_robot_sample(session, timeout_ms=5000) -> bool:
    deadline = time.monotonic() + timeout_ms / 1000.0
    while time.monotonic() < deadline:
        if not session.is_open():
            return False
        if robot_online(session) or is_fresh(session.motion.last_receive_time(), MOTION_MAX_AGE):
            return True
        time.sleep(0.02)
    return False


def pause_before_close(pause_ms=500) -> None:
    time.sleep(pause_ms / 1000.0)


def wait_for_motion_sample(session, timeout_ms=5000) -> bool:
    deadline = time.monotonic() + timeout_ms / 1000.0
    while time.monotonic() < deadline:
        if is_fresh(session.motion.last_receive_time(), MOTION_MAX_AGE) and session.motion.current_motion() is not None:
            return True
        time.sleep(0.02)
    return False


def require_current_motion(session, name) -> bool:
    if not wait_for_motion_sample(session):
        print_no_robot_data_hint()
        return False
    if not motion_is(session, name):
        print(f"current motion={motion_or_none(session)}, required={name}", file=sys.stderr)
        return False
    return True


def wait_joint_state(session, timeout_ms=5000):
    deadline = time.monotonic() + timeout_ms / 1000.0
    while time.monotonic() < deadline:
        fresh = is_fresh(session.joint_state.last_receive_time(), STATE_MAX_AGE)
        joint = session.joint_state.latest()
        if fresh and joint is not None and joint.position:
            return joint
        time.sleep(0.02)
    return None


_stop = False


def stop_requested() -> bool:
    return _stop


def request_stop() -> None:
    global _stop
    _stop = True


def catch_sigint() -> None:
    global _stop
    _stop = False

    def _handle(_signum, _frame):
        global _stop
        _stop = True

    signal.signal(signal.SIGINT, _handle)


class TerminalRaw:
    """Non-canonical stdin. No-op when stdin is not a terminal."""

    def __init__(self, enable=True):
        self.active = False
        self._fd = None
        self._saved = None
        if not enable or not sys.stdin.isatty():
            return
        self._fd = sys.stdin.fileno()
        try:
            self._saved = termios.tcgetattr(self._fd)
            raw = termios.tcgetattr(self._fd)
            raw[3] = raw[3] & ~(termios.ICANON | termios.ECHO)
            raw[6][termios.VMIN] = 0
            raw[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, raw)
        except termios.error:
            self._saved = None
            return
        self.active = True

    def close(self) -> None:
        if self.active and self._saved is not None and self._fd is not None:
            termios.tcsetattr(self._fd, termios.TCSANOW, self._saved)
            self.active = False

    def read_chars(self):
        if not self.active or self._fd is None:
            return
        while True:
            try:
                data = os.read(self._fd, 1)
            except OSError:
                return
            if not data:
                return
            yield data.decode("latin1")
