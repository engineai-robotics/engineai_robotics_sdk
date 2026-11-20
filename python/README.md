# EngineAI RobotSession Python SDK

[中文](README.zh.md) | English

This is the language binding for the C++ RobotSession public API.

Class names, method names, and fields are in `src/engineai_sdkpy/__init__.pyi`. This file explains how to install and call it. Run every command from the `python/` directory.

The extensions, session library, `libddsc`, and iceoryx for the three platforms are in `lib/<platform>/` at the repository root. The production repository writes them there. The loader and examples stay in `python/`. The version matches the C++ SDK.

## Install

Run inside a virtual environment. Create it with the system `python3`: Python 3.10 on Ubuntu 22.04, Python 3.12 on Ubuntu 24.04. The extension is built for that version (`cpython-310` / `cpython-312`). conda, pyenv, or another minor version cannot load the matching `.so`.

From the `python/` directory:

```bash
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
python3 -c "import engineai_sdkpy; print(engineai_sdkpy.__version__)"
```

After `source .venv/bin/activate`, `python3` in this terminal prefers `.venv/bin/python3`. The package is installed into that virtual environment's interpreter. A new terminal needs `source .venv/bin/activate` again.

Without an environment, the system `python3` can run it directly:

```bash
PYTHONPATH=src python3 -c "import engineai_sdkpy; print(engineai_sdkpy.__version__)"
```

`pip install .` without `-e` does not put `lib/` into site-packages. The loader looks for `lib/` from the repository root, so use the editable install above, or `PYTHONPATH=src`.

The platform comes from `ENGINEAI_ROBOT_SESSION_PLATFORM`. When it is unset, the loader reads `/etc/os-release` and the CPU. `import` fails outside `ubuntu22.04-amd64`, `ubuntu22.04-arm64`, and `ubuntu24.04-arm64`.

Each `lib/<platform>/` contains everything that platform needs to run: the Python extension, `libEngineAIRobotSession.so`, `libddsc.so.0.10.5`, and the four iceoryx libraries. `LD_LIBRARY_PATH` is not required. The extension and the session library both use `$ORIGIN` as `RUNPATH`.

## Use

`--domain-id` must equal the robot `ROS_DOMAIN_ID`. It is `0` when unset. On a running robot it is usually `69`. Networking is in [docs/en/network.md](../docs/en/network.md).

Run the following in the activated virtual environment. If the editable install was not done, prefix the command with `PYTHONPATH=src`.

In simulation, run `export ROS_LOCALHOST_ONLY=1` in the same terminal first. Do not pass `--interface` or `--network-address`:

```bash
python3 examples/basic_session.py --domain-id 69
```

Across machines on a real robot, also pass this machine's NIC name:

```bash
python3 examples/basic_session.py --domain-id 69 --interface eth0
```

The program below does the same thing as the C++ shortest use: wait for a heartbeat, set the lamp to blinking green once the robot is online, check the return value, wait 0.5 seconds, then close. `--domain-id` must be a complete non-negative integer. Leave `network_interface_name` unset in simulation. On a real robot across machines, set it to this machine's NIC name. `try`/`finally` still closes the session when the wait times out or an exception leaves the block. Motion changes belong in `basic_motion.py`, not in this program.

```python
import sys
import time

from engineai_sdkpy import LedColor, Robot, SessionError, SessionOptions


def main(argv: list[str]) -> int:
    domain_id = -1
    if len(argv) == 3 and argv[1] == "--domain-id" and argv[2].isascii() and argv[2].isdigit():
        domain_id = int(argv[2])
    if domain_id < 0:
        print(f"usage: {argv[0]} --domain-id N", file=sys.stderr)
        return 2

    options = SessionOptions()
    options.dds_domain_id = domain_id

    created = Robot.create(options)
    if not created or created.session is None:
        return int(created.error)

    session = created.session
    try:
        deadline = time.monotonic() + 5.0
        online = False
        while time.monotonic() < deadline:
            t = session.node_heartbeat.last_receive_time()
            if t is not None and time.monotonic() - t <= 3.0:
                online = True
                break
            time.sleep(0.02)
        if not online:
            return 1

        err = session.led.set_mode(LedColor.kBlinkGreen)
        if err != SessionError.kOk:
            return 1
        time.sleep(0.5)
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
```

`create()` only creates local DDS resources. It is not a handshake with the robot. `is_open()` only means the local resources are still held. `kOk` means the call in this process finished. It does not mean the robot has executed it. Velocity must be refreshed faster than 500 ms. At the end of an override, send `weight=0` yourself, and do it before `close()`. `close()` does not send it for you.

Leaving a `with session:` block calls `close()`. Writes after `close()` return `kSessionClosed`. Whether a call should be sent, and whether a sample is fresh, is in [docs/en/usage.md](../docs/en/usage.md).

### Whether the robot state is still updating

Before sending a command that depends on robot state, check that the state is still updating. `latest()` is the most recent cache. After the robot stops publishing or the network drops, that cache can still be the old value, so a value by itself does not mean communication is healthy.

`last_receive_time()` is seconds on the same clock as `time.monotonic()`. `None` means nothing has arrived yet. If the time since the last receive is too long, treat the sample as stale and do not send a command from it. `joint_command` uses `last_feedback_time()`. Read the time first, then the value. Thresholds are in [docs/en/usage.md](../docs/en/usage.md). The example code is `is_fresh` and `robot_online` in `examples/_cli.py`.

```python
t = session.joint_state.last_receive_time()
if t is None or time.monotonic() - t > 0.1:
    ...  # stale: stop sending commands
joint = session.joint_state.latest()
```

### Threads

Every method can be called from any thread, including `close()` while other threads are reading or writing. Those calls then return `None` or `kSessionClosed`. The GIL is released during send, read, `create()`, and `close()`, so a blocking DDS call does not stall other Python threads.

## Examples

Every command is in [docs/en/examples.md](../docs/en/examples.md). Scenario programs use the same names and parameters as C++. There is no `joint_bridge_stand_and_walk`. With no path given, they read `cpp/examples/config` in the repository. Programs that read YAML also need PyYAML (`python3-yaml`). That is not an install dependency of this package.
