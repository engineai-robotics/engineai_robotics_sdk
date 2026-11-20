# EngineAI RobotSession SDK

[中文](README.zh.md) | English

Create a local session in C++ or Python and call the robot control and state interfaces. The client machine does not need ROS. ROS2 interface and examples are in [engineai_ros2_workspace](https://github.com/engineai-robotics/engineai_ros2_workspace). Python is the language binding for this C++ public API, and it ships in this repository together with the C++ examples.

- `include/engineai_sdk/`: public headers `robot.h`, `types.h`, and `version.h`. The current version is in `version.h`.
- `lib/<platform>/`: the session library, Python extension, `libddsc`, and iceoryx for the three platforms.
- `cpp/`: C++ examples and build scripts. Both languages share the YAML files in `cpp/examples/config/`.
- `python/`: the Python package and examples.

See [CHANGELOG.md](CHANGELOG.md) for changes. Python install and calls are in [python/README.md](python/README.md). Networking is in [docs/en/network.md](docs/en/network.md), calling rules are in [docs/en/usage.md](docs/en/usage.md), and every example command is in [docs/en/examples.md](docs/en/examples.md). `joint_bridge_stand_and_walk` is C++ only.

## Environment

Supported platforms are `ubuntu22.04-amd64`, `ubuntu22.04-arm64`, and `ubuntu24.04-arm64`.

```bash
sudo apt install -y cmake build-essential libyaml-cpp-dev libeigen3-dev
```

## Simulation and the real robot

Learn the SDK in the simulation repository first, then verify your own program there. Move it to the real robot only after that check passes. Take care on the real robot.

Simulation starts from [engineai_robot_simulation_runtime](https://github.com/engineai-robotics/engineai_robot_simulation_runtime). Run this repository's programs on the host. In the same terminal, before you run them:

```bash
export ROS_LOCALHOST_ONLY=1
```

A simulation command carries only `--domain-id`. Do not pass `--interface` or `--network-address`. Either flag rebinds DDS to another NIC, and the program will not find the motion control process inside the simulation.

On the real robot, connect first, copy this repository onto an onboard computer, then build and run it there. PM01 onboard computers are listed in the [hardware page](https://engineai.com.cn/open/docs/pm01/pm01/02?product=pm01&entry=docs):

| Unit | Model | Cores | IP | SSH user |
|---|---|---|---|---|
| Application computer | Jetson Orin NX | 8 | 192.168.0.162 | ubuntu |
| Motion-control module | Nezha | 4 | 192.168.0.163 | user |

The initial password is in the table on that page. When the program and the robot process run on the same machine, follow [On the same machine](docs/en/network.md#on-the-same-machine).

## Build

From the `cpp/` directory:

```bash
./build.sh
```

Executables are in `cpp/build/`. The build selects the libraries under the repository-root `lib/` for this machine. On a real robot, copy the repository onto the onboard computer as described above, then run this from `cpp/` on that machine.

## First run

`--domain-id` must equal the robot `ROS_DOMAIN_ID`. In simulation and on a running robot this value is usually `69`. Stay in the `cpp/` directory.

In simulation, run `export ROS_LOCALHOST_ONLY=1` first. Do not pass `--interface` or `--network-address`:

```bash
./build/basic_read_state --domain-id 69
```

Across machines on a real robot, also pass this machine's NIC name or subnet:

```bash
./build/basic_read_state --domain-id 69 --interface eth0
```

Printed joint and IMU samples mean robot data has arrived. On `(no sample)`, check networking in [docs/en/network.md](docs/en/network.md). Every example accepts `--help`.

Start C++ examples that read config files from `cpp/`, so they can find `examples/config`. Those paths are resolved from the current working directory. A program that does not read a config file can also be started from the repository root as `./cpp/build/<name>`. When a config-reading program is started from another directory, pass `--config`, `--actions`, `--bindings`, or `--config-dir` as a path relative to that directory. Every command is in [docs/en/examples.md](docs/en/examples.md).

## Optional: pin CPUs

To run a program on chosen CPUs, prefix the command with `taskset`. This only limits which CPUs the process may run on. It is not a required SDK step, and it does not make the program realtime or faster. Skip it when you are not placing work on specific CPUs. CPU numbers start at 0. On that board, run `nproc --all` first. Orin NX is 0–7. Nezha is 0–3. Build the C++ examples first. For Python, finish the setup in [python/README.md](python/README.md) first.

```bash
# cpp/, after ./build.sh
taskset -c 2 ./build/basic_read_state --domain-id 69
# python/, with the virtual environment active
taskset -c 2 python3 examples/basic_session.py --domain-id 69
```

`-c` takes one CPU, a comma-separated list, or a range, such as `2`, `2,3`, or `2-4`.

## Shortest use

The program below is complete. It waits for a heartbeat, sets the lamp to blinking green once the robot is online, checks the return value, waits 500 ms, then closes the session. `--domain-id` must be a complete non-negative integer. Leave `network_interface_name` unset in simulation. On a real robot across machines, set it to this machine's NIC name. Motion changes belong in `basic_motion`, not in this program.

```cpp
#include <charconv>
#include <chrono>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "engineai_sdk/robot.h"

int main(int argc, char** argv) {
  int domain_id = -1;
  if (argc == 3 && std::string(argv[1]) == "--domain-id") {
    const std::string text = argv[2];
    const char* first = text.data();
    const char* last = first + text.size();
    const auto parsed = std::from_chars(first, last, domain_id);
    if (parsed.ec != std::errc{} || parsed.ptr != last || domain_id < 0) {
      domain_id = -1;
    }
  }
  if (domain_id < 0) {
    std::cerr << "usage: " << argv[0] << " --domain-id N\n";
    return 2;
  }

  engineai_sdk::SessionOptions options;
  options.dds_domain_id = domain_id;

  auto result = engineai_sdk::Robot::create(options);
  if (!result) {
    return 1;
  }
  auto session = std::move(result.session);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool online = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto t = session->node_heartbeat().last_receive_time();
    if (t && std::chrono::steady_clock::now() - *t <= std::chrono::seconds(3)) {
      online = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!online) {
    session->close();
    return 1;
  }

  const auto err = session->led().set_mode(engineai_sdk::LedColor::kBlinkGreen);
  if (err != engineai_sdk::SessionError::kOk) {
    session->close();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  session->close();
  return 0;
}
```

A non-zero return means failure. `0` means success. A successful `create()` only means the local session exists. A successful command return only means the local call finished. It does not mean the robot has executed the command.

`latest()` is the most recent cache. After the robot stops publishing or the network drops, that cache can still be the old value, so a value by itself does not mean communication is healthy. Before sending a command that depends on robot state, read `last_receive_time()`: no value means nothing has arrived yet, and a long gap since the last receive means do not use that sample. Thresholds are in [docs/en/usage.md](docs/en/usage.md).

## Example index

Each row is only the purpose. The required motion, the operation, and how to judge success are in [docs/en/examples.md](docs/en/examples.md). `joint_bridge_stand_and_walk` is C++ only.

| Program | Purpose |
|---|---|
| [`basic_session`](docs/en/examples.md#basic-examples) | Create a local session |
| [`basic_read_state`](docs/en/examples.md#basic-examples) | Read joints, IMU, and other state |
| [`basic_motion`](docs/en/examples.md#basic-examples) | Query and switch the motion mode |
| [`basic_body_velocity`](docs/en/examples.md#basic-examples) | Send body velocity on a period |
| [`joint_plan_sequence`](docs/en/examples.md#planned-motions) | Run a planned motion from a config file |
| [`basic_joint_command`](docs/en/examples.md#basic-examples) | Send a joint command in joint passthrough |

## Add your own example

1. Put the source file under `cpp/examples/`, for example `cpp/examples/basic/my_program.cpp`. The shortest-use program can be saved at that kind of path.
2. Register it in `cpp/examples/CMakeLists.txt` with the existing `add_sdk_example`, for example `add_sdk_example(my_program basic/my_program.cpp)`.
3. From `cpp/`, run `./build.sh`.
4. Stay in `cpp/` and run `./build/my_program --domain-id 69`.

## Artifact integrity

[`ARTIFACTS.sha256`](ARTIFACTS.sha256) at the repository root records the
regular `.so` files and `.mnn` models distributed with this repository. After
checking out the repository, run:

```bash
sha256sum -c ARTIFACTS.sha256
```

The manifest identifies artifacts and detects content changes. It does not
replace license notices, provenance records, or a publisher signature.

## License

EngineAI-owned content is distributed under the
[BSD 3-Clause License](LICENSE.txt). Bundled third-party source and binaries
remain under their respective licenses. See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the complete texts under
[`licenses/`](licenses/).
