# EngineAI RobotSession SDK

中文 | [English](README.md)

用 C++ 或 Python 创建本地会话，调用机器人的控制与状态接口。客户机不需要安装 ROS。ROS2 接口和示例见 [engineai_ros2_workspace](https://github.com/engineai-robotics/engineai_ros2_workspace)。Python 是这套 C++ 公共接口的语言绑定，与 C++ 示例放在同一仓库。

- `include/engineai_sdk/`：公开头文件 `robot.h`、`types.h`、`version.h`。当前版本见 `version.h`。
- `lib/<platform>/`：三个平台的会话库、Python 扩展、`libddsc` 和 iceoryx。
- `cpp/`：C++ 示例和构建脚本。两套示例共用的 YAML 在 `cpp/examples/config/`。
- `python/`：Python 包和示例。

变更记录见 [CHANGELOG.md](CHANGELOG.md)。Python 的安装和调用见 [python/README.zh.md](python/README.zh.md)。组网见 [docs/zh/network.md](docs/zh/network.md)，调用约定见 [docs/zh/usage.md](docs/zh/usage.md)，全部示例命令见 [docs/zh/examples.md](docs/zh/examples.md)。`joint_bridge_stand_and_walk` 只有 C++。

## 环境

支持 `ubuntu22.04-amd64`、`ubuntu22.04-arm64`、`ubuntu24.04-arm64`。

```bash
sudo apt install -y cmake build-essential libyaml-cpp-dev libeigen3-dev
```

## 仿真与真机

建议先在仿真仓库里学习 SDK 接口的用法，再在仿真里验证自己的程序，确认无误后再移植到真机。真机验证时注意安全。

仿真从 [engineai_robot_simulation_runtime](https://github.com/engineai-robotics/engineai_robot_simulation_runtime) 进入。程序在宿主机运行。执行前在同一个终端执行：

```bash
export ROS_LOCALHOST_ONLY=1
```

仿真命令只带 `--domain-id`。不要传 `--interface` 或 `--network-address`，传了会改绑到别的网卡，发现不到仿真里的运动控制。

真机验证时，先连上机器人，把本仓库拷到机载计算机上，再在那台机器上编译和运行。PM01 的机载计算机见 [硬件信息](https://engineai.com.cn/open/docs/pm01/pm01/02?product=pm01&entry=docs)：

| 单元 | 型号 | 核数 | IP | SSH 用户 |
|---|---|---|---|---|
| 应用计算单元 | Jetson Orin NX | 8 | 192.168.0.162 | ubuntu |
| 运控计算模组 | Nezha | 4 | 192.168.0.163 | user |

初始密码以该页表格为准。程序和机器人进程在同一台机器上时，按 [docs/zh/network.md](docs/zh/network.md) 的「同一台机器」设置。

## 构建

在 `cpp/` 目录执行：

```bash
./build.sh
```

可执行文件在 `cpp/build/`。本机平台会自动选择仓库根 `lib/` 下的库。真机先按上一节把仓库拷到机载计算机，再在那台机器的 `cpp/` 里执行。

## 第一次运行

`--domain-id` 必须等于机器人的 `ROS_DOMAIN_ID`。仿真和机器人运行时，这个值一般是 `69`。仍在 `cpp/` 目录执行。

仿真时先 `export ROS_LOCALHOST_ONLY=1`，不要传 `--interface` 或 `--network-address`：

```bash
./build/basic_read_state --domain-id 69
```

真机跨机器时，再传本机网卡名或网段：

```bash
./build/basic_read_state --domain-id 69 --interface eth0
```

能打印关节、IMU 等样本即表示已经收到机器人数据。输出为 `(no sample)` 时，按 [docs/zh/network.md](docs/zh/network.md) 检查组网。每个示例都支持 `--help`。

读配置文件的 C++ 示例要从 `cpp/` 启动，这样才找得到 `examples/config`。这些路径按当前工作目录解析。不读配置的程序也可以在仓库根用 `./cpp/build/<name>` 启动。换目录启动读配置的程序时，把 `--config`、`--actions`、`--bindings` 或 `--config-dir` 改成相对新工作目录的路径。完整命令见 [docs/zh/examples.md](docs/zh/examples.md)。

## 可选：绑定 CPU

要让程序只在指定 CPU 上运行，在启动命令前加 `taskset`。它只限制进程可以跑在哪些核上，不是 SDK 的必做步骤，也不保证实时性或更快。没有做调度优化时可以跳过。编号从 0 开始。先在这块板上执行 `nproc --all` 确认核数：Orin NX 是 0–7，Nezha 是 0–3。C++ 要先按「构建」编译完成。Python 要先按 [python/README.zh.md](python/README.zh.md) 配好环境。

```bash
# cpp/，已经执行过 ./build.sh
taskset -c 2 ./build/basic_read_state --domain-id 69
# python/，虚拟环境已激活
taskset -c 2 python3 examples/basic_session.py --domain-id 69
```

`-c` 可以写一个核、逗号列表或范围，例如 `2`、`2,3`、`2-4`。

## 最短用法

下面是一份完整程序：等待心跳，确认在线后设置一次绿灯，检查返回值，等待 500 ms 再关闭会话。`--domain-id` 必须是完整的非负整数。仿真不要设置 `network_interface_name`。真机跨机器时再把它设为本机网卡名。模式切换见 `basic_motion`，不在这段里做。

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

失败返回非 0，成功返回 0。`create()` 成功只表示本机会话已建立。命令返回成功只表示本机调用完成，不表示机器人已经执行。

`latest()` 读到的是最近一次缓存。机器人停止发布或网络断开后，仍可能是旧值，不能只凭读到数据判断通信正常。发送依赖机器人状态的命令前，先看 `last_receive_time()`：没有值表示尚未收到；距上次收到的时间过长就不要用这份数据。阈值见 [docs/zh/usage.md](docs/zh/usage.md)。

## 示例索引

每行只有用途。所需运动、操作和成功判断见 [docs/zh/examples.md](docs/zh/examples.md)。`joint_bridge_stand_and_walk` 只有 C++。

| 程序 | 作用 |
|---|---|
| [`basic_session`](docs/zh/examples.md#基础案例) | 创建本机会话 |
| [`basic_read_state`](docs/zh/examples.md#基础案例) | 读取关节、IMU 等状态 |
| [`basic_motion`](docs/zh/examples.md#基础案例) | 查询并切换运动模式 |
| [`basic_body_velocity`](docs/zh/examples.md#基础案例) | 按周期发送身体速度 |
| [`joint_plan_sequence`](docs/zh/examples.md#规划动作) | 按配置执行规划动作 |
| [`basic_joint_command`](docs/zh/examples.md#基础案例) | 在关节透传下发送关节命令 |

## 把自己的程序加进示例工程

1. 源文件放在 `cpp/examples/`，例如 `cpp/examples/basic/my_program.cpp`。「最短用法」里的程序可以存成这个路径。
2. 在 `cpp/examples/CMakeLists.txt` 里用已有的 `add_sdk_example` 登记，例如 `add_sdk_example(my_program basic/my_program.cpp)`。
3. 在 `cpp/` 执行 `./build.sh`。
4. 仍在 `cpp/` 运行 `./build/my_program --domain-id 69`。

## 制品完整性

仓库根目录的 [`ARTIFACTS.sha256`](ARTIFACTS.sha256) 记录随仓交付的 `.so` 普通
文件和 `.mnn` 模型。检出仓库后可以执行：

```bash
sha256sum -c ARTIFACTS.sha256
```

该清单用于识别制品并检查内容是否变化，不替代许可证、来源证明或发布者签名。

## 许可证

EngineAI 自有内容按 [BSD 3-Clause License](LICENSE.txt) 分发。随仓提供的第三方
源码和二进制继续适用其各自许可证，详见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 和
[`licenses/`](licenses/) 下的完整文本。
