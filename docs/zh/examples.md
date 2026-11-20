# 示例

中文 | [English](../en/examples.md)

C++ 在 `cpp/` 目录构建和运行。下文 C++ 命令里的 `./build/` 和 `examples/config` 都相对 `cpp/`。Python 在 `python/` 目录运行，命令是 `python3 examples/...`。Python 场景的默认配置目录是仓库里的 `cpp/examples/config`；命令里单独写出的 YAML 路径相对 `python/`。

`joint_bridge_stand_and_walk` 只有 C++。其余示例两种语言都有，参数相同。

C++ 的命令行解析、等待样本和 Ctrl+C 在 `cpp/examples/support/`，这些头文件不是公开接口。

需要特定运动的程序不会自行切换。先用 `motion_mode_manager` 切到目标运动。

## 基础案例

C++ 源码在 `cpp/examples/basic/`，Python 源码是 `python/examples/basic_*.py`。每个程序对应一组接口。

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `basic_session` | 创建本机会话。`is_open` 只表示会话还在 | `./build/basic_session --domain-id 69 --interface eth0` | `python3 examples/basic_session.py --domain-id 69 --interface eth0` |
| `basic_read_state` | 读关节、电机、IMU、手柄，并打印距上次收到的时长 `since_receive` | `./build/basic_read_state --domain-id 69` | `python3 examples/basic_read_state.py --domain-id 69` |
| `basic_read_diagnostics` | 读电源、电机诊断、心跳、电机命令 | `./build/basic_read_diagnostics --domain-id 69` | `python3 examples/basic_read_diagnostics.py --domain-id 69` |
| `basic_motion` | 列出当前运动和可切换列表。目标不在列表中则不发送，发送后轮询到切换完成。示例命令会切入关节透传：注意安全。该模式下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地 | `./build/basic_motion --domain-id 69 --motion joint_bridge` | `python3 examples/basic_motion.py --domain-id 69 --motion joint_bridge` |
| `basic_joint_command` | 注意安全。关节透传下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地。C++ 在 `joint_bridge` 下小幅移动一个关节，按周期续发，并用实测关节确认。Python 重复发送启动时测到的关节位置，命令回显不能说明关节已经移动。`--seconds 0` 直到 Ctrl+C | `./build/basic_joint_command --domain-id 69` | `python3 examples/basic_joint_command.py --domain-id 69` |
| `basic_joint_motion_planner` | 查询规划器。`--reset` 仅在 `lower_body_balance` 下发送，并轮询到完成 | `./build/basic_joint_motion_planner --domain-id 69 --reset` | `python3 examples/basic_joint_motion_planner.py --domain-id 69 --reset` |
| `basic_body_velocity` | 按周期发身体速度，退出时发 0。没有执行回读。`--vx`、`--vy`、`--yaw` 对应 `linear_velocity[0]`、`linear_velocity[1]`、`yaw_velocity` | `./build/basic_body_velocity --domain-id 69 --vx 0.5 --seconds 3` | `python3 examples/basic_body_velocity.py --domain-id 69 --vx 0.5 --seconds 3` |
| `basic_joint_override` | 以 10 Hz 刷新关节 override，结束时发 weight 0。`close()` 不会释放 override | `./build/basic_joint_override --domain-id 69 --seconds 1` | `python3 examples/basic_joint_override.py --domain-id 69 --seconds 1` |
| `basic_led` | 设置一次灯效。没有回读 | `./build/basic_led --domain-id 69` | `python3 examples/basic_led.py --domain-id 69` |

只查询当前运动、不切换：

```bash
# cpp/
./build/basic_motion --domain-id 69
# python/
python3 examples/basic_motion.py --domain-id 69
```

## 场景例程

C++ 源码在 `cpp/examples/scenarios/`，Python 源码在 `python/examples/scenarios/`。YAML 只保留一份，在 `cpp/examples/config/pm01/`、`t800/`、`t800pro/`。读 YAML 的 Python 程序需要另外安装 PyYAML（`python3-yaml`），它不是 SDK 安装依赖。

### 接入与状态

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `connectivity_doctor` | 检查会话，以及心跳、运动、关节、IMU、电源是否到达。可加 `--json` | `./build/connectivity_doctor --domain-id 69 --timeout 5` | `python3 examples/scenarios/connectivity_doctor.py --domain-id 69 --timeout 5` |
| `robot_status_snapshot` | 一屏状态。断流后仍显示上次状态，并标成过期。`--watch` 连续刷新，`--json` 输出固定字段 | `./build/robot_status_snapshot --domain-id 69 --json` | `python3 examples/scenarios/robot_status_snapshot.py --domain-id 69 --json` |

### 运动模式

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `motion_mode_manager` | 列出可切换运动。目标不在列表中则不发送。`--json` 供脚本读取最终结果。示例命令会切入关节透传：注意安全。该模式下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地 | `./build/motion_mode_manager --domain-id 69 --target-motion joint_bridge` | `python3 examples/scenarios/motion_mode_manager.py --domain-id 69 --target-motion joint_bridge` |

### 行走

速度例程不把运动名固定为 `lower_body_balance`。当前运动接受身体速度即可。`--x` / `--vx`、`--y` / `--vy`、`--yaw` / `--yaw-rate` 分别对应机体速度接口的 `linear_velocity[0]`、`linear_velocity[1]`、`yaw_velocity`。字段、单位和限制见[机体速度控制](https://engineai.com.cn/open/docs/t800/ros2-interface/04)。

下表是 PM01 前向行进的建议最小 `linear_velocity[0]`。侧向速度和偏航角速度设为 0，以约 100 Hz 持续发送。这些是实测建议，程序不拒绝更小的值。T800 可从较小值开始，确认能连续迈步后再加大。

| 步态 | 建议最小前向指令 `linear_velocity[0]` |
|---|---|
| 下肢平衡 `lower_body_balance` | 0.5 |
| 地形 `rl_terrain` | 0.4 |
| 拟人/自然 `rl_basic` | 0.3 |

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `safe_velocity_walk` | 按给定前向指令走一段。默认 50 Hz，低于 20 Hz 则拒绝。`--duration 0` 直到 Ctrl+C。退出时发零速度 | `./build/safe_velocity_walk --domain-id 69 --x 0.5 --duration 2` | `python3 examples/scenarios/safe_velocity_walk.py --domain-id 69 --x 0.5 --duration 2` |
| `keyboard_velocity_control` | 终端 WASD / QE。超过 `--deadman` 秒没有按键重复就发零速度 | `./build/keyboard_velocity_control --domain-id 69` | `python3 examples/scenarios/keyboard_velocity_control.py --domain-id 69` |

### 规划动作

`joint_motion_planner` 目前只在 `lower_body_balance` 下执行。当前运动不是它时，下面两个程序不发送 `execute`。

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `joint_plan_sequence` | 按机型执行挥手、握手或复位。`--config` 改用自己的规划 YAML。t800pro 没有挥手文件 | `./build/joint_plan_sequence --domain-id 69 --model pm01 --action wave` | `python3 examples/scenarios/joint_plan_sequence.py --domain-id 69 --model pm01 --action wave` |
| `interactive_action_trigger` | 键盘 1/2/3 或手柄 A/B/X 触发同一套动作。忙时或运动被切走时拒绝 | `./build/interactive_action_trigger --domain-id 69 --model pm01 --input keyboard` | `python3 examples/scenarios/interactive_action_trigger.py --domain-id 69 --model pm01 --input keyboard` |

挥手文件是 `cpp/examples/config/pm01/motion_plan_wave_hands.yaml` 和 `cpp/examples/config/t800/motion_plan_wave_hands.yaml`。握手是三个机型的 `motion_plan_shake_hand.yaml`。复位是三个机型的 `motion_plan_reset.yaml`。按键映射见 `cpp/examples/config/interactive_bindings.yaml`。

### 关节控制

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `joint_bridge_trajectory` | 注意安全。关节透传下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地。本程序在 `joint_bridge` 下按 YAML 的 `num_steps` 插值。必须带 `--enable-low-level`。任一关节超过 `--max-delta` 则整段不发 | `./build/joint_bridge_trajectory --domain-id 69 --enable-low-level --config examples/config/pm01/joint_test.yaml` | `python3 examples/scenarios/joint_bridge_trajectory.py --domain-id 69 --enable-low-level --config ../cpp/examples/config/pm01/joint_test.yaml` |
| `upper_body_override` | 在 `lower_body_balance` 下按 YAML 往返上肢。省略 `--seconds` 则直到 Ctrl+C，退出时发 weight 0 | `./build/upper_body_override --domain-id 69 --config examples/config/pm01/upper_joint_override.yaml --seconds 20` | `python3 examples/scenarios/upper_body_override.py --domain-id 69 --config ../cpp/examples/config/pm01/upper_joint_override.yaml --seconds 20` |
| `t800pro_hand_gesture_control` | 注意安全。关节透传下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地。仅 t800pro，43 个关节，运动为 `joint_bridge`。张开、握拳、松开，或 `--pose both` 先握拳再张开。必须带 `--enable-low-level` | `./build/t800pro_hand_gesture_control --domain-id 69 --enable-low-level --gesture open` | `python3 examples/scenarios/t800pro_hand_gesture_control.py --domain-id 69 --enable-low-level --gesture open` |

### 灯、记录、配置

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `led_control` | 按名称设置灯效。`--sequence all` 依次播放全部灯效。`--cycles 0` 直到 Ctrl+C。没有回读 | `./build/led_control --domain-id 69 --sequence all --cycles 1` | `python3 examples/scenarios/led_control.py --domain-id 69 --sequence all --cycles 1` |
| `telemetry_logger` | 按本机轮询时间写 CSV 或 JSONL | `./build/telemetry_logger --domain-id 69 --duration 5 --output /tmp/robot.csv` | `python3 examples/scenarios/telemetry_logger.py --domain-id 69 --duration 5 --output /tmp/robot.csv` |
| `gamepad_input_monitor` | 打印具名按键和摇杆。无样本时给出排查提示 | `./build/gamepad_input_monitor --domain-id 69 --watch` | `python3 examples/scenarios/gamepad_input_monitor.py --domain-id 69 --watch` |
| `yaml_motion_config_check` | 只检查 YAML，不发命令。关节数：pm01 为 24，t800 为 25，t800pro 为 43 | `./build/yaml_motion_config_check --file examples/config/pm01/motion_plan_wave_hands.yaml --model pm01 --check joint-plan` | `python3 examples/scenarios/yaml_motion_config_check.py --file ../cpp/examples/config/pm01/motion_plan_wave_hands.yaml --model pm01 --check joint-plan` |

### 组合

| 程序 | 作用 | C++ | Python |
|---|---|---|---|
| `walk_and_motion_plan` | 当前运动必须是 `lower_body_balance`。行走的同时执行挥手、握手或复位。退出时发零速度。默认前向指令 `--x` 是 0.5，对应 `linear_velocity[0]`。传入 `joint-override` 会拒绝 | `./build/walk_and_motion_plan --domain-id 69 --model pm01` | `python3 examples/scenarios/walk_and_motion_plan.py --domain-id 69 --model pm01` |
| `joint_bridge_stand_and_walk` | 注意安全。关节透传下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地。本程序在 `joint_bridge` 中保持当前关节，手柄触发后站立，再按 MNN 策略行走。运行前先进入 `pd_stand` 并保持站立，再切到 `joint_bridge`。只有 C++ | `./build/joint_bridge_stand_and_walk --domain-id 69 --model pm01 --enable-low-level` | 无 |

## 关节透传下的站立与行走

这一节只有 C++。Python 没有 MNN 运行时，不提供对应程序。

注意安全。关节透传下的站立不一定稳，需要把机器人吊起。进入行走之前必须保持双脚着地。

这个程序在 `joint_bridge` 里直接发送关节命令。周围留出空位。进入行走时先确认双脚着地，再按 LB+B。Ctrl+C 只结束本程序，运动模式仍停留在 `joint_bridge`，需要再手动切到其他运动。

它也是一个小型状态机，状态只有 `hold`、`stand`、`walk`。启动后是 `hold`，保持当时测到的关节位置。手柄 RB+A 进入 `stand`，按 `pd_stand.yaml` 做五次插值。插值完成并打印 `stand complete` 后，确认双脚着地，再按 LB+B 进入 `walk`，按 `rl_walking.yaml` 里的 MNN 策略行走。程序不会从 `walk` 自动回到 `hold`。离开 `joint_bridge` 或 Ctrl+C 时结束，不改变运动模式。

pm01、t800、t800pro 各有一套配置，目录是 `cpp/examples/config/<model>/custom_motion/`。MNN 在 `cpp/examples/third_party/mnn/`，按平台链接。编译还需要本机的 Eigen3。

只加载配置和策略、做一次推理、不建会话：

```bash
./build/joint_bridge_stand_and_walk --model pm01 --dry-run
```

连机器人时同时带 `--domain-id` 和 `--enable-low-level`。先用 `motion_mode_manager` 进入 `pd_stand`，保持站立姿态，再切换到 `joint_bridge`。`joint_bridge` 会保持切换当时的关节位置，这样进入后仍是站立姿态。当前运动不是 `joint_bridge` 时程序直接退出，不会发送切换。之后要在 `--timeout`（默认 8 秒）内收到与 `joints.yaml` 等长的关节状态，以及 IMU 和手柄样本。

```bash
./build/joint_bridge_stand_and_walk --domain-id 69 --model pm01 --enable-low-level
./build/joint_bridge_stand_and_walk --domain-id 69 --model t800 --enable-low-level
./build/joint_bridge_stand_and_walk --domain-id 69 --model t800pro --enable-low-level --interface eth0
```

进入后是 `hold`，用站立的刚度和阻尼保持启动时的关节位置。手柄取上升沿。

| 操作 | 作用 |
|---|---|
| RB+A | 开始站立。任一关节相对站立目标的偏差超过 `pd_stand.yaml` 的 `initial_joint_position_bias_threshold` 时拒绝 |
| LB+B | 注意先让机器人双脚着地，打印 `stand complete` 之后可以开始行走。站立未完成时拒绝 |
| 左摇杆 X / Y | 行走时的 x、y 速度。正负分别乘 `command_scale_pos`、`command_scale_neg` |
| 右摇杆 Y | 行走时的转向，缩放方式同上 |

周期来自 `rl_walking.yaml` 的 `control_dt`。现有配置是 0.01 秒。摇杆指令在配置打开滤波时会先做低通。大约每 0.5 秒打印一行 `state=`、`stand_done=`、`motion=`。

行走中手柄、关节或 IMU 样本过期时，摇杆指令按回中处理，策略继续运行，走动停止。`hold` 和 `stand` 在样本过期时继续发送各自的保持或站立命令。Ctrl+C，或已经离开 `joint_bridge` 时，程序直接结束，不改变运动模式。

四份配置在同一目录：`joints.yaml` 是关节名顺序，`pd_stand.yaml` 是站立目标、刚度、阻尼和时长，`rl_walking.yaml` 的 `policy_file` 指向同目录下的 `.mnn`。换目录用 `--config-dir`，其下仍要有 `<model>/custom_motion/`。
