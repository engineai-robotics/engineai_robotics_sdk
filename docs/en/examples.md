# Examples

[中文](../zh/examples.md) | English

Build and run C++ from the `cpp/` directory. In the C++ commands below, `./build/` and `examples/config` are relative to `cpp/`. Run Python from the `python/` directory with `python3 examples/...`. The default config directory for Python scenarios is `cpp/examples/config` in the repository. A YAML path written out in a command is relative to `python/`.

`joint_bridge_stand_and_walk` is C++ only. Every other example exists in both languages, with the same parameters.

C++ command-line parsing, sample waits, and Ctrl+C live in `cpp/examples/support/`. Those headers are not a public interface.

A program that needs a particular motion does not switch by itself. Use `motion_mode_manager` to enter the target motion first.

## Basic examples

C++ sources are in `cpp/examples/basic/`. Python sources are `python/examples/basic_*.py`. Each program covers one group of interfaces.

| Program | What it does | C++ | Python |
|---|---|---|---|
| `basic_session` | Create a local session. `is_open` only means the session is still held | `./build/basic_session --domain-id 69 --interface eth0` | `python3 examples/basic_session.py --domain-id 69 --interface eth0` |
| `basic_read_state` | Read joints, motors, IMU, and the gamepad, and print `since_receive` | `./build/basic_read_state --domain-id 69` | `python3 examples/basic_read_state.py --domain-id 69` |
| `basic_read_diagnostics` | Read power, motor diagnostics, heartbeat, and motor commands | `./build/basic_read_diagnostics --domain-id 69` | `python3 examples/basic_read_diagnostics.py --domain-id 69` |
| `basic_motion` | List the current motion and the transitions that can be requested. A target outside that list is not sent. After a send, poll until the switch finishes. The sample command enters joint passthrough: take care. The standing pose in this mode is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk | `./build/basic_motion --domain-id 69 --motion joint_bridge` | `python3 examples/basic_motion.py --domain-id 69 --motion joint_bridge` |
| `basic_joint_command` | Take care. The standing pose in joint passthrough is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk. C++ moves one joint a small amount in `joint_bridge`, keeps sending on a period, and checks the measured joint. Python repeats the joint position measured at startup. Its command echo does not show that the joint moved. `--seconds 0` runs until Ctrl+C | `./build/basic_joint_command --domain-id 69` | `python3 examples/basic_joint_command.py --domain-id 69` |
| `basic_joint_motion_planner` | Query the planner. `--reset` is sent only in `lower_body_balance`, then polled until it finishes | `./build/basic_joint_motion_planner --domain-id 69 --reset` | `python3 examples/basic_joint_motion_planner.py --domain-id 69 --reset` |
| `basic_body_velocity` | Send body velocity on a period, and send 0 on exit. There is no execution readback. `--vx`, `--vy`, and `--yaw` map to `linear_velocity[0]`, `linear_velocity[1]`, and `yaw_velocity` | `./build/basic_body_velocity --domain-id 69 --vx 0.5 --seconds 3` | `python3 examples/basic_body_velocity.py --domain-id 69 --vx 0.5 --seconds 3` |
| `basic_joint_override` | Refresh a joint override at 10 Hz, and send weight 0 at the end. `close()` does not release the override | `./build/basic_joint_override --domain-id 69 --seconds 1` | `python3 examples/basic_joint_override.py --domain-id 69 --seconds 1` |
| `basic_led` | Set the LED once. There is no readback | `./build/basic_led --domain-id 69` | `python3 examples/basic_led.py --domain-id 69` |

Query the current motion and do not switch:

```bash
# cpp/
./build/basic_motion --domain-id 69
# python/
python3 examples/basic_motion.py --domain-id 69
```

## Scenarios

C++ sources are in `cpp/examples/scenarios/`. Python sources are in `python/examples/scenarios/`. There is one copy of the YAML, under `cpp/examples/config/pm01/`, `t800/`, and `t800pro/`. Python programs that read YAML also need PyYAML (`python3-yaml`). It is not an SDK install dependency.

### Connection and state

| Program | What it does | C++ | Python |
|---|---|---|---|
| `connectivity_doctor` | Check the session, and whether heartbeat, motion, joints, IMU, and power have arrived. `--json` is optional | `./build/connectivity_doctor --domain-id 69 --timeout 5` | `python3 examples/scenarios/connectivity_doctor.py --domain-id 69 --timeout 5` |
| `robot_status_snapshot` | One screen of status. After the stream stops, the last values stay on screen and are marked stale. `--watch` refreshes continuously. `--json` prints fixed fields | `./build/robot_status_snapshot --domain-id 69 --json` | `python3 examples/scenarios/robot_status_snapshot.py --domain-id 69 --json` |

### Motion mode

| Program | What it does | C++ | Python |
|---|---|---|---|
| `motion_mode_manager` | List motions that can be requested. A target outside that list is not sent. `--json` is for a script to read the final result. The sample command enters joint passthrough: take care. The standing pose in this mode is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk | `./build/motion_mode_manager --domain-id 69 --target-motion joint_bridge` | `python3 examples/scenarios/motion_mode_manager.py --domain-id 69 --target-motion joint_bridge` |

### Walking

Velocity examples do not fix the motion name as `lower_body_balance`. The current motion only has to accept body velocity. `--x` / `--vx`, `--y` / `--vy`, and `--yaw` / `--yaw-rate` map to `linear_velocity[0]`, `linear_velocity[1]`, and `yaw_velocity` on the [body velocity interface](https://engineai.com.cn/open/docs/t800/ros2-interface/04). That page has the fields, units, and limits.

The table below is the suggested minimum `linear_velocity[0]` for PM01 walking forward, with lateral velocity and yaw rate at 0, sent at about 100 Hz. These are tested hints. The programs do not reject smaller values. For T800, start smaller and increase after the robot takes continuous steps. 

| Motion | Suggested minimum forward command `linear_velocity[0]` |
|---|---|
| Lower-body balance `lower_body_balance` | 0.5 |
| Terrain `rl_terrain` | 0.4 |
| Natural `rl_basic` | 0.3 |

| Program | What it does | C++ | Python |
|---|---|---|---|
| `safe_velocity_walk` | Walk at the given forward command for a while. The default is 50 Hz, and a rate below 20 Hz is rejected. `--duration 0` runs until Ctrl+C. Exit sends zero velocity | `./build/safe_velocity_walk --domain-id 69 --x 0.5 --duration 2` | `python3 examples/scenarios/safe_velocity_walk.py --domain-id 69 --x 0.5 --duration 2` |
| `keyboard_velocity_control` | Terminal WASD / QE. If no key repeats within `--deadman` seconds, zero velocity is sent | `./build/keyboard_velocity_control --domain-id 69` | `python3 examples/scenarios/keyboard_velocity_control.py --domain-id 69` |

### Planned motions

`joint_motion_planner` currently runs only in `lower_body_balance`. When the current motion is something else, the next two programs do not send `execute`.

| Program | What it does | C++ | Python |
|---|---|---|---|
| `joint_plan_sequence` | Wave, shake hands, or reset for the selected model. `--config` uses your own plan YAML. t800pro has no wave file | `./build/joint_plan_sequence --domain-id 69 --model pm01 --action wave` | `python3 examples/scenarios/joint_plan_sequence.py --domain-id 69 --model pm01 --action wave` |
| `interactive_action_trigger` | Keyboard 1/2/3 or gamepad A/B/X triggers the same motions. A busy planner, or a motion that was switched away, is rejected | `./build/interactive_action_trigger --domain-id 69 --model pm01 --input keyboard` | `python3 examples/scenarios/interactive_action_trigger.py --domain-id 69 --model pm01 --input keyboard` |

The wave files are `cpp/examples/config/pm01/motion_plan_wave_hands.yaml` and `cpp/examples/config/t800/motion_plan_wave_hands.yaml`. The handshake file is `motion_plan_shake_hand.yaml` for all three models. The reset file is `motion_plan_reset.yaml` for all three models. Key bindings are in `cpp/examples/config/interactive_bindings.yaml`.

### Joint control

| Program | What it does | C++ | Python |
|---|---|---|---|
| `joint_bridge_trajectory` | Take care. The standing pose in joint passthrough is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk. This program interpolates in `joint_bridge` for `num_steps` from the YAML. `--enable-low-level` is required. If any joint exceeds `--max-delta`, the whole trajectory is not sent | `./build/joint_bridge_trajectory --domain-id 69 --enable-low-level --config examples/config/pm01/joint_test.yaml` | `python3 examples/scenarios/joint_bridge_trajectory.py --domain-id 69 --enable-low-level --config ../cpp/examples/config/pm01/joint_test.yaml` |
| `upper_body_override` | Move the upper body back and forth from YAML in `lower_body_balance`. Omitting `--seconds` runs until Ctrl+C, and exit sends weight 0 | `./build/upper_body_override --domain-id 69 --config examples/config/pm01/upper_joint_override.yaml --seconds 20` | `python3 examples/scenarios/upper_body_override.py --domain-id 69 --config ../cpp/examples/config/pm01/upper_joint_override.yaml --seconds 20` |
| `t800pro_hand_gesture_control` | Take care. The standing pose in joint passthrough is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk. t800pro only, 43 joints, motion `joint_bridge`. Open, close, relax, or `--pose both` to close and then open. `--enable-low-level` is required | `./build/t800pro_hand_gesture_control --domain-id 69 --enable-low-level --gesture open` | `python3 examples/scenarios/t800pro_hand_gesture_control.py --domain-id 69 --enable-low-level --gesture open` |

### Lights, logs, and config

| Program | What it does | C++ | Python |
|---|---|---|---|
| `led_control` | Set the LED by name. `--sequence all` plays every pattern in order. `--cycles 0` runs until Ctrl+C. There is no readback | `./build/led_control --domain-id 69 --sequence all --cycles 1` | `python3 examples/scenarios/led_control.py --domain-id 69 --sequence all --cycles 1` |
| `telemetry_logger` | Write CSV or JSONL using the local poll time | `./build/telemetry_logger --domain-id 69 --duration 5 --output /tmp/robot.csv` | `python3 examples/scenarios/telemetry_logger.py --domain-id 69 --duration 5 --output /tmp/robot.csv` |
| `gamepad_input_monitor` | Print named buttons and sticks. When there is no sample, it prints what to check | `./build/gamepad_input_monitor --domain-id 69 --watch` | `python3 examples/scenarios/gamepad_input_monitor.py --domain-id 69 --watch` |
| `yaml_motion_config_check` | Check YAML only, and send no command. Joint counts: pm01 24, t800 25, t800pro 43 | `./build/yaml_motion_config_check --file examples/config/pm01/motion_plan_wave_hands.yaml --model pm01 --check joint-plan` | `python3 examples/scenarios/yaml_motion_config_check.py --file ../cpp/examples/config/pm01/motion_plan_wave_hands.yaml --model pm01 --check joint-plan` |

### Combined

| Program | What it does | C++ | Python |
|---|---|---|---|
| `walk_and_motion_plan` | The current motion must be `lower_body_balance`. Walk while waving, shaking hands, or resetting. Exit sends zero velocity. The default forward command `--x` is 0.5, which maps to `linear_velocity[0]`. `joint-override` is rejected | `./build/walk_and_motion_plan --domain-id 69 --model pm01` | `python3 examples/scenarios/walk_and_motion_plan.py --domain-id 69 --model pm01` |
| `joint_bridge_stand_and_walk` | Take care. The standing pose in joint passthrough is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk. This program holds the current joints in `joint_bridge`, stands when the gamepad asks, then walks with the MNN policy. Enter `pd_stand` and hold that pose before switching to `joint_bridge`. C++ only | `./build/joint_bridge_stand_and_walk --domain-id 69 --model pm01 --enable-low-level` | none |

## Stand and walk in joint passthrough

This section is C++ only. Python has no MNN runtime, so there is no matching program.

Take care. The standing pose in joint passthrough is not necessarily stable, so hoist the robot. Both feet must be on the ground before entering walk.

This program sends joint commands directly in `joint_bridge`. Leave space around the robot. Confirm both feet are on the ground, then press LB+B to walk. Ctrl+C only stops this program. The motion stays in `joint_bridge`, and another motion has to be selected by hand.

It is a small state machine with `hold`, `stand`, and `walk`. It starts in `hold` and keeps the joint positions measured at startup. RB+A enters `stand` and runs five quintic steps from `pd_stand.yaml`. After interpolation finishes and `stand complete` is printed, confirm both feet are on the ground, then press LB+B to enter `walk` and follow the MNN policy in `rl_walking.yaml`. The program does not return from `walk` to `hold` by itself. It stops when the motion leaves `joint_bridge` or on Ctrl+C, and it does not change the motion.

pm01, t800, and t800pro each have a config set in `cpp/examples/config/<model>/custom_motion/`. MNN is in `cpp/examples/third_party/mnn/` and is linked for the current platform. The build also needs the local Eigen3.

Load the config and policy, run one inference, and do not open a session:

```bash
./build/joint_bridge_stand_and_walk --model pm01 --dry-run
```

On a robot, pass both `--domain-id` and `--enable-low-level`. Use `motion_mode_manager` to enter `pd_stand`, hold the standing pose, then switch to `joint_bridge`. `joint_bridge` holds the joint positions from the moment of the switch, so the robot is still in that standing pose afterwards. If the current motion is not `joint_bridge`, the program exits and does not send a switch. It then needs, within `--timeout` (8 seconds by default), a joint state as long as `joints.yaml`, plus IMU and gamepad samples.

```bash
./build/joint_bridge_stand_and_walk --domain-id 69 --model pm01 --enable-low-level
./build/joint_bridge_stand_and_walk --domain-id 69 --model t800 --enable-low-level
./build/joint_bridge_stand_and_walk --domain-id 69 --model t800pro --enable-low-level --interface eth0
```

After entry the state is `hold`. It holds the startup joint positions with the standing stiffness and damping. Gamepad edges are rising edges.

| Input | Effect |
|---|---|
| RB+A | Start standing. Refused when any joint is farther from the stand target than `initial_joint_position_bias_threshold` in `pd_stand.yaml` |
| LB+B | Both feet must be on the ground first. Walking can start after `stand complete` is printed. Refused before standing finishes |
| Left stick X / Y | x and y velocity while walking. Positive and negative sides use `command_scale_pos` and `command_scale_neg` |
| Right stick Y | Yaw while walking, scaled the same way |

The period comes from `control_dt` in `rl_walking.yaml`. The current configs use 0.01 seconds. Stick commands pass through a low-pass filter when the config enables it. About every 0.5 seconds the program prints one line of `state=`, `stand_done=`, and `motion=`.

If the gamepad, joint, or IMU sample expires during walking, the stick command is treated as centered, the policy keeps running, and walking stops. `hold` and `stand` keep sending their hold or stand command when a sample expires. On Ctrl+C, or after the motion has already left `joint_bridge`, the program ends and does not change the motion.

Four configs share one directory. `joints.yaml` is the joint-name order. `pd_stand.yaml` is the stand target, stiffness, damping, and duration. `policy_file` in `rl_walking.yaml` points at the `.mnn` in the same directory. `--config-dir` selects another directory, which still needs `<model>/custom_motion/` underneath.
