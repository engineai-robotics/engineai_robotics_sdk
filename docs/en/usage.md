# Calling rules

[中文](../zh/usage.md) | English

C++ and Python have the same calling rules. The code below is C++. Python class and method names are in `python/src/engineai_sdkpy/__init__.pyi`. Install and the shortest call are in [python/README.md](../../python/README.md).

The public interface is `Robot::create()` and the capability accessors on `Robot`. The headers do not contain DDS or ROS types.

## Session

`Robot::create()` creates the local DDS domain, participant, and SDK endpoints. It does not handshake with the robot.

When `result` is false, `result.session` is empty and the reason is `result.error`. When it is true, `session->is_open()` only means the local resources are still held. It does not mean a robot has been discovered or that data has arrived.

The way to confirm a robot is connected is to read a sample, such as heartbeat, motion state, or joint state. The C++ example wait is `WaitForRobotSample` in `cpp/examples/support/common_cli.hpp`. The Python wait is `wait_for_robot_sample` in `python/examples/_cli.py`.

## Whether a sample is fresh

`latest()` returns the most recent cached value. After the robot stops publishing or the network drops, it still returns that last value.

Freshness comes from `last_receive_time()`. In C++ it is the last time this topic was received on the local `std::chrono::steady_clock`. It is `std::nullopt` when nothing has arrived. In Python it is seconds on the same clock as `time.monotonic()`, and `None` when nothing has arrived. `JointCommandClient` uses `last_feedback_time()`.

Read `last_receive_time()` first, then `latest()`. The interval is the time since the last receive, and this estimate can only be long.

```cpp
const auto t = session->joint_state().last_receive_time();
if (!t || std::chrono::steady_clock::now() - *t > std::chrono::milliseconds(100)) {
  // stop sending commands
}
session->joint_state().latest(joint);
```

To tell a new sample from the old cache, keep the previous time. A different time is a new sample.

Thresholds are several times the publish period. The examples use:

| Data | Publish period | Example threshold |
|---|---|---|
| Joints, IMU | about 2 ms | 100 ms |
| Motion state | about 200 ms | 1 s |
| Heartbeat | about 1 s | 3 s |

The C++ code is `IsFresh` and `RobotOnline` in `cpp/examples/support/common_cli.hpp`. The Python code is `is_fresh` and `robot_online` in `python/examples/_cli.py`. `basic_joint_command` and `joint_bridge_stand_and_walk` use the time since the last receive to decide that a sample has expired. `telemetry_logger` compares successive `last_receive_time()` values and prints `joint_since_receive_ms` and `joint_new`. `basic_read_state` prints the same interval as `since_receive`.

## Whether a command took effect

A control call that returns `SessionError::kOk` only means the local call finished. It does not mean the robot has received or executed it.

Pause briefly before `close()` for a one-shot command, so the call is not torn down with the session before the robot receives it. The examples use `PauseBeforeClose`.

When the robot must be in a particular motion, read `current_motion()` and `available_transitions()` first. Do not send a transition when the target is not in the available list. The current motion read afterwards is what shows that the transition finished.

Do not send high-rate joint or velocity commands from this SDK and from ROS at the same time. Do not send SDK body velocity while navigation has taken over.

## Threads

Every method can be called from any thread, including `close()`. After `close()`, calls return `kSessionClosed` or an empty value. Do not destroy the `Robot` object while another thread is still using it. Python releases the GIL during send, read, `create()`, and `close()`. See [python/README.md](../../python/README.md).
