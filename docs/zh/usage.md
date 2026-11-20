# 调用约定

中文 | [English](../en/usage.md)

C++ 和 Python 的调用语义相同。下文代码是 C++。Python 的类名和方法名见 `python/src/engineai_sdkpy/__init__.pyi`，安装和最短调用见 [python/README.zh.md](../../python/README.zh.md)。

公开接口只有 `Robot::create()` 和 `Robot` 上的各个能力访问器。头文件不包含 DDS 或 ROS 类型。

## 会话

`Robot::create()` 只创建本机的 DDS domain、participant 和 SDK 端点，不与机器人握手。

`result` 为假时，`result.session` 为空，原因在 `result.error`。为真时，`session->is_open()` 只表示本机资源还在，不表示已经发现机器人或收到数据。

确认连上机器人的办法是读到样本，例如心跳、运动状态或关节状态。C++ 示例里的等待写在 `cpp/examples/support/common_cli.hpp` 的 `WaitForRobotSample`。Python 写在 `python/examples/_cli.py` 的 `wait_for_robot_sample`。

## 样本是否新鲜

`latest()` 返回最近一次缓存。机器人停止发布或网络断开后，它仍返回最后一次的值。

是否新鲜看 `last_receive_time()`。C++ 里它是本机 `std::chrono::steady_clock` 上最近一次收到该话题的时间，还没收到过时是 `std::nullopt`。Python 里它是秒，与 `time.monotonic()` 同一时钟，还没收到过时是 `None`。`JointCommandClient` 对应的是 `last_feedback_time()`。

先读 `last_receive_time()`，再读 `latest()`。这段间隔称为距上次收到的时长，这样估算只会偏长。

```cpp
const auto t = session->joint_state().last_receive_time();
if (!t || std::chrono::steady_clock::now() - *t > std::chrono::milliseconds(100)) {
  // 停止发送命令
}
session->joint_state().latest(joint);
```

要区分新样本和旧缓存时，保存上一次的时间。两次时间不同，就是新样本。

阈值按发布周期放大若干倍。示例里的取值：

| 数据 | 发布周期 | 示例阈值 |
|---|---|---|
| 关节、IMU | 约 2 ms | 100 ms |
| 运动状态 | 约 200 ms | 1 s |
| 心跳 | 约 1 s | 3 s |

对应代码：C++ 是 `cpp/examples/support/common_cli.hpp` 的 `IsFresh` 和 `RobotOnline`；Python 是 `python/examples/_cli.py` 的 `is_fresh` 和 `robot_online`。`basic_joint_command`、`joint_bridge_stand_and_walk` 用距上次收到的时长判断样本是否过期。`telemetry_logger` 比较前后两次 `last_receive_time()`，并输出 `joint_since_receive_ms` 和 `joint_new`。`basic_read_state` 把同一段时长打印为 `since_receive`。

## 命令是否生效

控制接口返回 `SessionError::kOk`，只说明本机调用完成，不说明机器人已经收到或已经执行。

一次性命令在 `close()` 前稍停，避免会话拆掉时这次调用还没被机器人收到。示例里是 `PauseBeforeClose`。

需要机器人处于特定运动时，先读 `current_motion()` 和 `available_transitions()`。目标不在可切换列表中时不要发送切换。切换是否完成，以之后读到的当前运动为准。

不要同时用本 SDK 和 ROS 高频发送关节命令或速度命令。导航接管期间不要发送 SDK 身体速度。

## 线程

所有方法都可以从任意线程调用，包括 `close()`。`close()` 之后，调用返回 `kSessionClosed` 或空值。其他线程还在使用 `Robot` 时，不要销毁该对象。Python 在发送、读取、`create()` 和 `close()` 期间释放 GIL，说明见 [python/README.zh.md](../../python/README.zh.md)。
