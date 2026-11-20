# EngineAI RobotSession Python SDK

中文 | [English](README.md)

这是 C++ RobotSession 公共 API 的语言绑定。

类名、方法名和字段见 `src/engineai_sdkpy/__init__.pyi`。本文件说明怎么安装和怎么调用。命令都在 `python/` 目录执行。

三个平台的扩展、会话库、`libddsc` 和 iceoryx 在仓库根的 `lib/<platform>/`，由生产仓同步写入。加载器和示例留在 `python/`。版本号与 C++ SDK 相同。

## 安装

建议在虚拟环境中运行。用本机系统 `python3` 创建环境：Ubuntu 22.04 是 3.10，Ubuntu 24.04 是 3.12。扩展按这个版本编译（`cpython-310` / `cpython-312`）。conda、pyenv 或其他小版本加载不了对应的 `.so`。

在 `python/` 目录执行：

```bash
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
python3 -c "import engineai_sdkpy; print(engineai_sdkpy.__version__)"
```

执行 `source .venv/bin/activate` 之后，本终端的 `python3` 会优先使用 `.venv/bin/python3`。包装进这个虚拟环境的解释器。新开终端需要再执行一次 `source .venv/bin/activate`。

不建环境也可以，用系统 `python3` 直接跑：

```bash
PYTHONPATH=src python3 -c "import engineai_sdkpy; print(engineai_sdkpy.__version__)"
```

`pip install .`（不带 `-e`）不会把 `lib/` 装进 site-packages。加载器从仓库根找 `lib/`，所以只能用上面的可编辑安装，或者 `PYTHONPATH=src`。

平台由 `ENGINEAI_ROBOT_SESSION_PLATFORM` 决定；未设置时读取 `/etc/os-release` 和 CPU。不在
`ubuntu22.04-amd64`、`ubuntu22.04-arm64`、`ubuntu24.04-arm64` 之中则 `import` 失败。

每个 `lib/<platform>/` 里是该平台运行所需的全部文件：Python 扩展、`libEngineAIRobotSession.so`、`libddsc.so.0.10.5` 和四份 iceoryx。不需要设置 `LD_LIBRARY_PATH`。扩展和会话库的 `RUNPATH` 都是 `$ORIGIN`。

## 用法

`--domain-id` 必须等于机器人的 `ROS_DOMAIN_ID`。未设置时是 `0`。机器人运行时，这个值一般是 `69`。组网见 [docs/zh/network.md](../docs/zh/network.md)。

下列命令在已激活的虚拟环境中执行。若未做可编辑安装，则在命令前加 `PYTHONPATH=src`。

仿真时先在同一个终端执行 `export ROS_LOCALHOST_ONLY=1`，不要传 `--interface` 或 `--network-address`：

```bash
python3 examples/basic_session.py --domain-id 69
```

真机跨机器时，再传本机网卡名：

```bash
python3 examples/basic_session.py --domain-id 69 --interface eth0
```

下面这份程序和 C++ 最短用法做同一件事：等待心跳，确认在线后设置一次绿灯，检查返回值，等待 0.5 秒再关闭。`--domain-id` 必须是完整的非负整数。仿真不要设置 `network_interface_name`。真机跨机器时再把它设为本机网卡名。`try`/`finally` 保证超时和异常退出时也会关闭会话。模式切换见 `basic_motion.py`，不在这段里做。

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

`create()` 只建立本机 DDS 资源，不是与机器人握手。`is_open()` 只表示本机资源仍在。`kOk` 表示本进程调用完成，不表示机器人已经执行。速度需要快于 500 ms 刷新。Override 结束时必须自己发送 `weight=0`，并且要在 `close()` 之前。`close()` 不会代发。

`with session:` 退出时调用 `close()`。`close()` 之后的写操作返回 `kSessionClosed`。调用是否应发送、样本是否新鲜，见 [docs/zh/usage.md](../docs/zh/usage.md)。

### 判断机器人状态是否仍在更新

发送依赖机器人状态的控制命令前，先确认这份状态仍在更新。`latest()` 读到的是最近一次缓存。机器人停止发布或网络断开后，仍可能读到旧值，不能只凭「读到了数据」判断通信正常。

`last_receive_time()` 是秒，与 `time.monotonic()` 同一时钟。`None` 表示尚未收到数据；距上次收到的时间过长则视为过期，不要用这份数据发命令。`joint_command` 对应 `last_feedback_time()`。先读时间，再读值。阈值见 [docs/zh/usage.md](../docs/zh/usage.md)。示例写法见 `examples/_cli.py` 的 `is_fresh` 和 `robot_online`。

```python
t = session.joint_state.last_receive_time()
if t is None or time.monotonic() - t > 0.1:
    ...  # 数据过期：停止发送命令
joint = session.joint_state.latest()
```

### 线程

所有方法都可以在任意线程调用，包括在其他线程读写时调用 `close()`，之后这些调用返回 `None` 或 `kSessionClosed`。发送、读取、`create()` 和 `close()` 期间会释放 GIL，阻塞的 DDS 调用不会卡住其他 Python 线程。

## 示例

全部命令见 [docs/zh/examples.md](../docs/zh/examples.md)。场景程序与 C++ 同名，参数相同。没有 `joint_bridge_stand_and_walk`。不带路径时，它们读取仓库里的 `cpp/examples/config`。需要读 YAML 的程序另装 PyYAML（`python3-yaml`），这不是本包的安装依赖。
