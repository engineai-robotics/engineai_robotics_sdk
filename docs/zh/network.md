# 组网

中文 | [English](../en/network.md)

`--domain-id` 填写机器人的 `ROS_DOMAIN_ID`。该变量未设置时，ROS 使用 domain `0`。机器人正常运行时为 `69`。

## 同一台机器

机器人设置了 `ROS_LOCALHOST_ONLY=1` 时，客户机先执行：

```bash
export ROS_LOCALHOST_ONLY=1
```

此时不要再传 `--interface` 或 `--network-address`。传了会改绑到别的网卡，发现不到机器人。

## 跨机器

用下面两个参数之一，选择客户机上能到达机器人的网卡。它们都不是机器人的 IP。两个都省略时，由 CycloneDDS 自己选择。两个都填写时，网卡名优先。

`--interface` 是本机网卡名，例如 `eth0`。

```bash
ip -br link
```

`--network-address` 是本机所在的 IPv4 网段，主机位为 0，例如 `192.168.0.0`。各机器的网卡名可以不同，CycloneDDS 会在该网段上找网卡。

```bash
ip -br addr
```

C++ 在 `cpp/` 目录执行，Python 在 `python/` 目录执行：

```bash
./build/basic_read_state --domain-id 69 --interface eth0
./build/basic_read_state --domain-id 69 --network-address 192.168.0.0
python3 examples/basic_read_state.py --domain-id 69 --interface eth0
python3 examples/basic_read_state.py --domain-id 69 --network-address 192.168.0.0
```

## 仍然没有样本

输出 `(no sample)` 时，逐项确认：

1. 机器人进程在运行。
2. `--domain-id` 与机器人的 `ROS_DOMAIN_ID` 相同。
3. 同机且 `ROS_LOCALHOST_ONLY=1` 时，客户机已导出该变量，并且命令行没有 `--interface` 和 `--network-address`。
4. 跨机时，`--interface` 或 `--network-address` 指向客户机通往机器人的那块网卡。
