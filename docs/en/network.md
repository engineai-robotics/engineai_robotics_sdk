# Networking

[中文](../zh/network.md) | English

`--domain-id` is the robot `ROS_DOMAIN_ID`. When that variable is unset, ROS uses domain `0`. On a running robot it is `69`.

## On the same machine

When the robot has `ROS_LOCALHOST_ONLY=1`, the client first runs:

```bash
export ROS_LOCALHOST_ONLY=1
```

Do not pass `--interface` or `--network-address`. Either flag rebinds DDS to another NIC, and the program will not find the robot.

## Across machines

Pass one of the two parameters below to select the client NIC that can reach the robot. Neither value is the robot IP. If both are omitted, CycloneDDS chooses. If both are set, the NIC name wins.

`--interface` is the local NIC name, for example `eth0`.

```bash
ip -br link
```

`--network-address` is the local IPv4 subnet with a zero host part, for example `192.168.0.0`. NIC names may differ between machines. CycloneDDS looks for a NIC on that subnet.

```bash
ip -br addr
```

Run C++ from `cpp/` and Python from `python/`:

```bash
./build/basic_read_state --domain-id 69 --interface eth0
./build/basic_read_state --domain-id 69 --network-address 192.168.0.0
python3 examples/basic_read_state.py --domain-id 69 --interface eth0
python3 examples/basic_read_state.py --domain-id 69 --network-address 192.168.0.0
```

## Still no sample

When the output is `(no sample)`, check each item:

1. The robot process is running.
2. `--domain-id` equals the robot `ROS_DOMAIN_ID`.
3. On the same machine with `ROS_LOCALHOST_ONLY=1`, the client has exported that variable, and the command line has neither `--interface` nor `--network-address`.
4. Across machines, `--interface` or `--network-address` selects the client NIC that leads to the robot.
