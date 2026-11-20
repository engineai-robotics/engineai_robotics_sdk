"""RobotSession Python API names. The meaning matches engineai_sdk/robot.h and types.h.

This file is for reading and type checking. Runtime classes come from
lib/<platform>/_engineai_sdkpy*.so.

latest() returns the most recent cached sample. After the robot stops
publishing, it still returns the old value and does not become None.
Freshness comes from last_receive_time(): seconds on the same clock as
time.monotonic(), or None when nothing has arrived.

Every method can be called from any thread. The GIL is released during
send, read, create(), and close().
"""

from collections.abc import MutableSequence, Sequence
from enum import IntEnum
from types import TracebackType

class SessionError(IntEnum):
    """Result of a call in this process. kOk does not mean the robot executed it."""

    kOk = 0
    kSessionClosed = 1
    kTransportUnavailable = 2
    kPublishFailed = 3
    kInvalidArgument = 4

class LedColor(IntEnum):
    kBlinkRed = 0x1
    kBlinkGreen = 0x2
    kBlinkBlue = 0x3
    kBlinkWhite = 0x4
    kConstantOnWhite = 0x5
    kConstantOnGreen = 0x6
    kBreatheWhite = 0x7
    kWaterWhite = 0x8
    kBreatheRed = 0x9
    kBlinkOrange = 0xA
    kConstantOnOrange = 0xB
    kBlinkRedAndWhite = 0xC
    kConstantOnRed = 0xD

class ParallelParserType(IntEnum):
    kClassic = 0
    kRl = 1

class JointMotionPlanRequestType(IntEnum):
    kPlanExecute = 0
    kCancel = 1
    kReset = 2

class JointMotionPlanStatus(IntEnum):
    kDisabled = 0
    kIdle = 1
    kExecuting = 2
    kExiting = 3

class GamepadButton(IntEnum):
    """Index into GamepadKeys.digital_states. 0 is released, 1 is pressed."""

    kLb = 0
    kRb = 1
    kA = 2
    kB = 3
    kX = 4
    kY = 5
    kBack = 6
    kStart = 7
    kCrossXUp = 8
    kCrossXDown = 9
    kCrossYLeft = 10
    kCrossYRight = 11

class GamepadAxis(IntEnum):
    """Index into GamepadKeys.analog_states. Values are in [-1, 1]."""

    kLt = 0
    kRt = 1
    kLeftStickX = 2
    kLeftStickY = 3
    kRightStickX = 4
    kRightStickY = 5

class DoubleList(MutableSequence[float]):
    """float64 vector. Index assignment writes through. Assign a sequence to replace it. json.dumps needs list(values)."""

    def __init__(self, values: Sequence[float] = ...) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> float: ...
    def __setitem__(self, index: int, value: float) -> None: ...
    def __delitem__(self, index: int) -> None: ...
    def insert(self, index: int, value: float) -> None: ...

class FloatList(MutableSequence[float]):
    """float32 vector. Same write behavior as DoubleList."""

    def __init__(self, values: Sequence[float] = ...) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> float: ...
    def __setitem__(self, index: int, value: float) -> None: ...
    def __delitem__(self, index: int) -> None: ...
    def insert(self, index: int, value: float) -> None: ...

class IntList(MutableSequence[int]):
    def __init__(self, values: Sequence[int] = ...) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> int: ...
    def __setitem__(self, index: int, value: int) -> None: ...
    def __delitem__(self, index: int) -> None: ...
    def insert(self, index: int, value: int) -> None: ...

class ByteList(MutableSequence[int]):
    """uint8 vector. Values are integers."""

    def __init__(self, values: Sequence[int] = ...) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> int: ...
    def __setitem__(self, index: int, value: int) -> None: ...
    def __delitem__(self, index: int) -> None: ...
    def insert(self, index: int, value: int) -> None: ...

class StringList(MutableSequence[str]):
    def __init__(self, values: Sequence[str] = ...) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> str: ...
    def __setitem__(self, index: int, value: str) -> None: ...
    def __delitem__(self, index: int) -> None: ...
    def insert(self, index: int, value: str) -> None: ...

class SessionOptions:
    """create() uses only these fields to open local DDS. They are not a robot address and not handshake parameters."""

    dds_domain_id: int
    network_interface_name: str
    network_address: str
    def __init__(self) -> None: ...

class Vector3:
    x: float
    y: float
    z: float
    def __init__(self) -> None: ...

class Quaternion:
    """Order is x, y, z, w."""

    x: float
    y: float
    z: float
    w: float
    def __init__(self) -> None: ...

class JointState:
    position: DoubleList
    velocity: DoubleList
    torque: DoubleList
    def __init__(self) -> None: ...

class JointCommand:
    position: DoubleList
    velocity: DoubleList
    feed_forward_torque: DoubleList
    torque: DoubleList
    stiffness: DoubleList
    damping: DoubleList
    parallel_parser_type: ParallelParserType
    def __init__(self) -> None: ...

class BodyVelocityCommand:
    """The robot clamps each component to [-1, 1]. Refresh faster than 500 ms while moving; send all zeros to stop."""

    x: float
    y: float
    yaw_rate: float
    def __init__(self) -> None: ...

class JointOverrideCommand:
    """Send weight 0 to end an override, and do it before close(). close() does not send it."""

    weight: float
    joint_indices: IntList
    command: JointCommand
    def __init__(self) -> None: ...

class ImuInfo:
    quaternion: Quaternion
    rpy: Vector3
    linear_acceleration: Vector3
    angular_velocity: Vector3
    def __init__(self) -> None: ...

class GamepadKeys:
    hardware_connected: bool
    digital_states: list[int]
    analog_states: list[float]
    def __init__(self) -> None: ...

class JointMotionPlanRequest:
    request_id: int
    request_type: JointMotionPlanRequestType
    use_gravity_compensation: bool
    joint_indices: IntList
    target_positions: DoubleList
    target_velocities: DoubleList
    execution_time: float
    stiffness: DoubleList
    damping: DoubleList
    def __init__(self) -> None: ...

class JointMotionPlanState:
    request_id: int
    status: JointMotionPlanStatus
    progress: float
    def __init__(self) -> None: ...

class PowerInfo:
    enable: bool
    percentage: float
    voltage: float
    current: float
    current_limit: float
    error_code: int
    def __init__(self) -> None: ...

class BatteryInfo:
    soc: int
    bms_type: int
    bms_current: int
    total_voltage: int
    temp: float
    charge_count: int
    capacity_remaining: int
    protect_word: int
    error_word: int
    cell_max_voltage: int
    cell_min_voltage: int
    battery_status: int
    cycle_num: int
    voltage_diff: int
    soh: int
    sop: int
    ntc0: float
    ntc1: float
    ntc2: float
    ntc3: float
    ntc4: float
    ntc5: float
    def __init__(self) -> None: ...

class PowerSystemInfo:
    """battery_info is None when this sample has no BMS data."""

    power_info: PowerInfo
    battery_info: BatteryInfo | None
    def __init__(self) -> None: ...

class MotorDebug:
    mos_temperature: FloatList
    motor_temperature: FloatList
    voltage: FloatList
    current: FloatList
    error_code: IntList
    offline: ByteList
    enable: ByteList
    def __init__(self) -> None: ...

class Heartbeat:
    """Published about once per second. Liveness is NodeHeartbeatReader.last_receive_time()."""

    node_name: str
    startup_timestamp: int
    """Wall clock when the robot published this sample, Unix epoch nanoseconds. Not the startup time."""
    node_status: str
    error_code: int
    error_message: str
    def __init__(self) -> None: ...

class MotionController:
    def request_transition(self, motion_name: str) -> SessionError:
        """kOk only means the request was sent. current_motion() is the result."""
    def current_motion(self) -> str | None: ...
    def available_transitions(self) -> StringList | None: ...
    def last_receive_time(self) -> float | None:
        """Arrival time of the motion-state sample behind the two reads above, in time.monotonic() seconds."""

class JointMotionPlanner:
    def execute(self, request: JointMotionPlanRequest) -> SessionError: ...
    def latest_state(self) -> JointMotionPlanState | None: ...
    def last_receive_time(self) -> float | None: ...

class JointStateReader:
    def latest(self) -> JointState | None: ...
    def last_receive_time(self) -> float | None: ...

class MotorStateReader:
    def latest(self) -> JointState | None: ...
    def last_receive_time(self) -> float | None: ...

class ImuReader:
    def latest(self) -> ImuInfo | None: ...
    def last_receive_time(self) -> float | None: ...

class GamepadReader:
    def latest(self) -> GamepadKeys | None: ...
    def last_receive_time(self) -> float | None: ...

class JointCommandClient:
    def send(self, command: JointCommand) -> SessionError:
        """Valid only in motions that accept whole-body joint commands, such as joint_bridge, and must be sent every control period."""
    def latest_feedback(self) -> JointCommand | None: ...
    def last_feedback_time(self) -> float | None:
        """Arrival time of the latest_feedback() sample, in time.monotonic() seconds."""

class BaseVelocityPublisher:
    def send(self, command: BodyVelocityCommand) -> SessionError: ...

class JointOverridePublisher:
    def send(self, command: JointOverrideCommand) -> SessionError: ...

class LedController:
    def set_mode(self, mode: LedColor) -> SessionError:
        """kOk only means this process sent the call. There is no interface to read the current LED color."""

class PowerReader:
    def latest(self) -> PowerSystemInfo | None: ...
    def last_receive_time(self) -> float | None: ...

class MotorDiagnosticsReader:
    def latest(self) -> MotorDebug | None: ...
    def last_receive_time(self) -> float | None: ...

class NodeHeartbeatReader:
    def latest(self) -> Heartbeat | None: ...
    def last_receive_time(self) -> float | None: ...

class MotorCommandReader:
    def latest(self) -> JointCommand | None: ...
    def last_receive_time(self) -> float | None: ...

class Robot:
    """create() only creates local DDS resources. is_open() only means those resources are still held."""

    @staticmethod
    def create(options: SessionOptions) -> SessionCreateResult: ...
    def close(self) -> None:
        """Close the local session. Does not send joint override weight 0.

        Can be called while other threads are still reading or writing.
        Those calls then return None or kSessionClosed.
        """
    def is_open(self) -> bool: ...
    def __enter__(self) -> Robot: ...
    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> bool: ...
    @property
    def motion(self) -> MotionController: ...
    @property
    def joint_motion_planner(self) -> JointMotionPlanner: ...
    @property
    def joint_state(self) -> JointStateReader: ...
    @property
    def motor_state(self) -> MotorStateReader: ...
    @property
    def imu(self) -> ImuReader: ...
    @property
    def gamepad(self) -> GamepadReader: ...
    @property
    def joint_command(self) -> JointCommandClient: ...
    @property
    def base_velocity(self) -> BaseVelocityPublisher: ...
    @property
    def joint_override(self) -> JointOverridePublisher: ...
    @property
    def led(self) -> LedController: ...
    @property
    def power(self) -> PowerReader: ...
    @property
    def motor_diagnostics(self) -> MotorDiagnosticsReader: ...
    @property
    def node_heartbeat(self) -> NodeHeartbeatReader: ...
    @property
    def motor_command(self) -> MotorCommandReader: ...

class SessionCreateResult:
    """When bool(result) is true, session is usable and error is kOk."""

    error: SessionError
    session: Robot | None
    def __bool__(self) -> bool: ...

__version__: str
