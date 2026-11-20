#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace engineai_sdk {

// Result of a local SDK call. kOk means this process completed the call, not
// that the robot applied a command.
enum class SessionError {
  kOk = 0,                // Call succeeded locally.
  kSessionClosed,         // The local session has already been closed.
  kTransportUnavailable,  // Local DDS session could not be created.
  kPublishFailed,         // Local send failed; does not imply the robot acted.
  kInvalidArgument,       // Caller input rejected (e.g. missing domain id).
};

// Requested lamp pattern. The robot may show a different pattern while a
// higher-priority fault or safety indication is active.
enum class LedColor : uint8_t {
  kBlinkRed = 0x1,
  kBlinkGreen = 0x2,
  kBlinkBlue = 0x3,
  kBlinkWhite = 0x4,
  kConstantOnWhite = 0x5,
  kConstantOnGreen = 0x6,
  kBreatheWhite = 0x7,
  kWaterWhite = 0x8,
  kBreatheRed = 0x9,
  kBlinkOrange = 0xa,
  kConstantOnOrange = 0xb,
  kBlinkRedAndWhite = 0xc,
  kConstantOnRed = 0xd,
};

// Parallel-mechanism parser used with a joint command.
enum class ParallelParserType : uint8_t {
  kClassic = 0,
  kRl = 1,
};

// What a joint-motion-plan request asks the robot to do.
enum class JointMotionPlanRequestType : uint8_t {
  kPlanExecute = 0,  // Plan and execute the given targets.
  kCancel = 1,       // Cancel the request identified by request_id.
  kReset = 2,        // Return to the default pose.
};

// Latest planner lifecycle reported by the robot.
enum class JointMotionPlanStatus : uint8_t {
  kDisabled = 0,
  kIdle = 1,
  kExecuting = 2,
  kExiting = 3,
};

// Indices into GamepadKeys::digital_states (size 12). 0 = released, 1 = pressed.
enum class GamepadButton : uint8_t {
  kLb = 0,
  kRb = 1,
  kA = 2,
  kB = 3,
  kX = 4,
  kY = 5,
  kBack = 6,
  kStart = 7,
  kCrossXUp = 8,
  kCrossXDown = 9,
  kCrossYLeft = 10,
  kCrossYRight = 11,
};

// Indices into GamepadKeys::analog_states (size 6). Values are in [-1, 1].
enum class GamepadAxis : uint8_t {
  kLt = 0,
  kRt = 1,
  kLeftStickX = 2,
  kLeftStickY = 3,
  kRightStickX = 4,
  kRightStickY = 5,
};

// Local session parameters. create() uses these to open DDS on this machine;
// they are not a robot address or handshake.
struct SessionOptions {
  // Required. Non-negative DDS domain id; must equal the robot's ROS_DOMAIN_ID
  // (0 when that variable is unset). Negative values make create() return
  // kInvalidArgument.
  int dds_domain_id{-1};
  // Optional local NIC name on this machine (e.g. "eth0"), not a robot IP.
  // Non-empty takes priority over network_address.
  std::string network_interface_name;
  // Optional IPv4 network (host part zero, e.g. "192.168.0.0"). The NIC name
  // may differ per machine; CycloneDDS matches the interface on that network.
  // Empty, together with an empty network_interface_name, => autodetermine.
  std::string network_address;
};

// Cartesian 3-vector. Units come from the containing type.
struct Vector3 {
  double x{};
  double y{};
  double z{};
};

// Unit quaternion in x, y, z, w order (w is the scalar).
struct Quaternion {
  double x{};
  double y{};
  double z{};
  double w{};
};

// One sample of measured joint or motor feedback. Arrays are the same length,
// one entry per joint, in the robot's joint order. joint_state() is joint
// space; motor_state() is motor space.
struct JointState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> torque;
};

// Full-body joint command. Arrays are the same length, one entry per joint.
// Effective only while the current motion accepts joint commands (for example
// joint_bridge). Keep sending at a control rate; one send does not hold a pose.
struct JointCommand {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> feed_forward_torque;
  std::vector<double> torque;
  std::vector<double> stiffness;
  std::vector<double> damping;
  ParallelParserType parallel_parser_type{ParallelParserType::kClassic};
};

// Body-frame locomotion command. Effective only in a motion that accepts it.
// Refresh faster than every 500 ms while active; send zeros to stop.
// The robot clamps each component to [-1, 1].
struct BodyVelocityCommand {
  double x{};         // Forward
  double y{};         // Lateral
  double yaw_rate{};  // Yaw
};

// Partial joint overlay on top of the current motion. Effective only in motions
// that accept override. When finished, send weight 0 before close(). close()
// does not send it. The locomotion node keeps the last positive-weight sample
// until that weight 0 sample arrives.
struct JointOverrideCommand {
  double weight{};                     // Blend weight; 0 releases
  std::vector<int32_t> joint_indices;  // Joints to overlay, robot joint order
  JointCommand command;                // Targets for those joints
};

// IMU orientation and inertial readings in the IMU frame.
struct ImuInfo {
  Quaternion quaternion;
  Vector3 rpy;                  // Roll, pitch, yaw, rad
  Vector3 linear_acceleration;  // m/s^2
  Vector3 angular_velocity;     // rad/s
};

// Latest hardware gamepad snapshot. Use GamepadButton / GamepadAxis as indices.
struct GamepadKeys {
  bool hardware_connected{};                 // True when a physical pad is present
  std::array<int32_t, 12> digital_states{};  // 0 = released, 1 = pressed
  std::array<double, 6> analog_states{};     // [-1, 1]
};

// Request sent to the on-robot joint motion planner.
struct JointMotionPlanRequest {
  int32_t request_id{};
  JointMotionPlanRequestType request_type{JointMotionPlanRequestType::kPlanExecute};
  bool use_gravity_compensation{};
  std::vector<int32_t> joint_indices;     // Empty means reset to default
  std::vector<double> target_positions;   // rad
  std::vector<double> target_velocities;  // Empty lets the planner choose
  double execution_time{};                // Expected duration, s
  std::vector<double> stiffness;          // Empty uses robot defaults
  std::vector<double> damping;            // Empty uses robot defaults
};

// Latest planner sample for the request that the robot is tracking.
struct JointMotionPlanState {
  int32_t request_id{};
  JointMotionPlanStatus status{JointMotionPlanStatus::kDisabled};
  double progress{};  // 0 to 1
};

// Power-board summary from the latest power sample.
struct PowerInfo {
  bool enable{};       // Power output enabled
  float percentage{};  // Remaining energy, %
  float voltage{};     // V
  float current{};     // A
  float current_limit{};
  int32_t error_code{};  // Power-board fault bits; 0 means no reported fault
};

// BMS battery details. Present only when PowerSystemInfo::battery_info has a value.
struct BatteryInfo {
  uint8_t soc{};                  // State of charge, %
  uint8_t bms_type{};             // BMS type / protocol id
  int32_t bms_current{};          // mA; positive = discharge, negative = charge
  uint32_t total_voltage{};       // Pack voltage, mV
  float temp{};                   // Battery temperature, °C
  uint16_t charge_count{};        // Cumulative charge cycles counted by the BMS
  uint32_t capacity_remaining{};  // mAh
  uint32_t protect_word{};        // BMS protection bitmask
  uint32_t error_word{};          // BMS error bitmask
  uint16_t cell_max_voltage{};    // mV
  uint16_t cell_min_voltage{};    // mV
  uint16_t battery_status{};      // BMS operating status code
  uint16_t cycle_num{};           // Charge/discharge cycle count
  uint16_t voltage_diff{};        // Max cell delta (max - min), mV
  uint16_t soh{};                 // State of health, %
  uint16_t sop{};                 // State of power, %
  float ntc0{};                   // NTC0, °C
  float ntc1{};                   // NTC1, °C
  float ntc2{};                   // NTC2, °C
  float ntc3{};                   // NTC3, °C
  float ntc4{};                   // NTC4, °C
  float ntc5{};                   // NTC5, °C
};

// Latest power-system sample. battery_info is empty when BMS data is unavailable.
struct PowerSystemInfo {
  PowerInfo power_info;
  std::optional<BatteryInfo> battery_info;
};

// Per-motor diagnostics. Arrays are the same length, one entry per motor.
struct MotorDebug {
  std::vector<float> mos_temperature;
  std::vector<float> motor_temperature;
  std::vector<float> voltage;
  std::vector<float> current;
  std::vector<int32_t> error_code;  // Driver fault bits; 0 means no reported fault
  std::vector<uint8_t> offline;     // 0 = online, 1 = offline
  std::vector<uint8_t> enable;      // 0 = disabled, 1 = enabled
};

// Liveness sample from a robot node. Judge liveness by how recently it arrived
// (NodeHeartbeatReader::last_receive_time()), not by having one.
struct Heartbeat {
  std::string node_name;
  int64_t startup_timestamp{};  // Robot wall clock at each publish, Unix epoch ns (not the start time)
  std::string node_status;      // idle, running, error, or sleep
  int64_t error_code{};
  std::string error_message;
};

}  // namespace engineai_sdk
