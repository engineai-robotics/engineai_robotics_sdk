#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "engineai_sdk/types.h"

namespace engineai_sdk {

// Local std::chrono::steady_clock time at which the newest valid sample of a
// topic arrived. Read accessors keep returning the last sample after the robot
// stops publishing, so compare this with steady_clock::now() to detect stale
// data. Empty until a sample arrives, and again after close().
using ReceiveTime = std::chrono::steady_clock::time_point;

class MotionController {
 public:
  virtual ~MotionController() = default;

  // Asynchronous. kOk means the request was sent. Confirm with current_motion();
  // the name must appear in available_transitions() first.
  virtual SessionError request_transition(const std::string& motion_name) = 0;
  // Last known motion name. nullopt until a sample arrives — not implied by
  // is_open().
  virtual std::optional<std::string> current_motion() const = 0;
  // Motions the robot currently allows. nullopt until a sample arrives.
  virtual std::optional<std::vector<std::string>> available_transitions() const = 0;
  // Arrival time of the motion state sample behind the two readers above.
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  MotionController() = default;
  MotionController(const MotionController&) = delete;
  MotionController& operator=(const MotionController&) = delete;
};

class JointMotionPlanner {
 public:
  virtual ~JointMotionPlanner() = default;

  // Submit a plan request. kOk means sent; watch latest_state() for acceptance
  // and progress. Typical execute requires the planner to be Idle.
  virtual SessionError execute(const JointMotionPlanRequest& request) = 0;
  // Last planner sample. nullopt until one arrives.
  virtual std::optional<JointMotionPlanState> latest_state() const = 0;
  // Arrival time of the sample behind latest_state().
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  JointMotionPlanner() = default;
  JointMotionPlanner(const JointMotionPlanner&) = delete;
  JointMotionPlanner& operator=(const JointMotionPlanner&) = delete;
};

class JointStateReader {
 public:
  virtual ~JointStateReader() = default;
  // Last cached sample. nullopt means none received yet.
  virtual std::optional<JointState> latest() const = 0;
  // Fills out, reusing out.position/velocity/torque capacity. False if no sample yet.
  virtual bool latest(JointState& out) const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  JointStateReader() = default;
};

class MotorStateReader {
 public:
  virtual ~MotorStateReader() = default;
  // Last cached sample. nullopt means none received yet.
  virtual std::optional<JointState> latest() const = 0;
  // Fills out, reusing out.position/velocity/torque capacity. False if no sample yet.
  virtual bool latest(JointState& out) const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  MotorStateReader() = default;
};

class ImuReader {
 public:
  virtual ~ImuReader() = default;
  // Last cached sample. nullopt means none received yet.
  virtual std::optional<ImuInfo> latest() const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  ImuReader() = default;
};

class GamepadReader {
 public:
  virtual ~GamepadReader() = default;
  // Last cached sample. nullopt means none received yet.
  virtual std::optional<GamepadKeys> latest() const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  GamepadReader() = default;
};

class JointCommandClient {
 public:
  virtual ~JointCommandClient() = default;

  // Full-body joint command. Effective only while current motion is joint_bridge.
  // Keep sending at a control rate; one send does not hold the pose. Do not mix
  // with other high-rate joint or velocity writers.
  virtual SessionError send(const JointCommand& command) = 0;
  // Last command echo from the robot. nullopt until a sample arrives.
  virtual std::optional<JointCommand> latest_feedback() const = 0;
  // Arrival time of the sample behind latest_feedback().
  virtual std::optional<ReceiveTime> last_feedback_time() const = 0;

 protected:
  JointCommandClient() = default;
  JointCommandClient(const JointCommandClient&) = delete;
  JointCommandClient& operator=(const JointCommandClient&) = delete;
};

class BaseVelocityPublisher {
 public:
  virtual ~BaseVelocityPublisher() = default;
  // Body velocity command. Effective only in a locomotion motion that accepts it.
  // Refresh faster than every 500 ms while active; otherwise the robot releases
  // the external velocity override. Send zeros to stop. Do not mix with other
  // high-rate velocity writers or send while navigation takeover is active.
  virtual SessionError send(const BodyVelocityCommand& command) = 0;

 protected:
  BaseVelocityPublisher() = default;
};

class JointOverridePublisher {
 public:
  virtual ~JointOverridePublisher() = default;
  // Partial joint overlay. Effective only in motions that accept override
  // (for example lower_body_balance). When finished, send weight 0 before
  // close(). close() does not release the overlay. The locomotion node keeps
  // the last positive-weight sample until a weight 0 sample arrives.
  // kOk means the sample was sent, not that the robot applied it.
  virtual SessionError send(const JointOverrideCommand& command) = 0;

 protected:
  JointOverridePublisher() = default;
};

class LedController {
 public:
  virtual ~LedController() = default;
  // One-way lamp request. kOk means this process sent the request, not that the
  // robot applied it. There is no API to read the current lamp. Fault and safety
  // indications on the robot take priority and can hide this request until they
  // clear. The last requested color stays until you set another. If the robot is
  // still starting, the lamp may not change immediately.
  virtual SessionError set_mode(LedColor mode) = 0;

 protected:
  LedController() = default;
};

class PowerReader {
 public:
  virtual ~PowerReader() = default;
  // Last cached sample. nullopt means none received yet.
  virtual std::optional<PowerSystemInfo> latest() const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  PowerReader() = default;
};

class MotorDiagnosticsReader {
 public:
  virtual ~MotorDiagnosticsReader() = default;
  // Last cached sample. nullopt means none received yet.
  virtual std::optional<MotorDebug> latest() const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  MotorDiagnosticsReader() = default;
};

class NodeHeartbeatReader {
 public:
  virtual ~NodeHeartbeatReader() = default;
  // Last cached heartbeat. nullopt means none received yet. The sample stays
  // after the robot stops; is_open() says nothing about the robot either.
  virtual std::optional<Heartbeat> latest() const = 0;
  // A recent heartbeat arrival is the robot liveness signal. The robot
  // publishes once per second.
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  NodeHeartbeatReader() = default;
};

class MotorCommandReader {
 public:
  virtual ~MotorCommandReader() = default;
  // Last motor command the robot published (read-only). nullopt until a sample arrives.
  virtual std::optional<JointCommand> latest() const = 0;
  virtual std::optional<ReceiveTime> last_receive_time() const = 0;

 protected:
  MotorCommandReader() = default;
};

struct SessionCreateResult;

// All methods may be called from any thread, including close() while other
// threads read or send; those calls then return nullopt or kSessionClosed.
// Do not destroy the Robot while another thread is still using it.
class Robot {
 public:
  virtual ~Robot() = default;

  // Creates local DDS resources only; this is not a robot handshake.
  // On failure the result contains no session and reports the local error.
  // Confirm robot availability by reading a heartbeat or required state sample.
  static SessionCreateResult create(const SessionOptions& options);

  // Tears down this session. Does not release a joint override; send
  // weight 0 on joint_override() before calling this.
  // Idempotent. After this, is_open() is false. Other sessions in this
  // process that use the same domain id stay open.
  virtual void close() noexcept = 0;
  // True only while this process owns a valid local DDS session. Does not imply
  // a robot is present or publishing.
  virtual bool is_open() const noexcept = 0;

  virtual MotionController& motion() = 0;
  virtual const MotionController& motion() const = 0;
  virtual JointMotionPlanner& joint_motion_planner() = 0;
  virtual const JointMotionPlanner& joint_motion_planner() const = 0;
  virtual JointStateReader& joint_state() = 0;
  virtual const JointStateReader& joint_state() const = 0;
  virtual MotorStateReader& motor_state() = 0;
  virtual const MotorStateReader& motor_state() const = 0;
  virtual ImuReader& imu() = 0;
  virtual const ImuReader& imu() const = 0;
  virtual GamepadReader& gamepad() = 0;
  virtual const GamepadReader& gamepad() const = 0;
  virtual JointCommandClient& joint_command() = 0;
  virtual const JointCommandClient& joint_command() const = 0;
  virtual BaseVelocityPublisher& base_velocity() = 0;
  virtual const BaseVelocityPublisher& base_velocity() const = 0;
  virtual JointOverridePublisher& joint_override() = 0;
  virtual const JointOverridePublisher& joint_override() const = 0;
  virtual LedController& led() = 0;
  virtual const LedController& led() const = 0;
  virtual PowerReader& power() = 0;
  virtual const PowerReader& power() const = 0;
  virtual MotorDiagnosticsReader& motor_diagnostics() = 0;
  virtual const MotorDiagnosticsReader& motor_diagnostics() const = 0;
  virtual NodeHeartbeatReader& node_heartbeat() = 0;
  virtual const NodeHeartbeatReader& node_heartbeat() const = 0;
  virtual MotorCommandReader& motor_command() = 0;
  virtual const MotorCommandReader& motor_command() const = 0;

 protected:
  Robot() = default;
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
};

struct SessionCreateResult {
  std::unique_ptr<Robot> session;
  SessionError error{SessionError::kOk};

  explicit operator bool() const noexcept { return session != nullptr && error == SessionError::kOk; }
};

}  // namespace engineai_sdk
