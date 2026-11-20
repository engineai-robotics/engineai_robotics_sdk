// Safety: joint_bridge commands the joints directly. The standing pose in this
// mode is not necessarily stable, so hoist the robot before use. Both feet
// must be on the ground before entering walk.
//
// joint_bridge_stand_and_walk: stand and walk inside joint_bridge.
// Stand uses quintic interpolation. Walk sends JointCommand from an MNN policy.
//
//   ./joint_bridge_stand_and_walk --model pm01 --dry-run
//   ./joint_bridge_stand_and_walk --domain-id 0 --model t800pro --enable-low-level
//
// Does not change the motion. Enter pd_stand, hold the standing pose, then
// switch to joint_bridge before starting. The program keeps that pose.
//
// Gamepad, rising edge:
//   RB+A stand. LB+B walk (only after stand has finished).
// Sticks apply only while walking. A stale gamepad, joint, or IMU sample during
// walk feeds a zero stick command through the same filter, and the policy keeps
// running. Hold and stand keep commanding their pose through stale samples.
// This demo does not request passive.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <MNN/Interpreter.hpp>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/yaml_util.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

enum class Mode { kHold, kStand, kWalk };

struct ModelConfig {
  std::string model;
  std::string directory;
  std::vector<std::string> joint_names;
  std::vector<double> stand_position;
  std::vector<double> stand_kp;
  std::vector<double> stand_kd;
  double stand_duration = 0.0;
  double stand_bias_threshold = 1.0;

  std::string policy_path;
  int num_observations = 0;
  int num_actions = 0;
  int obs_steps = 0;
  double control_dt = 0.0;
  double observation_clip = 0.0;
  double action_clip = 0.0;
  double scale_lin = 0.0;
  double scale_ang = 0.0;
  double scale_dof_pos = 0.0;
  double scale_dof_vel = 0.0;
  double scale_quat = 0.0;
  std::vector<int> active_index;
  std::vector<double> default_q;
  std::vector<double> walk_kp;
  std::vector<double> walk_kd;
  std::vector<double> action_scale;
  bool command_lpf = false;
  double command_sample_hz = 0.0;
  double command_cutoff_hz = 0.0;
  Eigen::Vector3d command_scale_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d command_scale_neg = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_install_bias = Eigen::Vector3d::Zero();
  std::optional<double> transition_time;
  std::vector<int> disable_transition_index;
};

struct PolicyNet {
  MNN::Interpreter* net = nullptr;
  MNN::Session* session = nullptr;
  MNN::Tensor* input = nullptr;
  MNN::Tensor* output = nullptr;
  int input_n = 0;
  int output_n = 0;

  ~PolicyNet() {
    if (net != nullptr) {
      if (session != nullptr) {
        net->releaseSession(session);
      }
      net->releaseModel();
      MNN::Interpreter::destroy(net);
    }
  }

  PolicyNet() = default;
  PolicyNet(const PolicyNet&) = delete;
  PolicyNet& operator=(const PolicyNet&) = delete;

  void Load(const std::string& path, int expect_in, int expect_out) {
    net = MNN::Interpreter::createFromFile(path.c_str());
    if (net == nullptr) {
      throw std::runtime_error("failed to load MNN policy: " + path);
    }
    net->setSessionMode(MNN::Interpreter::Session_Release);
    MNN::ScheduleConfig config;
    config.numThread = 1;
    session = net->createSession(config);
    if (session == nullptr) {
      throw std::runtime_error("failed to create MNN session: " + path);
    }
    input = net->getSessionInput(session, nullptr);
    output = net->getSessionOutput(session, nullptr);
    if (input == nullptr || output == nullptr) {
      throw std::runtime_error("MNN policy is missing an input or output: " + path);
    }
    input_n = input->elementSize();
    output_n = output->elementSize();
    if (input_n != expect_in || output_n != expect_out) {
      throw std::runtime_error("MNN shape mismatch for " + path + ": input " + std::to_string(input_n) +
                               " expected " + std::to_string(expect_in) + ", output " + std::to_string(output_n) +
                               " expected " + std::to_string(expect_out));
    }
  }

  void Infer(const float* in, float* out) const {
    MNN::Tensor host_in(input, MNN::Tensor::CAFFE);
    std::memcpy(host_in.host<float>(), in, static_cast<size_t>(input_n) * sizeof(float));
    input->copyFromHostTensor(&host_in);
    net->runSession(session);
    MNN::Tensor host_out(output, MNN::Tensor::CAFFE);
    output->copyToHostTensor(&host_out);
    std::memcpy(out, host_out.host<float>(), static_cast<size_t>(output_n) * sizeof(float));
  }
};

struct CommandFilter {
  double alpha = 1.0;
  bool initialized = false;
  Eigen::Vector3d output = Eigen::Vector3d::Zero();

  explicit CommandFilter(double sample_hz, double cutoff_hz) {
    const double dt = 1.0 / sample_hz;
    const double rc = 1.0 / (2.0 * kPi * cutoff_hz);
    alpha = dt / (dt + rc);
  }

  Eigen::Vector3d Update(const Eigen::Vector3d& input) {
    if (!initialized) {
      output = input;
      initialized = true;
    } else {
      output = alpha * input + (1.0 - alpha) * output;
    }
    return output;
  }

  void Reset() { initialized = false; }
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --model pm01|t800|t800pro [--config-dir DIR] [--dry-run]\n"
            << "       --domain-id N [--interface IF] [--network-address ADDR] --enable-low-level [--timeout SEC]\n"
            << "  Requires motion joint_bridge and does not switch. Enter pd_stand, hold the\n"
            << "  standing pose, then switch to joint_bridge before starting.\n"
            << "  Copies under examples/config/<model>/custom_motion are the default.\n"
            << "  RB+A stands. LB+B walks after the stand finishes.\n"
            << "  A stale gamepad, joint, or IMU sample during walk clears the stick command.\n"
            << "  Hold and stand keep commanding through stale samples. Ctrl+C does not request passive.\n"
            << "  --dry-run loads the config and policy and does not open a session.\n";
}

const char* ModeName(Mode mode) {
  switch (mode) {
    case Mode::kHold:
      return "hold";
    case Mode::kStand:
      return "stand";
    case Mode::kWalk:
      return "walk";
  }
  return "unknown";
}

std::vector<std::string> LoadNames(const YAML::Node& node) {
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("expected a YAML sequence of names");
  }
  std::vector<std::string> names;
  for (const auto& item : node) {
    names.push_back(item.as<std::string>());
  }
  return names;
}

int FindJoint(const std::vector<std::string>& names, const std::string& name) {
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    throw std::runtime_error("joint is not in joints.yaml: " + name);
  }
  return static_cast<int>(it - names.begin());
}

Eigen::Vector3d LoadVec3(const YAML::Node& node, const char* key) {
  std::vector<double> values;
  robot_session_examples::FlattenDoubles(node[key], &values);
  if (values.size() != 3) {
    throw std::runtime_error(std::string(key) + " must have 3 values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

void RequireSize(const char* key, const std::vector<double>& values, size_t expect) {
  if (values.size() != expect) {
    throw std::runtime_error(std::string(key) + " has " + std::to_string(values.size()) + " values, expected " +
                             std::to_string(expect));
  }
}

ModelConfig LoadModel(const std::string& config_dir, const std::string& model) {
  ModelConfig cfg;
  cfg.model = model;
  cfg.directory = config_dir + "/" + model + "/custom_motion";
  if (!robot_session_examples::FileExists(cfg.directory + "/joints.yaml")) {
    throw std::runtime_error("config directory not found: " + cfg.directory);
  }

  const YAML::Node joints = YAML::LoadFile(cfg.directory + "/joints.yaml");
  cfg.joint_names = LoadNames(joints["joints"]);
  if (cfg.joint_names.empty()) {
    throw std::runtime_error("joints.yaml is empty");
  }
  const size_t n = cfg.joint_names.size();

  const YAML::Node stand = YAML::LoadFile(cfg.directory + "/pd_stand.yaml");
  robot_session_examples::FlattenDoubles(stand["desired_joint_position"], &cfg.stand_position);
  robot_session_examples::FlattenDoubles(stand["stiffness"], &cfg.stand_kp);
  robot_session_examples::FlattenDoubles(stand["damping"], &cfg.stand_kd);
  RequireSize("desired_joint_position", cfg.stand_position, n);
  RequireSize("stiffness", cfg.stand_kp, n);
  RequireSize("damping", cfg.stand_kd, n);
  cfg.stand_duration = stand["duration"].as<double>();
  if (cfg.stand_duration <= 0.0) {
    throw std::runtime_error("pd_stand duration must be > 0");
  }
  if (stand["initial_joint_position_bias_threshold"]) {
    cfg.stand_bias_threshold = stand["initial_joint_position_bias_threshold"].as<double>();
  }

  const YAML::Node walk = YAML::LoadFile(cfg.directory + "/rl_walking.yaml");
  cfg.policy_path = cfg.directory + "/" + walk["policy_file"].as<std::string>();
  cfg.num_observations = walk["num_observations"].as<int>();
  cfg.obs_steps = walk["num_include_obs_steps"].as<int>();
  cfg.control_dt = walk["control_dt"].as<double>();
  cfg.observation_clip = walk["observation_clip"].as<double>();
  cfg.action_clip = walk["action_clip"].as<double>();
  cfg.scale_lin = walk["observation_scale_linear_vel"].as<double>();
  cfg.scale_ang = walk["observation_scale_angular_vel"].as<double>();
  cfg.scale_dof_pos = walk["observation_scale_dof_pos"].as<double>();
  cfg.scale_dof_vel = walk["observation_scale_dof_vel"].as<double>();
  cfg.scale_quat = walk["observation_scale_quat"].as<double>();
  if (cfg.obs_steps <= 0 || cfg.control_dt <= 0.0) {
    throw std::runtime_error("observation steps and control_dt must be > 0");
  }
  const auto active = LoadNames(walk["active_joint_names"]);
  cfg.num_actions = static_cast<int>(active.size());
  if (cfg.num_observations != cfg.num_actions * 3 + 6) {
    throw std::runtime_error("num_observations does not match active joints * 3 + 6");
  }
  for (const auto& name : active) {
    cfg.active_index.push_back(FindJoint(cfg.joint_names, name));
  }
  robot_session_examples::FlattenDoubles(walk["default_joint_q"], &cfg.default_q);
  robot_session_examples::FlattenDoubles(walk["joint_kp"], &cfg.walk_kp);
  robot_session_examples::FlattenDoubles(walk["joint_kd"], &cfg.walk_kd);
  robot_session_examples::FlattenDoubles(walk["action_scale"], &cfg.action_scale);
  RequireSize("default_joint_q", cfg.default_q, n);
  RequireSize("joint_kp", cfg.walk_kp, n);
  RequireSize("joint_kd", cfg.walk_kd, n);
  RequireSize("action_scale", cfg.action_scale, static_cast<size_t>(cfg.num_actions));

  cfg.command_lpf = walk["enable_remote_command_lpf"].as<bool>();
  cfg.command_sample_hz = walk["remote_command_sampling_frequency"].as<double>();
  cfg.command_cutoff_hz = walk["remote_command_cut_off_frequency"].as<double>();
  if (cfg.command_lpf && (cfg.command_sample_hz <= 0.0 || cfg.command_cutoff_hz <= 0.0)) {
    throw std::runtime_error("command filter frequencies must be > 0");
  }
  cfg.command_scale_pos = LoadVec3(walk, "command_scale_pos");
  cfg.command_scale_neg = LoadVec3(walk, "command_scale_neg");
  if (walk["imu_install_bias"]) {
    cfg.imu_install_bias = LoadVec3(walk, "imu_install_bias");
  }
  if (walk["transition_time"]) {
    cfg.transition_time = walk["transition_time"].as<double>();
  }
  if (walk["disable_transition_joints"]) {
    for (const auto& name : LoadNames(walk["disable_transition_joints"])) {
      cfg.disable_transition_index.push_back(FindJoint(cfg.joint_names, name));
    }
  }
  if (!robot_session_examples::FileExists(cfg.policy_path)) {
    throw std::runtime_error("policy file not found: " + cfg.policy_path);
  }
  return cfg;
}

Eigen::Matrix3d RpyToMatrix(const Eigen::Vector3d& rpy) {
  const double c0 = std::cos(rpy.x());
  const double c1 = std::cos(rpy.y());
  const double c2 = std::cos(rpy.z());
  const double s0 = std::sin(rpy.x());
  const double s1 = std::sin(rpy.y());
  const double s2 = std::sin(rpy.z());
  const double c2_s1 = c2 * s1;
  const double s2_s1 = s2 * s1;
  Eigen::Matrix3d rotation;
  rotation << c2 * c1, c2_s1 * s0 - s2 * c0, c2_s1 * c0 + s2 * s0, s2 * c1, s2_s1 * s0 + c2 * c0,
      s2_s1 * c0 - c2 * s0, -s1, c1 * s0, c1 * c0;
  return rotation;
}

void QuinticInterpolate(const std::vector<double>& start, const std::vector<double>& end, double period, double phase,
                        std::vector<double>* position, std::vector<double>* velocity) {
  const double t = std::clamp(phase / period, 0.0, 1.0);
  const double r = ((6.0 * t - 15.0) * t + 10.0) * t * t * t;
  const double rd = ((30.0 * t - 60.0) * t + 30.0) * t * t;
  position->resize(start.size());
  velocity->resize(start.size());
  for (size_t i = 0; i < start.size(); ++i) {
    const double delta = end[i] - start[i];
    (*position)[i] = start[i] + delta * r;
    (*velocity)[i] = delta * (rd / period);
  }
}

engineai_sdk::JointCommand MakeCommand(const std::vector<double>& position, const std::vector<double>& velocity,
                                       const std::vector<double>& kp, const std::vector<double>& kd,
                                       engineai_sdk::ParallelParserType parser) {
  engineai_sdk::JointCommand command;
  command.position = position;
  command.velocity = velocity;
  command.feed_forward_torque.assign(position.size(), 0.0);
  command.torque.assign(position.size(), 0.0);
  command.stiffness = kp;
  command.damping = kd;
  command.parallel_parser_type = parser;
  return command;
}

bool Pressed(const engineai_sdk::GamepadKeys& pad, engineai_sdk::GamepadButton button) {
  return pad.digital_states[static_cast<size_t>(button)] != 0;
}

bool StandChord(const engineai_sdk::GamepadKeys& pad) {
  using B = engineai_sdk::GamepadButton;
  return pad.hardware_connected && Pressed(pad, B::kRb) && Pressed(pad, B::kA);
}

bool WalkChord(const engineai_sdk::GamepadKeys& pad) {
  using B = engineai_sdk::GamepadButton;
  return pad.hardware_connected && Pressed(pad, B::kLb) && Pressed(pad, B::kB);
}

bool Rising(bool down, bool* previous) {
  const bool edge = down && !*previous;
  *previous = down;
  return edge;
}

bool RequireJointBridge(engineai_sdk::Robot* session, int timeout_sec) {
  if (!robot_session_examples::WaitForMotionSample(session, timeout_sec * 1000)) {
    robot_session_examples::PrintNoRobotDataHint();
    return false;
  }
  if (robot_session_examples::MotionIs(*session, "joint_bridge")) {
    std::cout << "motion=joint_bridge\n";
    return true;
  }
  std::cerr << "current motion=" << robot_session_examples::MotionOrNone(*session)
            << ", required=joint_bridge\n"
            << "Enter pd_stand, hold the standing pose, then switch to joint_bridge "
            << "before starting. This program does not change the motion.\n";
  return false;
}

struct WalkRuntime {
  bool first = true;
  double time = 0.0;
  std::vector<double> history;
  std::vector<double> action;
  std::vector<double> initial_q;
  std::vector<double> initial_kp;
  std::vector<double> initial_kd;
  CommandFilter filter;
  bool transition = false;

  explicit WalkRuntime(const ModelConfig& cfg)
      : filter(cfg.command_sample_hz > 0.0 ? cfg.command_sample_hz : 1.0,
               cfg.command_cutoff_hz > 0.0 ? cfg.command_cutoff_hz : 1.0) {}

  void Reset(const ModelConfig& cfg, const std::vector<double>& q, const std::vector<double>& kp,
             const std::vector<double>& kd) {
    first = true;
    time = 0.0;
    history.assign(static_cast<size_t>(cfg.num_observations * cfg.obs_steps), 0.0);
    action.assign(static_cast<size_t>(cfg.num_actions), 0.0);
    initial_q = q;
    initial_kp = kp;
    initial_kd = kd;
    filter.Reset();
    transition = cfg.transition_time.has_value() && *cfg.transition_time > cfg.control_dt;
  }
};

bool BuildObservation(const ModelConfig& cfg, const engineai_sdk::JointState& joint, const engineai_sdk::ImuInfo& imu,
                      const Eigen::Vector3d& command, WalkRuntime* walk, std::vector<float>* net_input) {
  const int n = cfg.num_actions;
  const int obs_n = cfg.num_observations;
  Eigen::VectorXd observation = Eigen::VectorXd::Zero(obs_n);
  for (int i = 0; i < n; ++i) {
    const int joint_index = cfg.active_index[static_cast<size_t>(i)];
    observation[i] = (joint.position[static_cast<size_t>(joint_index)] - cfg.default_q[static_cast<size_t>(joint_index)]) *
                     cfg.scale_dof_pos;
    observation[n + i] = joint.velocity[static_cast<size_t>(joint_index)] * cfg.scale_dof_vel;
    observation[2 * n + i] = walk->action[static_cast<size_t>(i)];
  }

  const Eigen::Matrix3d install = RpyToMatrix(cfg.imu_install_bias);
  const Eigen::Quaterniond quaternion(imu.quaternion.w, imu.quaternion.x, imu.quaternion.y, imu.quaternion.z);
  const Eigen::Matrix3d local = quaternion.normalized().toRotationMatrix();
  const Eigen::Matrix3d real = local * install.transpose();
  const Eigen::Vector3d angular = real.transpose() * local *
                                  Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  const Eigen::Vector3d gravity = -real.transpose() * Eigen::Vector3d::UnitZ();
  observation.segment(3 * n, 3) = angular * cfg.scale_ang;
  observation.segment(3 * n + 3, 3) = gravity * cfg.scale_quat;
  observation = observation.cwiseMax(-cfg.observation_clip).cwiseMin(cfg.observation_clip);

  const int step = obs_n;
  if (walk->first) {
    for (int col = 0; col < cfg.obs_steps; ++col) {
      for (int row = 0; row < obs_n; ++row) {
        walk->history[static_cast<size_t>(col * step + row)] = observation[row];
      }
    }
    walk->first = false;
  } else {
    for (int i = 0; i < step * (cfg.obs_steps - 1); ++i) {
      walk->history[static_cast<size_t>(i)] = walk->history[static_cast<size_t>(i + step)];
    }
    for (int row = 0; row < obs_n; ++row) {
      walk->history[static_cast<size_t>((cfg.obs_steps - 1) * step + row)] = observation[row];
    }
  }

  const Eigen::Vector3d command_obs(command.x() * cfg.scale_lin, command.y() * cfg.scale_lin, command.z() * cfg.scale_ang);
  net_input->resize(walk->history.size() + 3);
  for (size_t i = 0; i < walk->history.size(); ++i) {
    (*net_input)[i] = static_cast<float>(walk->history[i]);
  }
  (*net_input)[walk->history.size()] = static_cast<float>(command_obs.x());
  (*net_input)[walk->history.size() + 1] = static_cast<float>(command_obs.y());
  (*net_input)[walk->history.size() + 2] = static_cast<float>(command_obs.z());
  return true;
}

Eigen::Vector3d StickCommand(const ModelConfig& cfg, const engineai_sdk::GamepadKeys& pad) {
  using A = engineai_sdk::GamepadAxis;
  const double x = pad.analog_states[static_cast<size_t>(A::kLeftStickX)];
  const double y = pad.analog_states[static_cast<size_t>(A::kLeftStickY)];
  const double yaw = pad.analog_states[static_cast<size_t>(A::kRightStickY)];
  Eigen::Vector3d command;
  command.x() = x >= 0.0 ? x * cfg.command_scale_pos.x() : x * cfg.command_scale_neg.x();
  command.y() = y >= 0.0 ? y * cfg.command_scale_pos.y() : y * cfg.command_scale_neg.y();
  command.z() = yaw >= 0.0 ? yaw * cfg.command_scale_pos.z() : yaw * cfg.command_scale_neg.z();
  return command;
}

bool SameSize(const engineai_sdk::JointState& joint, size_t n) {
  return joint.position.size() == n && joint.velocity.size() == n;
}

std::string BiasReport(const ModelConfig& cfg, const std::vector<double>& measured) {
  std::string report;
  for (size_t i = 0; i < measured.size(); ++i) {
    const double bias = measured[i] - cfg.stand_position[i];
    if (std::abs(bias) > cfg.stand_bias_threshold) {
      report += cfg.joint_names[i] + " bias=" + std::to_string(bias) + "\n";
    }
  }
  return report;
}

}  // namespace

int main(int argc, char** argv) {
  if (robot_session_examples::WantsHelp(argc, argv)) {
    PrintUsage(argv[0]);
    return 0;
  }

  std::string model;
  std::string config_dir = "examples/config";
  bool dry_run = false;
  bool enable = false;
  int timeout_sec = 8;
  engineai_sdk::SessionOptions options;
  bool have_domain = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--domain-id" && i + 1 < argc) {
      int domain_id = 0;
      if (!robot_session_examples::ReadStrictInt(argv[++i], &domain_id, "domain-id")) {
        return 2;
      }
      options.dds_domain_id = domain_id;
      have_domain = true;
    } else if (arg == "--interface" && i + 1 < argc) {
      options.network_interface_name = argv[++i];
    } else if (arg == "--network-address" && i + 1 < argc) {
      options.network_address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--config-dir" && i + 1 < argc) {
      config_dir = argv[++i];
    } else if (arg == "--timeout" && i + 1 < argc) {
      if (!robot_session_examples::ReadStrictInt(argv[++i], &timeout_sec, "timeout")) {
        return 2;
      }
    } else if (arg == "--dry-run") {
      dry_run = true;
    } else if (arg == "--enable-low-level") {
      enable = true;
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (model != "pm01" && model != "t800" && model != "t800pro") {
    std::cerr << "--model must be pm01, t800, or t800pro\n";
    return 2;
  }
  if (timeout_sec <= 0) {
    std::cerr << "timeout must be > 0\n";
    return 2;
  }
  if (!dry_run && !enable) {
    std::cerr << "refusing joint commands without --enable-low-level\n";
    return 2;
  }
  if (!dry_run && (!have_domain || options.dds_domain_id < 0)) {
    std::cerr << "Missing required --domain-id <int> (use the robot ROS_DOMAIN_ID; 0 if unset)\n";
    return 2;
  }

  ModelConfig cfg;
  try {
    cfg = LoadModel(config_dir, model);
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  PolicyNet policy;
  try {
    policy.Load(cfg.policy_path, cfg.num_observations * cfg.obs_steps + 3, cfg.num_actions);
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  std::cout << "model=" << cfg.model << " joints=" << cfg.joint_names.size() << " actions=" << cfg.num_actions
            << " obs=" << cfg.num_observations << "x" << cfg.obs_steps << " policy=" << cfg.policy_path << "\n";
  if (dry_run) {
    std::vector<float> input(static_cast<size_t>(policy.input_n), 0.0f);
    std::vector<float> output(static_cast<size_t>(policy.output_n), 0.0f);
    policy.Infer(input.data(), output.data());
    std::cout << "dry-run inference ok output0=" << output[0] << "\n";
    return 0;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!RequireJointBridge(session.get(), timeout_sec)) {
    session->close();
    return 1;
  }
  engineai_sdk::JointState joint;
  if (!robot_session_examples::WaitJointState(session.get(), &joint, timeout_sec * 1000) ||
      !SameSize(joint, cfg.joint_names.size())) {
    std::cerr << "joint_state missing or size " << joint.position.size() << " != " << cfg.joint_names.size() << "\n";
    session->close();
    return 1;
  }
  const auto imu_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  while (!session->imu().latest() && std::chrono::steady_clock::now() < imu_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto initial_imu = session->imu().latest();
  if (!initial_imu) {
    std::cerr << "imu has no sample\n";
    session->close();
    return 1;
  }
  engineai_sdk::ImuInfo imu_sample = *initial_imu;
  const auto pad_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  while (!session->gamepad().latest() && std::chrono::steady_clock::now() < pad_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!session->gamepad().latest()) {
    std::cerr << "gamepad has no sample\n";
    session->close();
    return 1;
  }

  robot_session_examples::CatchSigInt();
  Mode mode = Mode::kHold;
  bool stand_done = false;
  bool stand_edge = false;
  bool walk_edge = false;
  bool joint_ready = true;
  bool imu_ready = true;
  bool gamepad_ready = true;
  std::vector<double> hold_q = joint.position;
  std::vector<double> stand_q0 = joint.position;
  auto stand_t0 = std::chrono::steady_clock::now();
  std::vector<double> last_q = hold_q;
  std::vector<double> last_kp = cfg.stand_kp;
  std::vector<double> last_kd = cfg.stand_kd;
  WalkRuntime walk(cfg);
  auto next_tick = std::chrono::steady_clock::now();
  auto next_log = next_tick;
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(cfg.control_dt));

  auto send = [&](const engineai_sdk::JointCommand& command) {
    const auto err = session->joint_command().send(command);
    if (err != engineai_sdk::SessionError::kOk) {
      std::cerr << "joint_command.send -> " << robot_session_examples::ErrName(err) << "\n";
      return false;
    }
    last_q = command.position;
    last_kp = command.stiffness;
    last_kd = command.damping;
    return true;
  };

  std::cout << "keys: RB+A stand, LB+B walk after stand\n";
  while (!robot_session_examples::g_stop) {
    next_tick += period;
    if (!robot_session_examples::MotionIs(*session, "joint_bridge")) {
      std::cerr << "motion left joint_bridge, current=" << robot_session_examples::MotionOrNone(*session) << "\n";
      break;
    }
    // Check arrival time before reading: latest() keeps the last sample after the robot stops publishing.
    using robot_session_examples::IsFresh;
    using robot_session_examples::kStateMaxAge;
    const bool joint_fresh = IsFresh(session->joint_state().last_receive_time(), kStateMaxAge);
    engineai_sdk::JointState incoming;
    const bool got_joint = joint_fresh && session->joint_state().latest(incoming);
    bool joint_ok = false;
    if (got_joint && SameSize(incoming, cfg.joint_names.size())) {
      joint = std::move(incoming);
      joint_ok = true;
    } else if (got_joint) {
      std::cerr << "joint_state length " << incoming.position.size() << " != " << cfg.joint_names.size()
                << ", stop\n";
      session->close();
      return 1;
    }
    const bool imu_fresh = IsFresh(session->imu().last_receive_time(), kStateMaxAge);
    const bool pad_fresh = IsFresh(session->gamepad().last_receive_time(), kStateMaxAge);
    const auto imu = session->imu().latest();
    const auto pad = session->gamepad().latest();
    if (imu) {
      imu_sample = *imu;
    }
    const bool imu_ok = imu_fresh && imu.has_value();
    const bool pad_ok = pad_fresh && pad.has_value();
    if (joint_ok != joint_ready) {
      if (joint_ok) {
        std::cout << "joint_state restored\n";
      } else if (mode == Mode::kWalk) {
        std::cerr << "joint_state lost or stale, walk command cleared\n";
      } else {
        std::cerr << "joint_state lost or stale, holding pose\n";
      }
    }
    if (imu_ok != imu_ready) {
      if (imu_ok) {
        std::cout << "imu sample restored\n";
      } else if (mode == Mode::kWalk) {
        std::cerr << "imu sample lost or stale, walk command cleared\n";
      } else {
        std::cerr << "imu sample lost or stale, holding pose\n";
      }
    }
    joint_ready = joint_ok;
    imu_ready = imu_ok;
    if (pad_ok != gamepad_ready) {
      if (pad_ok) {
        std::cout << "gamepad sample restored\n";
      } else if (mode == Mode::kWalk) {
        std::cerr << "gamepad sample lost or stale, walk command cleared\n";
      } else {
        std::cerr << "gamepad sample lost or stale, holding pose\n";
      }
      gamepad_ready = pad_ok;
    }

    if (pad_ok && Rising(StandChord(*pad), &stand_edge)) {
      if (!joint_ok) {
        std::cerr << "stand refused, joint_state lost or stale\n";
      } else {
        const std::string bias = BiasReport(cfg, joint.position);
        if (!bias.empty()) {
          std::cerr << "stand refused, joint bias above " << cfg.stand_bias_threshold << "\n" << bias;
        } else {
          mode = Mode::kStand;
          stand_done = false;
          stand_q0 = joint.position;
          stand_t0 = std::chrono::steady_clock::now();
          std::cout << "state=stand\n";
        }
      }
    }
    if (pad_ok && Rising(WalkChord(*pad), &walk_edge)) {
      if (!stand_done) {
        std::cerr << "walk refused until stand finishes\n";
      } else if (!joint_ok || !imu_ok) {
        std::cerr << "walk refused, joint or imu lost or stale\n";
      } else {
        mode = Mode::kWalk;
        walk.Reset(cfg, last_q, last_kp, last_kd);
        std::cout << "state=walk\n";
      }
    }

    engineai_sdk::JointCommand command;
    if (mode == Mode::kHold) {
      const std::vector<double> zero(hold_q.size(), 0.0);
      command = MakeCommand(hold_q, zero, cfg.stand_kp, cfg.stand_kd, engineai_sdk::ParallelParserType::kClassic);
    } else if (mode == Mode::kStand) {
      const double phase = std::chrono::duration<double>(std::chrono::steady_clock::now() - stand_t0).count();
      std::vector<double> position;
      std::vector<double> velocity;
      QuinticInterpolate(stand_q0, cfg.stand_position, cfg.stand_duration, phase, &position, &velocity);
      command = MakeCommand(position, velocity, cfg.stand_kp, cfg.stand_kd, engineai_sdk::ParallelParserType::kClassic);
      if (phase >= cfg.stand_duration && !stand_done) {
        stand_done = true;
        std::cout << "stand complete\n";
      }
    } else {
      Eigen::Vector3d command_vel = Eigen::Vector3d::Zero();
      if (pad_ok && joint_ok && imu_ok) {
        command_vel = StickCommand(cfg, *pad);
      }
      if (cfg.command_lpf) {
        command_vel = walk.filter.Update(command_vel);
      }
      // latest() keeps the last sample, so a stale joint or IMU still feeds the policy.
      std::vector<float> net_input;
      BuildObservation(cfg, joint, imu_sample, command_vel, &walk, &net_input);
      std::vector<float> net_output(static_cast<size_t>(cfg.num_actions), 0.0f);
      policy.Infer(net_input.data(), net_output.data());
      for (int i = 0; i < cfg.num_actions; ++i) {
        walk.action[static_cast<size_t>(i)] =
            std::clamp(static_cast<double>(net_output[static_cast<size_t>(i)]), -cfg.action_clip, cfg.action_clip);
      }
      std::vector<double> position = cfg.default_q;
      for (int i = 0; i < cfg.num_actions; ++i) {
        const size_t joint_index = static_cast<size_t>(cfg.active_index[static_cast<size_t>(i)]);
        position[joint_index] += walk.action[static_cast<size_t>(i)] * cfg.action_scale[static_cast<size_t>(i)];
      }
      std::vector<double> kp = cfg.walk_kp;
      std::vector<double> kd = cfg.walk_kd;
      if (walk.transition && walk.time < *cfg.transition_time) {
        const double ratio = walk.time / *cfg.transition_time;
        const std::vector<double> policy_q = position;
        for (size_t i = 0; i < position.size(); ++i) {
          position[i] = ratio * policy_q[i] + (1.0 - ratio) * walk.initial_q[i];
          kp[i] = ratio * cfg.walk_kp[i] + (1.0 - ratio) * walk.initial_kp[i];
          kd[i] = ratio * cfg.walk_kd[i] + (1.0 - ratio) * walk.initial_kd[i];
        }
        for (int index : cfg.disable_transition_index) {
          const size_t i = static_cast<size_t>(index);
          position[i] = policy_q[i];
          kp[i] = cfg.walk_kp[i];
          kd[i] = cfg.walk_kd[i];
        }
      }
      const std::vector<double> zero(position.size(), 0.0);
      command = MakeCommand(position, zero, kp, kd, engineai_sdk::ParallelParserType::kRl);
      walk.time += cfg.control_dt;
    }

    if (!send(command)) {
      session->close();
      return 1;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_log) {
      std::cout << "state=" << ModeName(mode) << " stand_done=" << (stand_done ? 1 : 0) << " motion="
                << robot_session_examples::MotionOrNone(*session);
      if (!joint_ok) {
        std::cout << " joint=stale";
      }
      if (!imu_ok) {
        std::cout << " imu=stale";
      }
      if (!pad_ok) {
        std::cout << " gamepad=stale";
      }
      std::cout << "\n";
      next_log = now + std::chrono::milliseconds(500);
    }
    std::this_thread::sleep_until(next_tick);
  }

  session->close();
  return 0;
}
