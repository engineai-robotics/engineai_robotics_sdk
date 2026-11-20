// upper_body_override: quintic upper-body override in lower_body_balance
// Corresponds to interface_example upper_joint_override_example.py
//
//   ./upper_body_override --domain-id 0 --config examples/config/pm01/upper_joint_override.yaml

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/yaml_util.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR] --config PATH.yaml [--seconds N]\n"
            << "  --seconds 0 means run until Ctrl+C\n";
}

void QuinticSample(const std::vector<double>& q0, const std::vector<double>& qf, double duration,
                   double t, std::vector<double>* pos) {
  t = std::clamp(t, 0.0, duration);
  const double T = duration;
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;
  pos->resize(q0.size());
  for (size_t i = 0; i < q0.size(); ++i) {
    const double dq = qf[i] - q0[i];
    const double a3 = (20.0 * dq) / (2.0 * T3);
    const double a4 = (30.0 * (q0[i] - qf[i])) / (2.0 * T4);
    const double a5 = (12.0 * dq) / (2.0 * T5);
    (*pos)[i] = q0[i] + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (robot_session_examples::WantsHelp(argc, argv)) {
    PrintUsage(argv[0]);
    return 0;
  }

  engineai_sdk::SessionOptions options;
  std::vector<std::string> rest;
  if (!robot_session_examples::ParseSessionArgs(argc, argv, &options, &rest)) {
    PrintUsage(argv[0]);
    return 2;
  }

  std::string config_path;
  int seconds = 0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--config" && i + 1 < rest.size()) {
      config_path = rest[++i];
    } else if (arg == "--seconds" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &seconds, "seconds")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (config_path.empty()) {
    std::cerr << "Missing required --config PATH.yaml\n";
    PrintUsage(argv[0]);
    return 2;
  }
  if (!robot_session_examples::FileExists(config_path)) {
    std::cerr << "config not found: " << config_path << "\n";
    return 1;
  }
  if (seconds < 0) {
    std::cerr << "seconds must be >= 0\n";
    return 2;
  }

  double freq = 100.0;
  double duration = 5.0;
  std::vector<int32_t> joint_indices;
  std::vector<double> start_pos;
  std::vector<double> end_pos;
  std::vector<double> velocity;
  std::vector<double> feed_forward_torque;
  std::vector<double> torque;
  std::vector<double> stiffness;
  std::vector<double> damping;
  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    freq = root["timer_freq"].as<double>();
    duration = root["duration"].as<double>();
    robot_session_examples::FlattenInts(root["joint_indices"], &joint_indices);
    robot_session_examples::FlattenDoubles(root["start_pos"], &start_pos);
    robot_session_examples::FlattenDoubles(root["end_pos"], &end_pos);
    robot_session_examples::FlattenDoubles(root["velocity"], &velocity);
    robot_session_examples::FlattenDoubles(root["feed_forward_torque"], &feed_forward_torque);
    robot_session_examples::FlattenDoubles(root["torque"], &torque);
    robot_session_examples::FlattenDoubles(root["stiffness"], &stiffness);
    robot_session_examples::FlattenDoubles(root["damping"], &damping);
  } catch (const std::exception& e) {
    std::cerr << "failed to parse config: " << e.what() << "\n";
    return 1;
  }

  const size_t n = joint_indices.size();
  if (n == 0 || start_pos.size() != n || end_pos.size() != n || velocity.size() != n ||
      feed_forward_torque.size() != n || torque.size() != n || stiffness.size() != n ||
      damping.size() != n) {
    std::cerr << "config vector length mismatch vs joint_indices n=" << n << "\n";
    return 1;
  }
  if (freq <= 0.0 || duration <= 0.0) {
    std::cerr << "timer_freq and duration must be > 0\n";
    return 1;
  }

  const int steps = std::max(2, static_cast<int>(duration * freq));
  std::vector<std::vector<double>> trajectory(static_cast<size_t>(steps));
  for (int s = 0; s < steps; ++s) {
    const double t = duration * static_cast<double>(s) / static_cast<double>(steps - 1);
    QuinticSample(start_pos, end_pos, duration, t, &trajectory[static_cast<size_t>(s)]);
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "lower_body_balance")) {
    session->close();
    return 1;
  }

  engineai_sdk::JointOverrideCommand override_cmd;
  override_cmd.weight = 1.0;
  override_cmd.joint_indices = joint_indices;
  override_cmd.command.velocity = velocity;
  override_cmd.command.feed_forward_torque = feed_forward_torque;
  override_cmd.command.torque = torque;
  override_cmd.command.stiffness = stiffness;
  override_cmd.command.damping = damping;
  override_cmd.command.parallel_parser_type = engineai_sdk::ParallelParserType::kClassic;

  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::microseconds(static_cast<int>(1e6 / freq));
  const auto deadline = seconds == 0
                            ? std::chrono::steady_clock::time_point::max()
                            : std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  int idx = 0;
  int direction = 1;
  bool send_ok = true;
  bool sent = false;
  bool left_mode = false;
  while (send_ok && !left_mode && !robot_session_examples::g_stop &&
         std::chrono::steady_clock::now() < deadline) {
    if (!robot_session_examples::MotionIs(*session, "lower_body_balance")) {
      std::cerr << "left lower_body_balance, current="
                << robot_session_examples::MotionOrNone(*session) << "\n";
      left_mode = true;
      break;
    }
    override_cmd.command.position = trajectory[static_cast<size_t>(idx)];
    const auto err = session->joint_override().send(override_cmd);
    if (!robot_session_examples::CheckOk(err, "joint_override.send")) {
      send_ok = false;
      break;
    }
    sent = true;
    idx += direction;
    if (idx >= steps - 1) {
      idx = steps - 1;
      direction = -1;
    } else if (idx <= 0) {
      idx = 0;
      direction = 1;
    }
    std::this_thread::sleep_for(period);
  }

  bool released = true;
  if (sent) {
    engineai_sdk::JointOverrideCommand release;
    release.weight = 0.0;
    released = robot_session_examples::CheckOk(session->joint_override().send(release), "joint_override.release");
  }
  // Pause before close() so this call can still reach the robot.
  robot_session_examples::PauseBeforeClose();
  session->close();
  return send_ok && released && !left_mode ? 0 : 1;
}
