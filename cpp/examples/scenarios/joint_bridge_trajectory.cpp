// Safety: joint_bridge commands the joints directly. The standing pose in this
// mode is not necessarily stable, so hoist the robot before use. Both feet
// must be on the ground before entering walk.
//
// joint_bridge_trajectory: interpolate joints from YAML in joint_bridge.
// Refuses to send unless --enable-low-level is present.
// --max-delta refuses the whole trajectory when any joint move exceeds the limit.
//
//   ./joint_bridge_trajectory --domain-id 0 --enable-low-level --config examples/config/pm01/joint_test.yaml

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/yaml_util.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR] --enable-low-level --config PATH.yaml\n"
            << "       [--max-delta RAD]\n"
            << "  Requires motion joint_bridge. Without --max-delta, large targets are sent.\n"
            << "  With --max-delta, any joint whose total move exceeds the limit cancels the whole command.\n";
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
  bool enable = false;
  bool have_max_delta = false;
  double max_delta = 0.0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--enable-low-level") {
      enable = true;
    } else if (arg == "--config" && i + 1 < rest.size()) {
      config_path = rest[++i];
    } else if (arg == "--max-delta" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &max_delta, "max-delta")) {
        return 2;
      }
      have_max_delta = true;
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (!enable) {
    std::cerr << "refusing low-level joint command without --enable-low-level\n";
    return 2;
  }
  if (have_max_delta && max_delta < 0.0) {
    std::cerr << "max-delta must be >= 0\n";
    return 2;
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

  std::vector<double> target_positions;
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<int32_t> num_steps;
  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    robot_session_examples::FlattenDoubles(root["target_position"], &target_positions);
    robot_session_examples::FlattenDoubles(root["kp"], &kp);
    robot_session_examples::FlattenDoubles(root["kd"], &kd);
    robot_session_examples::FlattenInts(root["num_steps"], &num_steps);
  } catch (const std::exception& e) {
    std::cerr << "failed to parse config: " << e.what() << "\n";
    return 1;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "joint_bridge")) {
    session->close();
    return 1;
  }

  engineai_sdk::JointState joint;
  if (!robot_session_examples::WaitJointState(session.get(), &joint)) {
    std::cerr << "Timed out waiting for joint_state\n";
    session->close();
    return 1;
  }

  const size_t n = joint.position.size();
  if (target_positions.size() != n || kp.size() != n || kd.size() != n ||
      num_steps.size() != n) {
    std::cerr << "config/joint_state length mismatch: joints=" << n
              << " target=" << target_positions.size() << " kp=" << kp.size()
              << " kd=" << kd.size() << " num_steps=" << num_steps.size() << "\n";
    session->close();
    return 1;
  }
  if (have_max_delta) {
    double worst = 0.0;
    size_t worst_index = 0;
    for (size_t i = 0; i < n; ++i) {
      const double delta = std::abs(target_positions[i] - joint.position[i]);
      if (delta > worst) {
        worst = delta;
        worst_index = i;
      }
    }
    std::cout << "joints=" << n << " max_delta=" << worst << " at=" << worst_index << " limit=" << max_delta << "\n";
    if (worst > max_delta) {
      std::cerr << "refusing command: joint delta is above --max-delta\n";
      session->close();
      return 1;
    }
  }

  std::vector<std::vector<double>> trajectories(n);
  std::vector<int> current_steps(n, 0);
  std::vector<char> reached(n, 0);
  for (size_t i = 0; i < n; ++i) {
    const int steps = num_steps[i];
    if (steps < 2) {
      std::cerr << "num_steps[" << i << "] must be >= 2\n";
      session->close();
      return 1;
    }
    trajectories[i].resize(static_cast<size_t>(steps));
    const double step_size = (target_positions[i] - joint.position[i]) / (steps - 1);
    for (int s = 0; s < steps; ++s) {
      trajectories[i][static_cast<size_t>(s)] = joint.position[i] + step_size * s;
    }
  }

  engineai_sdk::JointCommand cmd;
  cmd.position.assign(n, 0.0);
  cmd.velocity.assign(n, 0.0);
  cmd.feed_forward_torque.assign(n, 0.0);
  cmd.torque.assign(n, 0.0);
  cmd.stiffness = kp;
  cmd.damping = kd;
  cmd.parallel_parser_type = engineai_sdk::ParallelParserType::kClassic;

  const auto period = std::chrono::milliseconds(2);  // 500 Hz
  robot_session_examples::CatchSigInt();
  bool all_reached = false;
  while (!robot_session_examples::g_stop && !all_reached) {
    if (!robot_session_examples::MotionIs(*session, "joint_bridge")) {
      std::cerr << "left joint_bridge, current="
                << robot_session_examples::MotionOrNone(*session) << "\n";
      session->close();
      return 1;
    }

    all_reached = true;
    for (size_t i = 0; i < n; ++i) {
      if (!reached[i]) {
        if (current_steps[i] < num_steps[i]) {
          cmd.position[i] = trajectories[i][static_cast<size_t>(current_steps[i])];
          ++current_steps[i];
          all_reached = false;
        } else {
          cmd.position[i] = target_positions[i];
          reached[i] = 1;
        }
      } else {
        cmd.position[i] = target_positions[i];
      }
    }

    if (all_reached) {
      break;
    }

    const auto err = session->joint_command().send(cmd);
    if (!robot_session_examples::CheckOk(err, "joint_command.send")) {
      session->close();
      return 1;
    }
    std::this_thread::sleep_for(period);
  }

  std::cout << "all joints reached target (or interrupted)\n";
  session->close();
  return 0;
}
