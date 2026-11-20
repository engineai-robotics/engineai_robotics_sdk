// basic_joint_override: refresh a joint override and release it explicitly
//
//   ./basic_joint_override --domain-id 0 [--weight W] [--seconds N]

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --domain-id N [--interface IF] [--network-address ADDR] [--weight W] [--seconds N]\n"
            << "  No execution feedback. Exit sends weight 0. close() does not release the overlay.\n";
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

  double weight = 0.5;
  int seconds = 1;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--weight" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &weight, "weight")) {
        return 2;
      }
    } else if (arg == "--seconds" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &seconds, "seconds")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (seconds <= 0) {
    PrintUsage(argv[0]);
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  robot_session_examples::WaitForMotionSample(session.get());
  std::cout << "current motion=" << robot_session_examples::MotionOrNone(*session) << "\n";

  engineai_sdk::JointState joint;
  if (!session->joint_state().latest(joint) || joint.position.empty()) {
    std::cerr << "No joint_state sample; abort\n";
    session->close();
    return 1;
  }

  engineai_sdk::JointOverrideCommand override_cmd;
  override_cmd.weight = weight;
  override_cmd.joint_indices.resize(static_cast<int32_t>(joint.position.size()));
  for (size_t i = 0; i < joint.position.size(); ++i) {
    override_cmd.joint_indices[i] = static_cast<int32_t>(i);
  }
  override_cmd.command.position = joint.position;
  override_cmd.command.velocity.assign(joint.position.size(), 0.0);
  override_cmd.command.feed_forward_torque.assign(joint.position.size(), 0.0);
  override_cmd.command.torque.assign(joint.position.size(), 0.0);
  override_cmd.command.stiffness.assign(joint.position.size(), 200.0);
  override_cmd.command.damping.assign(joint.position.size(), 5.0);
  override_cmd.command.parallel_parser_type = engineai_sdk::ParallelParserType::kClassic;

  robot_session_examples::CatchSigInt();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  bool send_ok = true;
  bool sent = false;
  while (send_ok && !robot_session_examples::g_stop && std::chrono::steady_clock::now() < deadline) {
    send_ok = robot_session_examples::CheckOk(session->joint_override().send(override_cmd), "joint_override.send");
    if (send_ok) {
      sent = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  std::cout << "joint_override refreshed for " << seconds << "s (weight=" << weight << ", n=" << joint.position.size()
            << ")\n";
  std::cout << "note: no execution feedback. Sending weight 0 before close().\n";

  bool released = true;
  if (sent) {
    engineai_sdk::JointOverrideCommand release;
    release.weight = 0.0;
    released = robot_session_examples::CheckOk(session->joint_override().send(release), "joint_override.release");
  }
  if (!send_ok || !released) {
    session->close();
    return 1;
  }
  // Pause before close() so this call can still reach the robot.
  robot_session_examples::PauseBeforeClose();
  session->close();
  return 0;
}
