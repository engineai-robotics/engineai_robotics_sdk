// basic_body_velocity: base_velocity().send
//
//   ./basic_body_velocity --domain-id 0 [--vx V] [--vy V] [--yaw W] [--seconds N]

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
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR] [--vx V] [--vy V] [--yaw W] [--seconds N]\n"
            << "  No velocity execution feedback. Refresh faster than every 500 ms or the robot releases it.\n"
            << "  --vx, --vy, and --yaw map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
            << "  On PM01 lower_body_balance, linear_velocity[0] around 0.5 is where stepping starts.\n"
            << "  Exit sends zero velocity. kOk means the command was sent.\n";
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

  double vx = 0.0, vy = 0.0, yaw = 0.0;
  int seconds = 1;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--vx" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &vx, "vx")) {
        return 2;
      }
    } else if (arg == "--vy" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &vy, "vy")) {
        return 2;
      }
    } else if (arg == "--yaw" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &yaw, "yaw")) {
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
    std::cerr << "seconds must be > 0\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  std::cout << "note: base_velocity has no execution feedback. kOk means the sample was sent.\n";

  engineai_sdk::BodyVelocityCommand cmd;
  cmd.x = vx;
  cmd.y = vy;
  cmd.yaw_rate = yaw;

  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::milliseconds(20);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  bool send_ok = true;
  while (send_ok && !robot_session_examples::g_stop && std::chrono::steady_clock::now() < deadline) {
    send_ok = robot_session_examples::CheckOk(session->base_velocity().send(cmd), "base_velocity.send");
    if (send_ok) {
      std::this_thread::sleep_for(period);
    }
  }

  engineai_sdk::BodyVelocityCommand stop;
  const bool stopped =
      robot_session_examples::CheckOk(session->base_velocity().send(stop), "base_velocity.send(0,0,0)");
  if (!send_ok || !stopped) {
    session->close();
    return 1;
  }

  // Pause before close() so this call can still reach the robot.
  robot_session_examples::PauseBeforeClose();
  session->close();
  return 0;
}
