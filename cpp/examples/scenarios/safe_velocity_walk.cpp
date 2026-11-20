// safe_velocity_walk: send a short body-velocity command and always finish at zero.
// kOk means the sample was sent. The SDK has no velocity-execution feedback.
//
//   ./safe_velocity_walk --domain-id 0 --x 0.2 --duration 2 --rate 50

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
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--x V] [--y V] [--yaw-rate W] [--duration SEC] [--rate HZ]\n"
            << "  Defaults: x=0 y=0 yaw-rate=0 duration=2 rate=50.\n"
            << "  --rate must be >= 20. --duration 0 runs until Ctrl+C.\n"
            << "  Requires a motion sample first. Exit always sends zero velocity.\n"
            << "  The robot releases an external velocity if it is not refreshed within 500 ms.\n"
            << "  --x, --y, and --yaw-rate map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
            << "  On PM01 lower_body_balance, linear_velocity[0] around 0.5 is where stepping starts.\n";
}

bool Send(engineai_sdk::Robot* session, double x, double y, double yaw) {
  engineai_sdk::BodyVelocityCommand cmd;
  cmd.x = x;
  cmd.y = y;
  cmd.yaw_rate = yaw;
  return robot_session_examples::CheckOk(session->base_velocity().send(cmd), "base_velocity.send");
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

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double duration = 2.0;
  double rate = 50.0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--x" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &x, "x")) {
        return 2;
      }
    } else if (arg == "--y" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &y, "y")) {
        return 2;
      }
    } else if (arg == "--yaw-rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &yaw, "yaw")) {
        return 2;
      }
    } else if (arg == "--duration" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &duration, "duration")) {
        return 2;
      }
    } else if (arg == "--rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &rate, "rate")) {
        return 2;
      }
    } else if (arg == "--stop-on-exit") {
      continue;
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (duration < 0.0 || rate < 20.0) {
    std::cerr << "duration must be >= 0 and rate must be >= 20\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::WaitForMotionSample(session.get())) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }
  std::cout << "motion=" << robot_session_examples::MotionOrNone(*session) << " x=" << x << " y=" << y
            << " yaw_rate=" << yaw << " rate_hz=" << rate << " duration_s=" << duration << "\n";
  std::cout << "note: send kOk means the command was sent, not that the robot applied the velocity\n";

  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate));
  const auto deadline = duration == 0.0
                            ? std::chrono::steady_clock::time_point::max()
                            : std::chrono::steady_clock::now() +
                                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(duration));
  bool send_ok = true;
  while (send_ok && !robot_session_examples::g_stop && std::chrono::steady_clock::now() < deadline) {
    send_ok = Send(session.get(), x, y, yaw);
    std::this_thread::sleep_for(period);
  }
  const bool stopped = Send(session.get(), 0.0, 0.0, 0.0);
  std::cout << "stop_sent=" << (stopped ? "yes" : "no") << " motion=" << robot_session_examples::MotionOrNone(*session)
            << "\n";
  robot_session_examples::PauseBeforeClose();
  session->close();
  return send_ok && stopped ? 0 : 1;
}
