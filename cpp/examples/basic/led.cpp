// basic_led: led().set_mode
//
//   ./basic_led --domain-id 0
//   ./basic_led --domain-id 0 --color 2

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --domain-id N [--interface IF] [--network-address ADDR] [--color N]\n"
            << "  Sets one lamp mode. There is no readback. Fault and safety lamps may hide the request.\n";
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

  int color = static_cast<int>(engineai_sdk::LedColor::kBlinkGreen);
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--color" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &color, "color")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  // Wait for a heartbeat or motion sample before a one-shot command.
  if (!robot_session_examples::WaitForRobotSample(session.get())) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }
  const auto mode = static_cast<engineai_sdk::LedColor>(color);
  const auto err = session->led().set_mode(mode);
  if (!robot_session_examples::CheckOk(err, "led.set_mode")) {
    session->close();
    return 1;
  }
  std::cout << "led.set_mode(" << color << ") sent. kOk means this process sent the request, not that the lamp changed.\n";
  // Pause before close() so this call can still reach the robot.
  robot_session_examples::PauseBeforeClose();
  session->close();
  return 0;
}
