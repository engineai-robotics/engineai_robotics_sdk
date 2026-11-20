// gamepad_input_monitor: print named buttons and sticks from gamepad().latest().
//
//   ./gamepad_input_monitor --domain-id 0 --watch --interval 0.2

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

const char* kButtons[] = {"LB", "RB", "A", "B", "X", "Y", "Back", "Start", "Up", "Down", "Left", "Right"};
const char* kAxes[] = {"LT", "RT", "LeftStickX", "LeftStickY", "RightStickX", "RightStickY"};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--watch] [--interval SEC] [--json]\n"
            << "  One sample by default. --watch repeats until Ctrl+C.\n";
}

void PrintPad(const engineai_sdk::GamepadKeys* pad, bool as_json) {
  if (pad == nullptr) {
    if (as_json) {
      std::cout << "{\"sample\": false}\n";
    } else {
      std::cout << "gamepad=(no sample)\n";
      std::cout << "hint: confirm the robot is publishing gamepad data, --domain-id matches ROS_DOMAIN_ID, "
                   "and the NIC selection can see the robot.\n";
    }
    return;
  }
  if (as_json) {
    std::cout << "{\"sample\": true, \"hardware_connected\": " << (pad->hardware_connected ? "true" : "false")
              << ", \"buttons\": {";
    for (size_t i = 0; i < 12; ++i) {
      if (i != 0) std::cout << ", ";
      std::cout << "\"" << kButtons[i] << "\": " << pad->digital_states[i];
    }
    std::cout << "}, \"axes\": {";
    for (size_t i = 0; i < 6; ++i) {
      if (i != 0) std::cout << ", ";
      std::cout << "\"" << kAxes[i] << "\": " << pad->analog_states[i];
    }
    std::cout << "}}\n";
    return;
  }
  std::cout << "hardware_connected=" << (pad->hardware_connected ? "true" : "false") << "\n";
  if (!pad->hardware_connected) {
    std::cout << "hint: gamepad sample arrived but hardware_connected is false. Check the physical pad.\n";
  }
  for (size_t i = 0; i < 12; ++i) {
    std::cout << kButtons[i] << "=" << pad->digital_states[i] << " ";
  }
  std::cout << "\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << kAxes[i] << "=" << pad->analog_states[i] << " ";
  }
  std::cout << "\n";
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
  bool watch = false;
  bool as_json = false;
  double interval = 0.2;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--watch") {
      watch = true;
    } else if (arg == "--json") {
      as_json = true;
    } else if (arg == "--interval" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &interval, "interval")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (interval <= 0.0) {
    std::cerr << "interval must be > 0\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  const auto discover = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < discover && !session->gamepad().latest()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(interval));
  bool saw_sample = false;
  do {
    const auto pad = session->gamepad().latest();
    if (pad) {
      saw_sample = true;
    }
    PrintPad(pad ? &*pad : nullptr, as_json);
    if (!watch || robot_session_examples::g_stop) {
      break;
    }
    std::this_thread::sleep_for(period);
  } while (!robot_session_examples::g_stop);
  session->close();
  return saw_sample ? 0 : 1;
}
