// basic_read_diagnostics: power / motor_diagnostics / node_heartbeat /
// motor_command
//
// Waits up to 5 s for all four. motor_debug arrives in about 10 ms; leaving on
// that first sample skips the 50 ms power sample and the 1 s heartbeat.
// A topic the robot does not publish stays "(no sample)" after the wait.
//
//   ./basic_read_diagnostics --domain-id 0 [--interface eth0] [--network-address 192.168.0.0]

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

void PrintUsage(const char* argv0) { std::cerr << "Usage: " << argv0 << " --domain-id N [--interface IF] [--network-address ADDR]\n"; }

}  // namespace

int main(int argc, char** argv) {
  if (robot_session_examples::WantsHelp(argc, argv)) {
    PrintUsage(argv[0]);
    return 0;
  }

  engineai_sdk::SessionOptions options;
  std::vector<std::string> rest;
  if (!robot_session_examples::ParseSessionArgs(argc, argv, &options, &rest) || !rest.empty()) {
    PrintUsage(argv[0]);
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!session->is_open()) {
      break;
    }
    if (session->power().latest() && session->motor_diagnostics().latest() && session->node_heartbeat().latest() &&
        session->motor_command().latest()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (const auto power = session->power().latest()) {
    const auto& board = power->power_info;
    std::cout << "power enable=" << board.enable << " pct=" << board.percentage << " V=" << board.voltage
              << " A=" << board.current << " err=" << board.error_code;
    if (power->battery_info) {
      const auto& battery = *power->battery_info;
      std::cout << " bms_soc=" << static_cast<unsigned>(battery.soc) << " bms_temp=" << battery.temp
                << " bms_current_ma=" << battery.bms_current;
    } else {
      std::cout << " bms=(unavailable)";
    }
    std::cout << "\n";
  } else {
    std::cout << "power (no sample)\n";
  }

  if (const auto dbg = session->motor_diagnostics().latest()) {
    std::cout << "motor_diagnostics n_temp=" << dbg->motor_temperature.size() << " n_offline=" << dbg->offline.size()
              << "\n";
  } else {
    std::cout << "motor_diagnostics (no sample)\n";
  }

  if (const auto hb = session->node_heartbeat().latest()) {
    std::cout << "node_heartbeat node=" << hb->node_name << " status=" << hb->node_status
              << " err=" << hb->error_message << "\n";
  } else {
    std::cout << "node_heartbeat (no sample)\n";
  }

  if (const auto motor_cmd = session->motor_command().latest()) {
    std::cout << "motor_command n_pos=" << motor_cmd->position.size();
    if (!motor_cmd->position.empty()) {
      std::cout << " q0=" << motor_cmd->position[0];
    }
    std::cout << "\n";
  } else {
    std::cout << "motor_command (no sample)\n";
  }

  if (!session->power().latest() && !session->motor_diagnostics().latest() && !session->node_heartbeat().latest() &&
      !session->motor_command().latest()) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }

  session->close();
  return 0;
}
