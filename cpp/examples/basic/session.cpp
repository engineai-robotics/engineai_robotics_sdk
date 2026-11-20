// basic_session: Robot::create / is_open / close
//
// create() only creates local DDS resources. It does not mean a robot was
// discovered or any topic has data. Use basic_motion / basic_read_state.
//
//   ./basic_session --domain-id 0 [--interface eth0] [--network-address 192.168.0.0]

#include <iostream>
#include <utility>
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

  auto result = engineai_sdk::Robot::create(options);
  if (!result) {
    std::cerr << "local session creation failed: " << robot_session_examples::ErrName(result.error) << "\n";
    return 1;
  }
  auto session = std::move(result.session);
  std::cout << "is_open=" << session->is_open() << "\n"
            << "Local session created. This is not a handshake with the robot.\n"
            << "Run basic_motion or basic_read_state to see if samples arrive.\n";
  session->close();
  return 0;
}
