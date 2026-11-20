// basic_read_state: joint_state / motor_state / imu / gamepad
//
// latest() keeps the last sample after the robot stops publishing, so each line
// also prints since_receive, the time since last_receive_time().
//
//   ./basic_read_state --domain-id 0 [--interface eth0] [--network-address 192.168.0.0]

#include <iostream>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --domain-id N [--interface IF] [--network-address ADDR]\n";
}

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

  using robot_session_examples::SinceReceiveText;
  auto session = robot_session_examples::CreateSessionOrExit(options);
  engineai_sdk::JointState joint_wait;
  const bool have_joint = robot_session_examples::WaitJointState(session.get(), &joint_wait);

  engineai_sdk::JointState joint;
  if (session->joint_state().latest(joint)) {
    std::cout << "joint_state n=" << joint.position.size();
    if (!joint.position.empty()) {
      std::cout << " q0=" << joint.position[0];
    }
    std::cout << " since_receive=" << SinceReceiveText(session->joint_state().last_receive_time()) << "\n";
  } else {
    std::cout << "joint_state (no sample)\n";
  }

  engineai_sdk::JointState motor;
  if (session->motor_state().latest(motor)) {
    std::cout << "motor_state n=" << motor.position.size()
              << " since_receive=" << SinceReceiveText(session->motor_state().last_receive_time()) << "\n";
  } else {
    std::cout << "motor_state (no sample)\n";
  }

  if (const auto imu = session->imu().latest()) {
    std::cout << "imu rpy=(" << imu->rpy.x << ", " << imu->rpy.y << ", " << imu->rpy.z << ")"
              << " since_receive=" << SinceReceiveText(session->imu().last_receive_time()) << "\n";
  } else {
    std::cout << "imu (no sample)\n";
  }

  if (const auto pad = session->gamepad().latest()) {
    std::cout << "gamepad connected=" << pad->hardware_connected
              << " since_receive=" << SinceReceiveText(session->gamepad().last_receive_time()) << "\n";
  } else {
    std::cout << "gamepad (no sample)\n";
  }

  if (!have_joint && !session->motor_state().latest() && !session->imu().latest() &&
      !session->gamepad().latest()) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }

  session->close();
  return 0;
}
