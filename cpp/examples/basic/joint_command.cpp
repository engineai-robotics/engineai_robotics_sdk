// Safety: joint_bridge commands the joints directly. The standing pose in this
// mode is not necessarily stable, so hoist the robot before use. Both feet
// must be on the ground before entering walk.
//
// basic_joint_command: read joints, send a small move, keep refreshing, confirm execution.
// Requires current motion == joint_bridge (does not call request_transition).
// latest_feedback is the robot's command echo. Execution is judged from joint_state.
//
//   ./basic_joint_command --domain-id 0 [--joint I] [--delta RAD] [--seconds N] [--rate HZ]
//   ./basic_joint_command --domain-id 0 --seconds 0

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

constexpr double kMaxAbsDelta = 0.1;
constexpr double kTrackTolerance = 0.03;

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--joint I] [--delta RAD] [--seconds N | --hold-ms N] [--rate HZ]\n"
            << "  Requires motion joint_bridge. Moves one joint by --delta (default 0.05 rad, joint 0).\n"
            << "  |delta| must be <= 0.1 rad. --seconds 0 runs until Ctrl+C. Default rate is 200 Hz.\n"
            << "  Stops if joint_state goes stale or the motion leaves joint_bridge.\n"
            << "  kOk means this process sent the command. Execution is the measured joint_state.\n"
            << "  latest_feedback is the command echo, not the measured joint.\n";
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

  int joint_index = 0;
  double delta = 0.05;
  double seconds = 2.0;
  bool have_seconds = false;
  bool have_hold_ms = false;
  double rate = 200.0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--joint" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &joint_index, "joint")) {
        return 2;
      }
    } else if (arg == "--delta" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &delta, "delta")) {
        return 2;
      }
    } else if (arg == "--seconds" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &seconds, "seconds")) {
        return 2;
      }
      have_seconds = true;
    } else if (arg == "--hold-ms" && i + 1 < rest.size()) {
      double parsed_seconds_ms = 0.0;
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &parsed_seconds_ms, "hold-ms")) {
        return 2;
      }
      seconds = parsed_seconds_ms / 1000.0;
      have_hold_ms = true;
    } else if (arg == "--rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &rate, "rate")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (have_seconds && have_hold_ms) {
    std::cerr << "pass only one of --seconds or --hold-ms\n";
    return 2;
  }
  if (joint_index < 0 || seconds < 0.0 || rate < 5.0 || std::abs(delta) > kMaxAbsDelta) {
    std::cerr << "joint must be >= 0, seconds must be >= 0, rate must be >= 5, |delta| must be <= " << kMaxAbsDelta
              << "\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "joint_bridge")) {
    session->close();
    return 1;
  }

  engineai_sdk::JointState joint;
  if (!robot_session_examples::WaitJointState(session.get(), &joint)) {
    std::cerr << "No joint_state sample; abort\n";
    session->close();
    return 1;
  }
  if (static_cast<size_t>(joint_index) >= joint.position.size()) {
    std::cerr << "joint index " << joint_index << " is outside joint_state n=" << joint.position.size() << "\n";
    session->close();
    return 2;
  }

  engineai_sdk::JointCommand cmd;
  cmd.position = joint.position;
  cmd.position[static_cast<size_t>(joint_index)] += delta;
  cmd.velocity.assign(joint.position.size(), 0.0);
  cmd.feed_forward_torque.assign(joint.position.size(), 0.0);
  cmd.torque.assign(joint.position.size(), 0.0);
  cmd.stiffness.assign(joint.position.size(), 200.0);
  cmd.damping.assign(joint.position.size(), 5.0);
  cmd.parallel_parser_type = engineai_sdk::ParallelParserType::kClassic;
  const double command_q = cmd.position[static_cast<size_t>(joint_index)];
  std::cout << "joint=" << joint_index << " measured=" << joint.position[static_cast<size_t>(joint_index)]
            << " command=" << command_q << " delta=" << delta << " rate_hz=" << rate << "\n";
  std::cout << "note: latest_feedback is the command echo. Execution is compared with joint_state.\n";

  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate));
  const auto deadline = seconds == 0.0
                            ? std::chrono::steady_clock::time_point::max()
                            : std::chrono::steady_clock::now() +
                                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(seconds));
  engineai_sdk::SessionError last = engineai_sdk::SessionError::kOk;
  while (!robot_session_examples::g_stop && std::chrono::steady_clock::now() < deadline) {
    if (!robot_session_examples::MotionIs(*session, "joint_bridge")) {
      std::cerr << "left joint_bridge, current=" << robot_session_examples::MotionOrNone(*session) << "\n";
      session->close();
      return 1;
    }
    if (!robot_session_examples::IsFresh(session->joint_state().last_receive_time(),
                                         robot_session_examples::kStateMaxAge)) {
      std::cerr << "joint_state stale (" << robot_session_examples::SinceReceiveText(session->joint_state().last_receive_time())
                << "); stop sending\n";
      session->close();
      return 1;
    }
    last = session->joint_command().send(cmd);
    if (last != engineai_sdk::SessionError::kOk) {
      break;
    }
    std::this_thread::sleep_for(period);
  }
  if (!robot_session_examples::CheckOk(last, "joint_command.send")) {
    session->close();
    return 1;
  }

  engineai_sdk::JointState measured;
  const bool have_measured =
      robot_session_examples::IsFresh(session->joint_state().last_receive_time(), robot_session_examples::kStateMaxAge) &&
      session->joint_state().latest(measured) && static_cast<size_t>(joint_index) < measured.position.size();
  if (!have_measured) {
    std::cerr << "no fresh joint_state to confirm execution\n";
    session->close();
    return 1;
  }
  const double error = std::abs(measured.position[static_cast<size_t>(joint_index)] - command_q);
  const bool executed = error <= kTrackTolerance;
  std::cout << "executed=" << (executed ? "yes" : "no") << " joint=" << joint_index << " command=" << command_q
            << " measured=" << measured.position[static_cast<size_t>(joint_index)] << " error=" << error << "\n";

  if (const auto fb = session->joint_command().latest_feedback()) {
    std::cout << "latest_feedback n=" << fb->position.size()
              << " since_receive=" << robot_session_examples::SinceReceiveText(session->joint_command().last_feedback_time())
              << " (command echo)\n";
  } else {
    std::cout << "latest_feedback (no sample); this does not decide execution\n";
  }

  session->close();
  return executed ? 0 : 1;
}
