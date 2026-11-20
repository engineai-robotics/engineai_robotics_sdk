// Safety: joint_bridge commands the joints directly. The standing pose in this
// mode is not necessarily stable, so hoist the robot before use. Both feet
// must be on the ground before entering walk.
//
// t800pro_hand_gesture_control: open, close, or relax the t800pro hands in joint_bridge.
// open and relax use the existing extend pose. close uses the existing loose_fist pose.
// pinch and custom-yaml have no verified angles in this tree and are refused.
//
//   ./t800pro_hand_gesture_control --domain-id 0 --enable-low-level --hand both --gesture open
//   ./t800pro_hand_gesture_control --domain-id 0 --enable-low-level --pose both

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

constexpr size_t kNumJoints = 43;
constexpr int kLeftHand[] = {20, 21, 22, 23, 24, 25, 26};
constexpr int kRightHand[] = {34, 35, 36, 37, 38, 39, 40};
const double kLeftExtend[] = {0, 0, 0, 0, 0, 0, 0};
const double kLeftFist[] = {-0.85, 0.62, 0.48, -0.80, -0.58, -0.80, -0.58};
const double kRightExtend[] = {0, 0, 0, 0, 0, 0, 0};
const double kRightFist[] = {0.85, 0.62, 0.48, 0.80, 0.58, 0.80, 0.58};
const double kStiffness[] = {
    1080, 480, 880, 1000, 800, 100, 1080, 480, 880, 1000, 800, 100, 200, 120, 120, 120, 120, 120, 10, 10, 5, 5,
    5,    5,   5,   5,    5,   120, 120,  120, 120, 120,  10,  10,  5,   5,   5,   5,  5,  5, 5,  100, 100};
const double kDamping[] = {25,  25,  25, 25, 2,   2,   25,  25,  25, 25, 2,   2,   1,   1.8, 1.5,
                           1.5, 1.8, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1.8, 1.5, 1.5,
                           1.8, 1.2, 0.8, 0.8, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1,   1};

static_assert(sizeof(kStiffness) / sizeof(kStiffness[0]) == kNumJoints, "stiffness size");
static_assert(sizeof(kDamping) / sizeof(kDamping[0]) == kNumJoints, "damping size");

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR] --enable-low-level\n"
            << "       [--hand left|right|both] [--gesture open|close|relax|both|pinch|custom-yaml]\n"
            << "       [--pose extend|loose_fist|both] [--exec-time SEC] [--hold-time SEC] [--duration SEC] [--rate HZ]\n"
            << "  T800 Pro joint_bridge only. Expects 43 joint_state values.\n"
            << "  Each pose interpolates for --exec-time, then holds for --hold-time.\n"
            << "  --pose both and --gesture both run loose fist, then extend. --duration sets --exec-time.\n"
            << "  pinch and custom-yaml are refused: this tree has no verified angles for them.\n";
}

std::vector<int> HandJoints(const std::string& hand) {
  if (hand == "left") {
    return std::vector<int>(std::begin(kLeftHand), std::end(kLeftHand));
  }
  if (hand == "right") {
    return std::vector<int>(std::begin(kRightHand), std::end(kRightHand));
  }
  std::vector<int> both(std::begin(kLeftHand), std::end(kLeftHand));
  both.insert(both.end(), std::begin(kRightHand), std::end(kRightHand));
  return both;
}

std::vector<double> GoalHands(const std::string& gesture, const std::string& hand) {
  const bool closed = gesture == "close";
  const double* left = closed ? kLeftFist : kLeftExtend;
  const double* right = closed ? kRightFist : kRightExtend;
  if (hand == "left") {
    return std::vector<double>(left, left + 7);
  }
  if (hand == "right") {
    return std::vector<double>(right, right + 7);
  }
  std::vector<double> both(left, left + 7);
  both.insert(both.end(), right, right + 7);
  return both;
}

engineai_sdk::JointCommand MakeCommand(const std::vector<double>& positions) {
  engineai_sdk::JointCommand cmd;
  cmd.position = positions;
  cmd.velocity.assign(kNumJoints, 0.0);
  cmd.feed_forward_torque.assign(kNumJoints, 0.0);
  cmd.torque.assign(kNumJoints, 0.0);
  cmd.stiffness.assign(kStiffness, kStiffness + kNumJoints);
  cmd.damping.assign(kDamping, kDamping + kNumJoints);
  cmd.parallel_parser_type = engineai_sdk::ParallelParserType::kClassic;
  return cmd;
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

  bool enable = false;
  std::string hand = "both";
  std::string gesture = "open";
  bool gesture_set = false;
  bool pose_set = false;
  double exec_time = 3.0;
  double hold_time = 2.5;
  double rate = 500.0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--enable-low-level") {
      enable = true;
    } else if (arg == "--hand" && i + 1 < rest.size()) {
      hand = rest[++i];
    } else if (arg == "--gesture" && i + 1 < rest.size()) {
      gesture = rest[++i];
      gesture_set = true;
    } else if (arg == "--pose" && i + 1 < rest.size()) {
      const std::string pose = rest[++i];
      pose_set = true;
      if (pose == "extend") {
        gesture = "open";
      } else if (pose == "loose_fist") {
        gesture = "close";
      } else if (pose == "both") {
        gesture = "both";
      } else {
        std::cerr << "--pose must be extend, loose_fist, or both\n";
        return 2;
      }
    } else if ((arg == "--exec-time" || arg == "--duration") && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &exec_time, "exec-time")) {
        return 2;
      }
    } else if (arg == "--hold-time" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &hold_time, "hold-time")) {
        return 2;
      }
    } else if (arg == "--rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &rate, "rate")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (!enable) {
    std::cerr << "refusing hand command without --enable-low-level\n";
    return 2;
  }
  if (gesture_set && pose_set) {
    std::cerr << "pass only one of --gesture or --pose\n";
    return 2;
  }
  if (hand != "left" && hand != "right" && hand != "both") {
    std::cerr << "--hand must be left, right, or both\n";
    return 2;
  }
  if (gesture == "pinch" || gesture == "custom-yaml") {
    std::cerr << "refusing gesture " << gesture << ": this tree has no verified angles for it\n";
    return 2;
  }
  if (gesture != "open" && gesture != "close" && gesture != "relax" && gesture != "both") {
    std::cerr << "--gesture must be open, close, relax, both, pinch, or custom-yaml\n";
    return 2;
  }
  if (exec_time <= 0.0 || hold_time < 0.0 || rate <= 0.0) {
    std::cerr << "exec-time and rate must be > 0, hold-time must be >= 0\n";
    return 2;
  }
  if (gesture == "relax") {
    std::cout << "relax uses the existing extend pose\n";
  }
  std::cout << "model_check=joint_count_43_only example=t800pro_hand_gesture_control\n";

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "joint_bridge")) {
    session->close();
    return 1;
  }
  engineai_sdk::JointState joint;
  if (!robot_session_examples::WaitJointState(session.get(), &joint) || joint.position.size() != kNumJoints) {
    std::cerr << "need joint_state with " << kNumJoints << " joints, got " << joint.position.size() << "\n";
    session->close();
    return 1;
  }
  const auto joints = HandJoints(hand);
  std::vector<std::string> poses;
  if (gesture == "both") {
    poses = {"close", "open"};
  } else if (gesture == "relax") {
    poses = {"open"};
  } else {
    poses = {gesture};
  }
  std::cout << "hand=" << hand << " gesture=" << gesture << " indices:";
  for (size_t i = 0; i < joints.size(); ++i) {
    std::cout << " " << joints[i] << "=" << joint.position[static_cast<size_t>(joints[i])];
  }
  std::cout << "\n";

  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate));
  auto hold = joint.position;
  bool ok = true;
  for (const auto& pose_name : poses) {
    if (!ok || robot_session_examples::g_stop) {
      break;
    }
    const auto goal = GoalHands(pose_name, hand);
    std::vector<double> start_hands;
    start_hands.reserve(joints.size());
    for (int index : joints) {
      start_hands.push_back(hold[static_cast<size_t>(index)]);
    }
    std::cout << "pose=" << pose_name << " exec_time_s=" << exec_time << " hold_time_s=" << hold_time << "\n";
    const auto started = std::chrono::steady_clock::now();
    while (ok && !robot_session_examples::g_stop) {
      if (!robot_session_examples::MotionIs(*session, "joint_bridge")) {
        std::cerr << "left joint_bridge, current=" << robot_session_examples::MotionOrNone(*session) << "\n";
        ok = false;
        break;
      }
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      if (elapsed >= exec_time) {
        break;
      }
      const double alpha = elapsed / exec_time;
      auto command_position = hold;
      for (size_t i = 0; i < joints.size(); ++i) {
        command_position[static_cast<size_t>(joints[i])] = start_hands[i] + (goal[i] - start_hands[i]) * alpha;
      }
      if (!robot_session_examples::CheckOk(session->joint_command().send(MakeCommand(command_position)),
                                           "joint_command.send")) {
        ok = false;
        break;
      }
      std::this_thread::sleep_for(period);
    }
    if (ok && !robot_session_examples::g_stop) {
      for (size_t i = 0; i < joints.size(); ++i) {
        hold[static_cast<size_t>(joints[i])] = goal[i];
      }
      if (!robot_session_examples::CheckOk(session->joint_command().send(MakeCommand(hold)), "joint_command.send")) {
        ok = false;
      }
    }
    const auto hold_until =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(hold_time));
    while (ok && !robot_session_examples::g_stop && std::chrono::steady_clock::now() < hold_until) {
      if (!robot_session_examples::MotionIs(*session, "joint_bridge")) {
        std::cerr << "left joint_bridge, current=" << robot_session_examples::MotionOrNone(*session) << "\n";
        ok = false;
        break;
      }
      if (!robot_session_examples::CheckOk(session->joint_command().send(MakeCommand(hold)), "joint_command.send")) {
        ok = false;
        break;
      }
      std::this_thread::sleep_for(period);
    }
  }
  if (const auto feedback = session->joint_command().latest_feedback()) {
    std::cout << "feedback";
    for (int index : joints) {
      if (static_cast<size_t>(index) < feedback->position.size()) {
        std::cout << " " << index << "=" << feedback->position[static_cast<size_t>(index)];
      }
    }
    std::cout << "\n";
  } else {
    std::cout << "feedback=(no sample)\n";
  }
  session->close();
  return ok ? 0 : 1;
}
