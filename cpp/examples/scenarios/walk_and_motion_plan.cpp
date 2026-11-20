// walk_and_motion_plan
// Walk with base_velocity while a joint-motion plan plays wave, handshake, or reset.
// Both are sent only while the current motion is lower_body_balance.
// Velocity is refreshed from the planner poll, faster than the 500 ms release window.
// Exit, Ctrl+C, and plan failure send one zero velocity command.
// joint-override is refused: this example does not send JointOverrideCommand.
//
//   ./walk_and_motion_plan --domain-id 0 --model pm01

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/motion_plan_runner.hpp"

namespace {

struct WalkCommand {
  double x = 0.5;
  double y = 0.0;
  double yaw = 0.0;
  bool stop = false;
  bool send_ok = true;
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       --model pm01|t800|t800pro [--interaction-action wave|handshake|reset]\n"
            << "       [--x V] [--y V] [--yaw-rate W] [--actions DIR] [--timeout SEC] [--dry-run]\n"
            << "  Default action is wave and --x is 0.5 (linear_velocity[0]).\n"
            << "  --x, --y, and --yaw-rate map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
            << "  Walking uses base_velocity. The action uses the existing motion-plan YAML.\n"
            << "  --control-mode joint-override is refused and sends nothing.\n";
}

bool SendVelocity(engineai_sdk::Robot* session, double x, double y, double yaw) {
  engineai_sdk::BodyVelocityCommand cmd;
  cmd.x = x;
  cmd.y = y;
  cmd.yaw_rate = yaw;
  return robot_session_examples::CheckOk(session->base_velocity().send(cmd), "base_velocity.send");
}

void PollWalk(engineai_sdk::Robot* session, void* user, bool* cancel) {
  auto* walk = static_cast<WalkCommand*>(user);
  if (robot_session_examples::g_stop) {
    walk->stop = true;
    *cancel = true;
  }
  const double x = walk->stop ? 0.0 : walk->x;
  const double y = walk->stop ? 0.0 : walk->y;
  const double yaw = walk->stop ? 0.0 : walk->yaw;
  if (!SendVelocity(session, x, y, yaw)) {
    walk->send_ok = false;
    *cancel = true;
  }
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

  std::string model;
  std::string control_mode = "motion-plan";
  std::string action = "wave";
  std::string actions_dir = "examples/config";
  double timeout_s = 0.0;
  bool dry_run = false;
  WalkCommand walk;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--model" && i + 1 < rest.size()) {
      model = rest[++i];
    } else if (arg == "--control-mode" && i + 1 < rest.size()) {
      control_mode = rest[++i];
    } else if (arg == "--interaction-action" && i + 1 < rest.size()) {
      action = rest[++i];
    } else if (arg == "--actions" && i + 1 < rest.size()) {
      actions_dir = rest[++i];
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &timeout_s, "timeout")) {
        return 2;
      }
    } else if (arg == "--x" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &walk.x, "x")) {
        return 2;
      }
    } else if (arg == "--y" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &walk.y, "y")) {
        return 2;
      }
    } else if (arg == "--yaw-rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &walk.yaw, "yaw-rate")) {
        return 2;
      }
    } else if (arg == "--dry-run") {
      dry_run = true;
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (control_mode == "joint-override") {
    std::cerr << "refusing joint-override: this example walks with base_velocity and gestures with a motion plan.\n";
    return 2;
  }
  if (control_mode != "motion-plan") {
    std::cerr << "control-mode must be motion-plan or joint-override\n";
    return 2;
  }

  std::string path;
  std::string error;
  if (!robot_session_examples::ResolveActionYaml(actions_dir, model, action, &path, &error)) {
    std::cerr << error << "\n";
    return 2;
  }
  robot_session_examples::MotionPlanFile plan;
  if (!robot_session_examples::LoadMotionPlanFile(path, &plan, &error)) {
    std::cerr << error << "\n";
    return 1;
  }

  std::cout << "model=" << model << " walking=base_velocity x=" << walk.x << " y=" << walk.y << " yaw_rate=" << walk.yaw
            << " interaction=motion_plan action=" << action << "\n";
  std::cout << "Keep sending velocity while the motion plan runs. Send zero velocity on completion, failure, or Ctrl+C. kOk only means the command was sent.\n";
  robot_session_examples::PrintPlanPreview(path, plan, 1.0);
  if (dry_run) {
    robot_session_examples::MotionPlanRunRequest request;
    request.dry_run = true;
    robot_session_examples::RunMotionPlan(nullptr, plan, request, nullptr, nullptr);
    return 0;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "lower_body_balance")) {
    session->close();
    return 1;
  }
  std::cout << "motion=" << robot_session_examples::MotionOrNone(*session) << "\n";
  robot_session_examples::CatchSigInt();
  if (!SendVelocity(session.get(), walk.x, walk.y, walk.yaw)) {
    session->close();
    return 1;
  }

  robot_session_examples::MotionPlanRunRequest request;
  request.timeout_s = timeout_s;
  const auto code = robot_session_examples::RunMotionPlan(session.get(), plan, request, PollWalk, &walk);
  walk.stop = true;
  const bool stopped = SendVelocity(session.get(), 0.0, 0.0, 0.0);
  std::cout << "interaction_result=" << robot_session_examples::PlanRunName(code)
            << " stop_sent=" << (stopped ? "yes" : "no")
            << " motion=" << robot_session_examples::MotionOrNone(*session) << "\n";
  robot_session_examples::PauseBeforeClose();
  session->close();
  const bool plan_ok = code == robot_session_examples::MotionPlanRunCode::kCompleted ||
                       code == robot_session_examples::MotionPlanRunCode::kCancelled;
  return walk.send_ok && stopped && plan_ok ? 0 : 1;
}
