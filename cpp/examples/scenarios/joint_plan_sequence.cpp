// joint_plan_sequence: run the existing wave, handshake, and reset YAML files.
// wave: examples/config/{pm01,t800}/motion_plan_wave_hands.yaml
// handshake: examples/config/{pm01,t800,t800pro}/motion_plan_shake_hand.yaml
// reset: examples/config/{model}/motion_plan_reset.yaml
// t800pro has no wave file.
//
//   ./joint_plan_sequence --domain-id 0 --model pm01 --action wave
//   ./joint_plan_sequence --domain-id 0 --model t800 --sequence wave,reset --dry-run

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/motion_plan_runner.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       (--model pm01|t800|t800pro (--action wave|handshake|reset | --sequence a,b)\n"
            << "        | --config PATH.yaml)\n"
            << "       [--actions DIR] [--speed-scale S] [--timeout SEC] [--cancel-after SEC] [--dry-run]\n"
            << "  execute requires current motion lower_body_balance.\n"
            << "  --config runs one motion-plan YAML and does not need --model.\n"
            << "  Default actions dir is examples/config. --dry-run does not send.\n"
            << "  Planner status/progress is the completion signal. Local kOk is only a send result.\n";
}

std::vector<std::string> Split(const std::string& text) {
  std::vector<std::string> parts;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
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
  std::string config_path;
  bool action_set = false;
  std::string actions_dir = "examples/config";
  std::vector<std::string> actions;
  double speed_scale = 1.0;
  double timeout_s = 0.0;
  int cancel_after_ms = -1;
  bool dry_run = false;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--model" && i + 1 < rest.size()) {
      model = rest[++i];
    } else if (arg == "--config" && i + 1 < rest.size()) {
      config_path = rest[++i];
    } else if (arg == "--action" && i + 1 < rest.size()) {
      action_set = true;
      actions.push_back(rest[++i]);
    } else if (arg == "--sequence" && i + 1 < rest.size()) {
      action_set = true;
      const auto parts = Split(rest[++i]);
      actions.insert(actions.end(), parts.begin(), parts.end());
    } else if (arg == "--actions" && i + 1 < rest.size()) {
      actions_dir = rest[++i];
    } else if (arg == "--speed-scale" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &speed_scale, "speed-scale")) {
        return 2;
      }
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &timeout_s, "timeout")) {
        return 2;
      }
    } else if (arg == "--cancel-after" && i + 1 < rest.size()) {
      double cancel_after_s = 0.0;
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &cancel_after_s, "cancel-after")) {
        return 2;
      }
      cancel_after_ms = static_cast<int>(cancel_after_s * 1000.0);
    } else if (arg == "--dry-run") {
      dry_run = true;
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (!config_path.empty() && action_set) {
    std::cerr << "pass either --config or --action/--sequence\n";
    return 2;
  }
  if (config_path.empty() && actions.empty()) {
    actions.push_back("wave");
  }
  if (speed_scale <= 0.0 || timeout_s < 0.0) {
    std::cerr << "speed-scale must be > 0 and timeout must be >= 0\n";
    return 2;
  }
  std::vector<std::pair<std::string, robot_session_examples::MotionPlanFile>> plans;
  if (!config_path.empty()) {
    robot_session_examples::MotionPlanFile plan;
    std::string error;
    if (!robot_session_examples::LoadMotionPlanFile(config_path, &plan, &error)) {
      std::cerr << error << "\n";
      return 1;
    }
    std::cout << "config=" << config_path << "\n";
    robot_session_examples::PrintPlanPreview(config_path, plan, speed_scale);
    plans.emplace_back(config_path, std::move(plan));
  }
  for (const auto& action : actions) {
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
    std::cout << "model=" << model << " action=" << action << "\n";
    robot_session_examples::PrintPlanPreview(path, plan, speed_scale);
    plans.emplace_back(path, std::move(plan));
  }
  if (dry_run) {
    robot_session_examples::MotionPlanRunRequest request;
    request.dry_run = true;
    request.speed_scale = speed_scale;
    for (const auto& item : plans) {
      robot_session_examples::RunMotionPlan(nullptr, item.second, request, nullptr, nullptr);
    }
    return 0;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "lower_body_balance")) {
    session->close();
    return 1;
  }
  engineai_sdk::JointState joint;
  if (!robot_session_examples::WaitJointState(session.get(), &joint)) {
    std::cerr << "no joint_state sample\n";
    session->close();
    return 1;
  }
  std::cout << "motion=" << robot_session_examples::MotionOrNone(*session) << " joint_n=" << joint.position.size() << "\n";
  robot_session_examples::CatchSigInt();
  robot_session_examples::MotionPlanRunRequest request;
  request.speed_scale = speed_scale;
  request.timeout_s = timeout_s;
  request.cancel_after_ms = cancel_after_ms;
  for (const auto& item : plans) {
    const auto code = robot_session_examples::RunMotionPlan(session.get(), item.second, request, nullptr, nullptr);
    std::cout << "result=" << robot_session_examples::PlanRunName(code) << " config=" << item.first << "\n";
    if (code != robot_session_examples::MotionPlanRunCode::kCompleted) {
      session->close();
      return 1;
    }
  }
  session->close();
  return 0;
}
