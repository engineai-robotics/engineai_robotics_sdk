// basic_joint_motion_planner: latest_state + optional execute(RESET)
// execute is sent only while the current motion is lower_body_balance.
//
//   ./basic_joint_motion_planner --domain-id 0
//   ./basic_joint_motion_planner --domain-id 0 --reset [--timeout SEC]

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
            << " --domain-id N [--interface IF] [--network-address ADDR] [--reset] [--timeout SEC]\n"
            << "  --reset requires current motion lower_body_balance and polls latest_state until done.\n"
            << "  kOk means the request was sent. Completion is the planner status for this request_id.\n";
}

void PrintPlan(engineai_sdk::Robot* session) {
  if (const auto plan = session->joint_motion_planner().latest_state()) {
    std::cout << "plan request_id=" << plan->request_id << " status=" << static_cast<int>(plan->status)
              << " progress=" << plan->progress << "\n";
  } else {
    std::cout << "plan (no sample)\n";
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

  bool do_reset = false;
  int timeout_sec = 8;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--reset") {
      do_reset = true;
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &timeout_sec, "timeout")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (timeout_sec <= 0) {
    std::cerr << "timeout must be > 0\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  robot_session_examples::WaitForMotionSample(session.get());
  PrintPlan(session.get());
  if (!do_reset) {
    session->close();
    return 0;
  }

  if (!robot_session_examples::RequireCurrentMotion(session.get(), "lower_body_balance")) {
    session->close();
    return 1;
  }

  engineai_sdk::JointMotionPlanRequest req;
  if (const auto plan = session->joint_motion_planner().latest_state()) {
    if (plan->status != engineai_sdk::JointMotionPlanStatus::kIdle) {
      std::cerr << "planner is not idle\n";
      PrintPlan(session.get());
      session->close();
      return 1;
    }
    req.request_id = plan->request_id + 1;
  } else {
    req.request_id = 1;
  }
  req.request_type = engineai_sdk::JointMotionPlanRequestType::kReset;
  const auto err = session->joint_motion_planner().execute(req);
  if (!robot_session_examples::CheckOk(err, "joint_motion_planner.execute")) {
    session->close();
    return 1;
  }
  std::cout << "execute RESET request_id=" << req.request_id
            << " -> " << robot_session_examples::ErrName(err) << " (sent, not completed)\n";

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  bool done = false;
  while (std::chrono::steady_clock::now() < deadline) {
    if (const auto plan = session->joint_motion_planner().latest_state()) {
      if (plan->request_id == req.request_id) {
        std::cout << "plan request_id=" << plan->request_id << " status=" << static_cast<int>(plan->status)
                  << " progress=" << plan->progress << "\n";
        if (plan->status == engineai_sdk::JointMotionPlanStatus::kDisabled) {
          std::cerr << "planner DISABLED\n";
          session->close();
          return 1;
        }
        if (plan->status == engineai_sdk::JointMotionPlanStatus::kIdle && plan->progress >= 1.0) {
          done = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!done) {
    std::cerr << "timeout waiting for RESET request_id=" << req.request_id << "\n";
    PrintPlan(session.get());
    session->close();
    return 1;
  }
  std::cout << "reset completed request_id=" << req.request_id << "\n";
  robot_session_examples::PauseBeforeClose();
  session->close();
  return 0;
}
