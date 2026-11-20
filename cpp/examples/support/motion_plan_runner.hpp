#pragma once

// Shared loader and runner for the motion-plan YAML files already in examples/config.
// Completion follows the motion-plan YAML runner: local kOk only means the request was sent.

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/yaml_util.hpp"

namespace robot_session_examples {

struct MotionPlanTask {
  std::string name;
  bool reset = false;
  std::vector<double> target_positions;
  double duration = 2.0;
  bool use_gravity_compensation = true;
  std::vector<double> stiffness;
  std::vector<double> damping;
};

struct MotionPlanFile {
  std::vector<int32_t> joint_indices;
  std::vector<MotionPlanTask> tasks;
};

enum class MotionPlanRunCode {
  kCompleted,
  kDryRun,
  kSendFailed,
  kTimeout,
  kCancelled,
  kNotConfirmed,
  kPlannerDisabled,
  kNotIdle,
  kNoState,
};

struct MotionPlanRunRequest {
  double speed_scale = 1.0;
  double timeout_s = 0.0;
  int cancel_after_ms = -1;
  bool dry_run = false;
};

using MotionPlanPoll = void (*)(engineai_sdk::Robot* session, void* user, bool* cancel);

inline const char* PlanStatusName(engineai_sdk::JointMotionPlanStatus status) {
  using S = engineai_sdk::JointMotionPlanStatus;
  switch (status) {
    case S::kDisabled:
      return "DISABLED";
    case S::kIdle:
      return "IDLE";
    case S::kExecuting:
      return "EXECUTING";
    case S::kExiting:
      return "EXITING";
  }
  return "UNKNOWN";
}

inline const char* PlanRunName(MotionPlanRunCode code) {
  switch (code) {
    case MotionPlanRunCode::kCompleted:
      return "completed";
    case MotionPlanRunCode::kDryRun:
      return "dry_run";
    case MotionPlanRunCode::kSendFailed:
      return "send_failed";
    case MotionPlanRunCode::kTimeout:
      return "timeout";
    case MotionPlanRunCode::kCancelled:
      return "cancelled";
    case MotionPlanRunCode::kNotConfirmed:
      return "not_confirmed";
    case MotionPlanRunCode::kPlannerDisabled:
      return "planner_disabled";
    case MotionPlanRunCode::kNotIdle:
      return "planner_not_idle";
    case MotionPlanRunCode::kNoState:
      return "no_planner_state";
  }
  return "unknown";
}

inline std::string ToUpperCopy(std::string text) {
  for (char& ch : text) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
  return text;
}

inline bool LoadMotionPlanFile(const std::string& path, MotionPlanFile* out, std::string* error) {
  if (!FileExists(path)) {
    if (error) {
      *error = "config not found: " + path;
    }
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node plan = root["motion_plan"];
    if (!plan || !plan.IsSequence() || plan.size() < 2) {
      if (error) {
        *error = "config missing motion_plan sequence";
      }
      return false;
    }
    MotionPlanFile loaded;
    FlattenInts(plan[0]["joint_indices"], &loaded.joint_indices);
    if (loaded.joint_indices.empty()) {
      if (error) {
        *error = "config missing joint_indices";
      }
      return false;
    }
    for (size_t i = 1; i < plan.size(); ++i) {
      const YAML::Node item = plan[i];
      MotionPlanTask task;
      task.name = item["motion"] ? item["motion"].as<std::string>() : ("motion_" + std::to_string(i));
      const std::string request_type = item["request_type"] ? ToUpperCopy(item["request_type"].as<std::string>()) : "";
      if (request_type == "RESET") {
        task.reset = true;
        loaded.tasks.push_back(std::move(task));
        continue;
      }
      FlattenDoubles(item["target_positions"], &task.target_positions);
      if (item["duration"]) {
        double duration = 0.0;
        if (!ParseFiniteDouble(item["duration"].Scalar(), &duration) || !(duration > 0.0)) {
          throw std::runtime_error("task '" + task.name + "' duration must be finite and > 0");
        }
        task.duration = duration;
      }
      if (item["use_gravity_compensation"]) {
        bool enabled = true;
        if (!ParseYamlBool(item["use_gravity_compensation"], &enabled)) {
          throw std::runtime_error("task '" + task.name + "' use_gravity_compensation must be true or false");
        }
        task.use_gravity_compensation = enabled;
      }
      FlattenDoubles(item["stiffness"], &task.stiffness);
      FlattenDoubles(item["damping"], &task.damping);
      if (task.target_positions.size() != loaded.joint_indices.size()) {
        if (error) {
          *error = "task '" + task.name + "' target_positions size != joint_indices";
        }
        return false;
      }
      loaded.tasks.push_back(std::move(task));
    }
    if (loaded.tasks.empty()) {
      if (error) {
        *error = "no motion tasks in config";
      }
      return false;
    }
    *out = std::move(loaded);
    return true;
  } catch (const std::exception& ex) {
    if (error) {
      *error = ex.what();
    }
    return false;
  }
}

inline bool ResolveActionYaml(const std::string& actions_dir, const std::string& model, const std::string& action,
                              std::string* path, std::string* error) {
  if (model != "pm01" && model != "t800" && model != "t800pro") {
    if (error) {
      *error = "model must be pm01, t800, or t800pro";
    }
    return false;
  }
  const std::string root = actions_dir.empty() ? "examples/config" : actions_dir;
  std::string file;
  if (action == "wave") {
    if (model == "t800pro") {
      if (error) {
        *error = "t800pro has no motion_plan_wave_hands.yaml; use handshake or reset";
      }
      return false;
    }
    file = "motion_plan_wave_hands.yaml";
  } else if (action == "handshake") {
    file = "motion_plan_shake_hand.yaml";
  } else if (action == "reset") {
    file = "motion_plan_reset.yaml";
  } else {
    if (error) {
      *error = "action must be wave, handshake, or reset";
    }
    return false;
  }
  *path = root + "/" + model + "/" + file;
  return true;
}

inline void PrintPlanPreview(const std::string& path, const MotionPlanFile& plan, double speed_scale) {
  std::cout << "config=" << path << "\n";
  std::cout << "joint_indices:";
  for (int32_t index : plan.joint_indices) {
    std::cout << " " << index;
  }
  std::cout << "\n";
  double total = 0.0;
  for (const auto& task : plan.tasks) {
    const double seconds = task.reset ? 0.0 : task.duration / speed_scale;
    total += seconds;
    std::cout << "step name=" << task.name << " type=" << (task.reset ? "RESET" : "PLAN_EXECUTE")
              << " joints=" << plan.joint_indices.size() << " duration_s=" << seconds << "\n";
  }
  std::cout << "estimated_duration_s=" << total << "\n";
}

inline MotionPlanRunCode ConfirmCancel(engineai_sdk::Robot* session, int32_t request_id, MotionPlanPoll poll,
                                       void* poll_user) {
  engineai_sdk::JointMotionPlanRequest req;
  req.request_id = request_id;
  req.request_type = engineai_sdk::JointMotionPlanRequestType::kCancel;
  const auto err = session->joint_motion_planner().execute(req);
  std::cout << "cancel send request_id=" << request_id << " -> " << ErrName(err) << "\n";
  if (err != engineai_sdk::SessionError::kOk) {
    std::cout << "cancel was not sent; robot stop is not confirmed\n";
    return MotionPlanRunCode::kNotConfirmed;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    bool ignored = false;
    if (poll != nullptr) {
      poll(session, poll_user, &ignored);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto latest = session->joint_motion_planner().latest_state();
    if (!latest || latest->request_id != request_id) {
      continue;
    }
    std::cout << "planner request_id=" << latest->request_id << " status=" << PlanStatusName(latest->status)
              << " progress=" << latest->progress << "\n";
    if (latest->status == engineai_sdk::JointMotionPlanStatus::kIdle) {
      std::cout << "robot confirmed planner idle after cancel\n";
      return MotionPlanRunCode::kCancelled;
    }
  }
  std::cout << "cancel was sent; robot did not confirm idle\n";
  return MotionPlanRunCode::kNotConfirmed;
}

inline MotionPlanRunCode RunMotionPlan(engineai_sdk::Robot* session, const MotionPlanFile& plan,
                                       const MotionPlanRunRequest& request, MotionPlanPoll poll, void* poll_user) {
  if (request.speed_scale <= 0.0) {
    std::cerr << "speed_scale must be > 0\n";
    return MotionPlanRunCode::kSendFailed;
  }
  if (request.dry_run) {
    for (const auto& task : plan.tasks) {
      const double seconds = task.reset ? 0.0 : task.duration / request.speed_scale;
      std::cout << "dry_run step=" << task.name << " type=" << (task.reset ? "RESET" : "PLAN_EXECUTE")
                << " duration_s=" << seconds << "\n";
    }
    return MotionPlanRunCode::kDryRun;
  }
  if (session == nullptr) {
    return MotionPlanRunCode::kNoState;
  }

  engineai_sdk::JointMotionPlanState state;
  const auto discover_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  bool have_state = false;
  while (std::chrono::steady_clock::now() < discover_deadline) {
    bool cancel = g_stop != 0;
    if (poll != nullptr) {
      poll(session, poll_user, &cancel);
    }
    if (cancel) {
      return MotionPlanRunCode::kCancelled;
    }
    if (const auto latest = session->joint_motion_planner().latest_state()) {
      state = *latest;
      have_state = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!have_state) {
    std::cerr << "timed out waiting for joint_motion_planner.latest_state\n";
    return MotionPlanRunCode::kNoState;
  }
  if (state.status != engineai_sdk::JointMotionPlanStatus::kIdle) {
    std::cerr << "planner not IDLE, status=" << PlanStatusName(state.status) << "\n";
    return MotionPlanRunCode::kNotIdle;
  }

  const auto run_start = std::chrono::steady_clock::now();
  for (size_t index = 0; index < plan.tasks.size(); ++index) {
    bool cancel = g_stop != 0;
    if (poll != nullptr) {
      poll(session, poll_user, &cancel);
    }
    if (request.cancel_after_ms >= 0) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - run_start);
      if (elapsed.count() >= request.cancel_after_ms) {
        cancel = true;
      }
    }
    if (cancel) {
      std::cout << "cancel before step " << plan.tasks[index].name << "\n";
      return MotionPlanRunCode::kCancelled;
    }

    const auto latest = session->joint_motion_planner().latest_state();
    if (!latest) {
      std::cerr << "no planner state; cannot send " << plan.tasks[index].name << "\n";
      return MotionPlanRunCode::kNoState;
    }
    engineai_sdk::JointMotionPlanRequest req;
    req.request_id = latest->request_id + 1;
    const int32_t sent_request_id = req.request_id;
    const MotionPlanTask& task = plan.tasks[index];
    const double scaled = task.reset ? 0.0 : task.duration / request.speed_scale;
    double wait_s = task.reset ? 15.0 : std::max(10.0, scaled * 3.0 + 5.0);
    if (request.timeout_s > 0.0) {
      wait_s = request.timeout_s;
    }
    if (task.reset) {
      req.request_type = engineai_sdk::JointMotionPlanRequestType::kReset;
      std::cout << "send RESET " << task.name << " request_id=" << sent_request_id << "\n";
    } else {
      req.request_type = engineai_sdk::JointMotionPlanRequestType::kPlanExecute;
      req.use_gravity_compensation = task.use_gravity_compensation;
      req.joint_indices = plan.joint_indices;
      req.target_positions = task.target_positions;
      req.execution_time = scaled;
      req.stiffness = task.stiffness;
      req.damping = task.damping;
      std::cout << "send PLAN_EXECUTE " << task.name << " request_id=" << sent_request_id
                << " duration_s=" << scaled << "\n";
    }
    const auto sent_at = session->joint_motion_planner().last_receive_time();
    if (!CheckOk(session->joint_motion_planner().execute(req), "joint_motion_planner.execute")) {
      return MotionPlanRunCode::kSendFailed;
    }

    const auto task_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(wait_s * 1000.0));
    bool step_done = false;
    int32_t logged_request_id = -1;
    auto logged_status = engineai_sdk::JointMotionPlanStatus::kDisabled;
    while (std::chrono::steady_clock::now() < task_deadline) {
      bool stop_now = g_stop != 0;
      if (poll != nullptr) {
        poll(session, poll_user, &stop_now);
      }
      if (request.cancel_after_ms >= 0) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - run_start);
        if (elapsed.count() >= request.cancel_after_ms) {
          stop_now = true;
        }
      }
      if (stop_now) {
        return ConfirmCancel(session, sent_request_id, poll, poll_user);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const auto sample = session->joint_motion_planner().latest_state();
      if (!sample || sample->request_id != sent_request_id) {
        continue;
      }
      if (sample->request_id != logged_request_id || sample->status != logged_status) {
        logged_request_id = sample->request_id;
        logged_status = sample->status;
        std::string motion = "(no sample)";
        if (const auto name = session->motion().current_motion()) {
          motion = *name;
        }
        std::cout << "planner request_id=" << sample->request_id << " status=" << PlanStatusName(sample->status)
                  << " progress=" << sample->progress << " motion=" << motion << "\n";
      }
      if (sample->status == engineai_sdk::JointMotionPlanStatus::kDisabled) {
        std::cerr << "planner DISABLED while waiting for " << task.name << " request_id=" << sent_request_id << "\n";
        return MotionPlanRunCode::kPlannerDisabled;
      }
      const auto received_at = session->joint_motion_planner().last_receive_time();
      const bool newer = received_at && (!sent_at || *received_at > *sent_at);
      const bool idle = sample->status == engineai_sdk::JointMotionPlanStatus::kIdle;
      const bool reset_settled = task.reset && sample->progress >= 1.0;
      const bool finished = idle && sample->progress >= 1.0;
      if (newer && (finished || reset_settled)) {
        std::cout << "completed " << task.name << " (" << (index + 1) << "/" << plan.tasks.size() << ")\n";
        step_done = true;
        break;
      }
    }
    if (!step_done) {
      const auto sample = session->joint_motion_planner().latest_state();
      std::cerr << "timed out waiting for " << task.name << " request_id=" << sent_request_id;
      if (sample) {
        std::cerr << " planner request_id=" << sample->request_id << " status=" << PlanStatusName(sample->status)
                  << " progress=" << sample->progress;
      }
      std::cerr << "\n";
      return MotionPlanRunCode::kTimeout;
    }
  }
  std::cout << "all motion plan tasks completed\n";
  return MotionPlanRunCode::kCompleted;
}

}  // namespace robot_session_examples
