// yaml_motion_config_check: local YAML check. This program does not send commands.
// Joint counts for --model come from examples/config/<model>/joint_test.yaml
// (pm01=24, t800=25, t800pro=43).
//
//   ./yaml_motion_config_check --file examples/config/pm01/motion_plan_wave_hands.yaml --model pm01 --check joint-plan

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/yaml_util.hpp"

namespace {

struct Issue {
  std::string field;
  std::string reason;
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --file PATH --check joint-plan|joint-command|override\n"
            << "       [--model pm01|t800|t800pro] [--domain-id N] [--interface IF] [--strict] [--json]\n"
            << "  Checks the file only. A failed check does not send a command.\n"
            << "  --model joint counts: pm01=24, t800=25, t800pro=43, from joint_test.yaml.\n";
}

void Add(std::vector<Issue>* issues, const std::string& field, const std::string& reason) {
  issues->push_back(Issue{field, reason});
}

int ModelJoints(const std::string& model) {
  if (model == "pm01") {
    return 24;
  }
  if (model == "t800") {
    return 25;
  }
  if (model == "t800pro") {
    return 43;
  }
  return -1;
}

void CheckSameSize(std::vector<Issue>* issues, const char* field, size_t actual, size_t expected) {
  if (actual != expected) {
    Add(issues, field, "length " + std::to_string(actual) + " != " + std::to_string(expected));
  }
}

void CheckIndices(std::vector<Issue>* issues, const std::vector<int32_t>& indices, int joint_count) {
  for (int32_t index : indices) {
    if (index < 0) {
      Add(issues, "joint_indices", "negative index " + std::to_string(index));
    } else if (joint_count >= 0 && index >= joint_count) {
      Add(issues, "joint_indices", "index " + std::to_string(index) + " >= joint count " + std::to_string(joint_count));
    }
  }
}

void CheckJointPlan(const YAML::Node& root, bool strict, int joint_count, std::vector<Issue>* issues) {
  const YAML::Node plan = root["motion_plan"];
  if (!plan || !plan.IsSequence() || plan.size() < 2) {
    Add(issues, "motion_plan", "need a sequence with joint_indices and at least one step");
    return;
  }
  std::vector<int32_t> indices;
  robot_session_examples::FlattenInts(plan[0]["joint_indices"], &indices);
  if (indices.empty()) {
    Add(issues, "joint_indices", "missing or empty");
  }
  CheckIndices(issues, indices, joint_count);
  for (size_t i = 1; i < plan.size(); ++i) {
    const YAML::Node item = plan[i];
    const std::string name = item["motion"] ? item["motion"].as<std::string>() : ("step_" + std::to_string(i));
    const std::string request_type =
        item["request_type"] ? item["request_type"].as<std::string>() : "";
    std::string upper = request_type;
    for (char& ch : upper) {
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    if (upper == "RESET") {
      continue;
    }
    std::vector<double> targets;
    std::vector<double> stiffness;
    std::vector<double> damping;
    robot_session_examples::FlattenDoubles(item["target_positions"], &targets);
    robot_session_examples::FlattenDoubles(item["stiffness"], &stiffness);
    robot_session_examples::FlattenDoubles(item["damping"], &damping);
    CheckSameSize(issues, (name + ".target_positions").c_str(), targets.size(), indices.size());
    double duration = -1.0;
    const bool duration_parsed =
        !item["duration"] || robot_session_examples::ParseFiniteDouble(item["duration"].Scalar(), &duration);
    if (!duration_parsed || !(duration > 0.0)) {
      Add(issues, name + ".duration", "execution time must be a finite number > 0");
    }
    if (item["use_gravity_compensation"]) {
      bool enabled = false;
      if (!robot_session_examples::ParseYamlBool(item["use_gravity_compensation"], &enabled)) {
        Add(issues, name + ".use_gravity_compensation", "must be true or false");
      }
    }
    if (stiffness.empty()) {
      if (strict) {
        Add(issues, name + ".stiffness", "missing");
      }
    } else {
      CheckSameSize(issues, (name + ".stiffness").c_str(), stiffness.size(), indices.size());
    }
    if (damping.empty()) {
      if (strict) {
        Add(issues, name + ".damping", "missing");
      }
    } else {
      CheckSameSize(issues, (name + ".damping").c_str(), damping.size(), indices.size());
    }
  }
}

void CheckJointCommand(const YAML::Node& root, int joint_count, std::vector<Issue>* issues) {
  std::vector<double> target;
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<int32_t> steps;
  robot_session_examples::FlattenDoubles(root["target_position"], &target);
  robot_session_examples::FlattenDoubles(root["kp"], &kp);
  robot_session_examples::FlattenDoubles(root["kd"], &kd);
  robot_session_examples::FlattenInts(root["num_steps"], &steps);
  if (target.empty()) {
    Add(issues, "target_position", "missing or empty");
  }
  CheckSameSize(issues, "kp", kp.size(), target.size());
  CheckSameSize(issues, "kd", kd.size(), target.size());
  CheckSameSize(issues, "num_steps", steps.size(), target.size());
  if (joint_count >= 0) {
    CheckSameSize(issues, "target_position", target.size(), static_cast<size_t>(joint_count));
  }
  for (size_t i = 0; i < steps.size(); ++i) {
    if (steps[i] < 2) {
      Add(issues, "num_steps", "index " + std::to_string(i) + " must be >= 2");
    }
  }
}

void CheckOverride(const YAML::Node& root, int joint_count, std::vector<Issue>* issues) {
  std::vector<int32_t> indices;
  std::vector<double> start_pos;
  std::vector<double> end_pos;
  std::vector<double> velocity;
  std::vector<double> feed_forward;
  std::vector<double> torque;
  std::vector<double> stiffness;
  std::vector<double> damping;
  robot_session_examples::FlattenInts(root["joint_indices"], &indices);
  robot_session_examples::FlattenDoubles(root["start_pos"], &start_pos);
  robot_session_examples::FlattenDoubles(root["end_pos"], &end_pos);
  robot_session_examples::FlattenDoubles(root["velocity"], &velocity);
  robot_session_examples::FlattenDoubles(root["feed_forward_torque"], &feed_forward);
  robot_session_examples::FlattenDoubles(root["torque"], &torque);
  robot_session_examples::FlattenDoubles(root["stiffness"], &stiffness);
  robot_session_examples::FlattenDoubles(root["damping"], &damping);
  if (indices.empty()) {
    Add(issues, "joint_indices", "missing or empty");
  }
  CheckIndices(issues, indices, joint_count);
  const size_t n = indices.size();
  CheckSameSize(issues, "start_pos", start_pos.size(), n);
  CheckSameSize(issues, "end_pos", end_pos.size(), n);
  CheckSameSize(issues, "velocity", velocity.size(), n);
  CheckSameSize(issues, "feed_forward_torque", feed_forward.size(), n);
  CheckSameSize(issues, "torque", torque.size(), n);
  if (stiffness.empty()) {
    Add(issues, "stiffness", "missing");
  } else {
    CheckSameSize(issues, "stiffness", stiffness.size(), n);
  }
  if (damping.empty()) {
    Add(issues, "damping", "missing");
  } else {
    CheckSameSize(issues, "damping", damping.size(), n);
  }
  double duration = -1.0;
  const bool duration_parsed =
      !root["duration"] || robot_session_examples::ParseFiniteDouble(root["duration"].Scalar(), &duration);
  if (!duration_parsed || !(duration > 0.0)) {
    Add(issues, "duration", "execution time must be a finite number > 0");
  }
  double freq = -1.0;
  const bool freq_parsed =
      !root["timer_freq"] || robot_session_examples::ParseFiniteDouble(root["timer_freq"].Scalar(), &freq);
  if (!freq_parsed || !(freq > 0.0)) {
    Add(issues, "timer_freq", "must be a finite number > 0");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (robot_session_examples::WantsHelp(argc, argv)) {
    PrintUsage(argv[0]);
    return 0;
  }

  std::string file;
  std::string model;
  std::string check;
  bool strict = false;
  bool as_json = false;
  bool have_domain = false;
  engineai_sdk::SessionOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--file" && i + 1 < argc) {
      file = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--check" && i + 1 < argc) {
      check = argv[++i];
    } else if (arg == "--strict") {
      strict = true;
    } else if (arg == "--json") {
      as_json = true;
    } else if (arg == "--domain-id" && i + 1 < argc) {
      int domain_id = 0;
      if (!robot_session_examples::ReadStrictInt(argv[++i], &domain_id, "domain-id")) {
        return 2;
      }
      options.dds_domain_id = domain_id;
      have_domain = true;
    } else if (arg == "--interface" && i + 1 < argc) {
      options.network_interface_name = argv[++i];
    } else if (arg == "--network-address" && i + 1 < argc) {
      options.network_address = argv[++i];
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (file.empty() || (check != "joint-plan" && check != "joint-command" && check != "override")) {
    PrintUsage(argv[0]);
    return 2;
  }
  if (!model.empty() && ModelJoints(model) < 0) {
    std::cerr << "model must be pm01, t800, or t800pro\n";
    return 2;
  }
  if (have_domain && options.dds_domain_id < 0) {
    std::cerr << "domain-id must be >= 0\n";
    return 2;
  }

  std::vector<Issue> issues;
  int joint_count = model.empty() ? -1 : ModelJoints(model);
  if (have_domain) {
    auto session = robot_session_examples::CreateSessionOrExit(options);
    engineai_sdk::JointState joint;
    if (!robot_session_examples::WaitJointState(session.get(), &joint)) {
      Add(&issues, "joint_state", "no sample");
    } else if (joint_count >= 0 && static_cast<int>(joint.position.size()) != joint_count) {
      Add(&issues, "joint_state", "sample length " + std::to_string(joint.position.size()) + " != model " + model +
                                      " count " + std::to_string(joint_count));
    } else {
      joint_count = static_cast<int>(joint.position.size());
    }
    session->close();
  }

  if (!robot_session_examples::FileExists(file)) {
    Add(&issues, "file", "not found: " + file);
  } else {
    try {
      const YAML::Node root = YAML::LoadFile(file);
      if (check == "joint-plan") {
        CheckJointPlan(root, strict, joint_count, &issues);
      } else if (check == "joint-command") {
        CheckJointCommand(root, joint_count, &issues);
      } else {
        CheckOverride(root, joint_count, &issues);
      }
    } catch (const std::exception& ex) {
      Add(&issues, "yaml", ex.what());
    }
  }

  if (as_json) {
    std::cout << "{\"ok\": " << (issues.empty() ? "true" : "false") << ", \"check\": "
              << robot_session_examples::JsonQuote(check) << ", \"errors\": [";
    for (size_t i = 0; i < issues.size(); ++i) {
      if (i != 0) {
        std::cout << ", ";
      }
      std::cout << "{\"field\": " << robot_session_examples::JsonQuote(issues[i].field)
                << ", \"reason\": " << robot_session_examples::JsonQuote(issues[i].reason) << "}";
    }
    std::cout << "]}\n";
  } else if (issues.empty()) {
    std::cout << "check=ok file=" << file << " kind=" << check;
    if (joint_count >= 0) {
      std::cout << " joint_count=" << joint_count;
    }
    std::cout << "\n";
  } else {
    std::cout << "check=failed file=" << file << " kind=" << check << "\n";
    for (const auto& issue : issues) {
      std::cout << "error field=" << issue.field << " reason=" << issue.reason << "\n";
    }
    std::cout << "no command sent\n";
  }
  return issues.empty() ? 0 : 1;
}
