// connectivity_doctor: local session and whether robot samples arrived.
//
// create() only opens local DDS. robot_sample=yes means at least one of
// heartbeat, motion, IMU, or power arrived (last_receive_time() is set).
// <topic>_since_receive is how long ago the newest sample arrived. The program waits up to 5 s so the
// slower heartbeat (about 1 s) and motion_state (about 200 ms) are not skipped
// just because IMU arrived first.
//
//   ./connectivity_doctor --domain-id 0 [--interface eth0] [--network-address 192.168.0.0]

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--timeout SEC] [--check heartbeat,motion,joint,imu,power] [--json]\n"
            << "  Default timeout is 5 s and every check is enabled.\n"
            << "  Domain and interface are the values passed in, not a discovered binding.\n";
}

void PrintHeartbeat(const engineai_sdk::Robot& session) {
  if (const auto hb = session.node_heartbeat().latest()) {
    std::cout << "node_heartbeat node=" << hb->node_name << " status=" << hb->node_status
              << " err=" << hb->error_code;
    if (!hb->error_message.empty()) {
      std::cout << " message=" << hb->error_message;
    }
    std::cout << "\n";
  } else {
    std::cout << "node_heartbeat (no sample)\n";
  }
}

void PrintMotion(const engineai_sdk::Robot& session) {
  std::cout << "current_motion=" << robot_session_examples::MotionOrNone(session) << "\n";
}

void PrintImu(const engineai_sdk::Robot& session) {
  if (const auto imu = session.imu().latest()) {
    std::cout << "imu rpy=(" << imu->rpy.x << ", " << imu->rpy.y << ", " << imu->rpy.z << ")\n";
  } else {
    std::cout << "imu (no sample)\n";
  }
}

void PrintPower(const engineai_sdk::Robot& session) {
  if (const auto power = session.power().latest()) {
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
}

void PrintJoint(const engineai_sdk::Robot& session) {
  engineai_sdk::JointState joint;
  if (session.joint_state().latest(joint)) {
    std::cout << "joint_state n=" << joint.position.size() << "\n";
  } else {
    std::cout << "joint_state (no sample)\n";
  }
}

struct Checks {
  bool heartbeat = true;
  bool motion = true;
  bool joint = true;
  bool imu = true;
  bool power = true;
};

bool ParseChecks(const std::string& text, Checks* checks) {
  checks->heartbeat = false;
  checks->motion = false;
  checks->joint = false;
  checks->imu = false;
  checks->power = false;
  std::string item;
  for (size_t i = 0; i <= text.size(); ++i) {
    if (i == text.size() || text[i] == ',') {
      if (item == "heartbeat") checks->heartbeat = true;
      else if (item == "motion") checks->motion = true;
      else if (item == "joint") checks->joint = true;
      else if (item == "imu") checks->imu = true;
      else if (item == "power") checks->power = true;
      else return false;
      item.clear();
    } else {
      item.push_back(text[i]);
    }
  }
  return checks->heartbeat || checks->motion || checks->joint || checks->imu || checks->power;
}

struct CheckEntry {
  const char* name;
  bool enabled;
};

std::vector<CheckEntry> CheckEntries(const Checks& checks) {
  return {
      {"heartbeat", checks.heartbeat},
      {"motion", checks.motion},
      {"joint", checks.joint},
      {"imu", checks.imu},
      {"power", checks.power},
  };
}

std::optional<engineai_sdk::ReceiveTime> ReceiveTime(const engineai_sdk::Robot& session, const char* name) {
  if (std::string(name) == "heartbeat") return session.node_heartbeat().last_receive_time();
  if (std::string(name) == "motion") return session.motion().last_receive_time();
  if (std::string(name) == "joint") return session.joint_state().last_receive_time();
  if (std::string(name) == "imu") return session.imu().last_receive_time();
  return session.power().last_receive_time();
}

bool SamplePresent(const engineai_sdk::Robot& session, const char* name) {
  return ReceiveTime(session, name).has_value();
}

bool SelectedReady(const engineai_sdk::Robot& session, const Checks& checks) {
  if (checks.heartbeat && !SamplePresent(session, "heartbeat")) return false;
  if (checks.motion && !SamplePresent(session, "motion")) return false;
  if (checks.joint && !SamplePresent(session, "joint")) return false;
  if (checks.imu && !SamplePresent(session, "imu")) return false;
  if (checks.power && !SamplePresent(session, "power")) return false;
  return true;
}

bool AnySelected(const engineai_sdk::Robot& session, const Checks& checks) {
  if (checks.heartbeat && SamplePresent(session, "heartbeat")) return true;
  if (checks.motion && SamplePresent(session, "motion")) return true;
  if (checks.joint && SamplePresent(session, "joint")) return true;
  if (checks.imu && SamplePresent(session, "imu")) return true;
  if (checks.power && SamplePresent(session, "power")) return true;
  return false;
}

void WaitForListedSamples(engineai_sdk::Robot* session, const Checks& checks, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!session->is_open() || SelectedReady(*session, checks)) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
  int timeout_sec = 5;
  bool as_json = false;
  Checks checks;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--json") {
      as_json = true;
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &timeout_sec, "timeout")) {
        return 2;
      }
    } else if (arg == "--check" && i + 1 < rest.size()) {
      if (!ParseChecks(rest[++i], &checks)) {
        std::cerr << "--check must be a comma list of heartbeat,motion,joint,imu,power\n";
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

  const std::string interface =
      options.network_interface_name.empty() ? "(empty)" : options.network_interface_name;
  const std::string address = options.network_address.empty() ? "(empty)" : options.network_address;

  auto result = engineai_sdk::Robot::create(options);
  if (!result) {
    const std::string message = "local DDS session was not created. Check the domain id and interface name.";
    if (as_json) {
      std::cout << "{\"session\": false, \"robot_sample\": false, \"error\": "
                << robot_session_examples::JsonQuote(robot_session_examples::ErrName(result.error))
                << ", \"dds_domain_id\": " << options.dds_domain_id
                << ", \"network_interface\": " << robot_session_examples::JsonQuote(interface)
                << ", \"network_address\": " << robot_session_examples::JsonQuote(address)
                << ", \"message\": " << robot_session_examples::JsonQuote(message) << "}\n";
    } else {
      std::cout << "session=failed error=" << robot_session_examples::ErrName(result.error) << "\n";
      std::cout << "stage=session_create dds_domain_id=" << options.dds_domain_id
                << " network_interface=" << interface << " network_address=" << address << "\n";
      std::cout << "hint: " << message << "\n";
    }
    return 1;
  }

  auto session = std::move(result.session);
  WaitForListedSamples(session.get(), checks, timeout_sec * 1000);
  const bool robot_sample = AnySelected(*session, checks);
  const bool ready = SelectedReady(*session, checks);
  const std::vector<CheckEntry> entries = CheckEntries(checks);
  std::string missing;
  for (const CheckEntry& entry : entries) {
    if (entry.enabled && !SamplePresent(*session, entry.name)) {
      if (!missing.empty()) {
        missing += " ";
      }
      missing += entry.name;
    }
  }
  if (as_json) {
    std::cout << "{\"session\": " << (session->is_open() ? "true" : "false")
              << ", \"robot_sample\": " << (robot_sample ? "true" : "false")
              << ", \"dds_domain_id\": " << options.dds_domain_id
              << ", \"network_interface\": " << robot_session_examples::JsonQuote(interface)
              << ", \"checks\": {";
    bool first = true;
    for (const CheckEntry& entry : entries) {
      if (!entry.enabled) continue;
      if (!first) std::cout << ", ";
      first = false;
      std::cout << "\"" << entry.name << "\": " << (SamplePresent(*session, entry.name) ? "true" : "false");
    }
    std::cout << "}";
    if (!ready) {
      std::cout << ", \"message\": " << robot_session_examples::JsonQuote("missing " + missing);
    }
    std::cout << "}\n";
  } else {
    std::cout << "session=" << (session->is_open() ? "ok" : "failed") << "\n";
    std::cout << "robot_sample=" << (robot_sample ? "yes" : "no") << "\n";
    std::cout << "dds_domain_id=" << options.dds_domain_id << "\n";
    std::cout << "network_interface=" << interface << "\n";
    std::cout << "network_address=" << address << "\n";
    if (checks.heartbeat) PrintHeartbeat(*session);
    if (checks.motion) PrintMotion(*session);
    if (checks.joint) PrintJoint(*session);
    if (checks.imu) PrintImu(*session);
    if (checks.power) PrintPower(*session);
    for (const CheckEntry& entry : entries) {
      if (entry.enabled) {
        std::cout << entry.name << "_since_receive="
                  << robot_session_examples::SinceReceiveText(ReceiveTime(*session, entry.name)) << "\n";
      }
    }
  }

  if (!ready) {
    if (as_json) {
      std::cerr << "stage=sample missing=" << missing << "\n";
    } else {
      std::cout << "stage=sample missing=" << missing << "\n";
    }
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }

  session->close();
  return 0;
}
