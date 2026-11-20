// robot_status_snapshot: one readable status table, or JSON with --json.
//
// Waits up to 5 s so the 1 s heartbeat is not skipped behind IMU. Alerts use
// the robot health-manager limits: battery percentage <= 20 (critical <= 5),
// battery temperature >= 55 C, motor warning codes 0x3401 / 0x3403, and
// motor offline / driver error / node error.
//
//   ./robot_status_snapshot --domain-id 0 [--interface IF] [--json]

#include <unistd.h>

#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

constexpr int kWaitMs = 5000;
constexpr double kLowSocPercent = 20.0;
constexpr double kCriticalSocPercent = 5.0;
constexpr double kMaxBatteryTempC = 55.0;
constexpr unsigned kMosOverTemperature = 0x3401;
constexpr unsigned kMotorOverTemperature = 0x3403;
constexpr unsigned kDriverErrorBit = 0x0800;

struct Alert {
  std::string item;
  std::string code;
  std::string detail;
};

struct Row {
  std::string item;
  bool alert = false;
  std::string detail;
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--json] [--once] [--watch] [--interval SEC] [--show-diagnostics]\n"
            << "  Prints one status table. --watch repeats until Ctrl+C. --once is the default.\n"
            << "  Exit 1 on a single snapshot when the session fails, no robot sample arrives, or any alert is set.\n"
            << "  A cached sample older than its threshold is marked stale and counts as an alert.\n";
}

std::string JsonEscape(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (unsigned char ch : text) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (ch < 0x20) {
          std::ostringstream hex;
          hex << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch);
          out += hex.str();
        } else {
          out.push_back(static_cast<char>(ch));
        }
        break;
    }
  }
  return out;
}

std::string JsonString(const std::string& text) { return "\"" + JsonEscape(text) + "\""; }

std::string JsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out << value;
  return out.str();
}

std::string IndexList(const std::vector<int>& indices) {
  std::ostringstream out;
  for (size_t i = 0; i < indices.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << indices[i];
  }
  return out.str();
}

unsigned DriverCode(int32_t code) { return static_cast<unsigned>(code) & 0xFFFF; }

std::string HexCode(unsigned code) {
  std::ostringstream out;
  out << "0x" << std::hex << code;
  return out.str();
}

bool HasSnapshotSource(const engineai_sdk::Robot& session) {
  return session.node_heartbeat().latest() || session.motion().current_motion() || session.imu().latest() ||
         session.power().latest() || session.motor_diagnostics().latest();
}

bool HasAllSnapshotSources(const engineai_sdk::Robot& session) {
  return session.node_heartbeat().latest() && session.motion().current_motion() && session.imu().latest() &&
         session.power().latest() && session.motor_diagnostics().latest();
}

void WaitForSnapshot(engineai_sdk::Robot* session) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kWaitMs);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!session->is_open() || HasAllSnapshotSources(*session)) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void Add(std::vector<Alert>* alerts, const char* item, const char* code, const std::string& detail) {
  alerts->push_back(Alert{item, code, detail});
}

bool ItemAlert(const std::vector<Alert>& alerts, const std::string& item) {
  for (const auto& alert : alerts) {
    if (alert.item == item) {
      return true;
    }
  }
  return false;
}

long long AgeMs(const std::optional<engineai_sdk::ReceiveTime>& received) {
  if (!received) {
    return -1;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - *received).count();
}

void NoteStale(const char* item, const std::optional<engineai_sdk::ReceiveTime>& received,
               std::chrono::steady_clock::duration max_age, std::vector<Alert>* alerts, std::string* detail) {
  if (robot_session_examples::IsFresh(received, max_age)) {
    return;
  }
  const std::string note = "since_receive_ms=" + std::to_string(AgeMs(received));
  if (!detail->empty()) {
    *detail += " ";
  }
  *detail += note;
  Add(alerts, item, "stale", note);
}

void PrintFreshness(const std::optional<engineai_sdk::ReceiveTime>& received,
                    std::chrono::steady_clock::duration max_age) {
  const bool stale = !robot_session_examples::IsFresh(received, max_age);
  std::cout << ", \"stale\": " << (stale ? "true" : "false") << ", \"since_receive_ms\": " << AgeMs(received);
}

struct MotorFlags {
  std::vector<int> offline;
  std::vector<int> over_temp;
  std::vector<int> fault;
  std::vector<int> warning;
  float max_motor = 0.0F;
  float max_mos = 0.0F;
  bool have_motor_temp = false;
  bool have_mos_temp = false;
};

MotorFlags ClassifyMotors(const engineai_sdk::MotorDebug& motors) {
  MotorFlags flags;
  for (size_t i = 0; i < motors.offline.size(); ++i) {
    if (motors.offline[i] != 0) {
      flags.offline.push_back(static_cast<int>(i));
    }
  }
  for (size_t i = 0; i < motors.error_code.size(); ++i) {
    const unsigned code = DriverCode(motors.error_code[i]);
    if (code == 0) {
      continue;
    }
    if (code == kMosOverTemperature || code == kMotorOverTemperature) {
      flags.over_temp.push_back(static_cast<int>(i));
    } else if ((code & kDriverErrorBit) != 0) {
      flags.fault.push_back(static_cast<int>(i));
    } else {
      flags.warning.push_back(static_cast<int>(i));
    }
  }
  for (float temp : motors.motor_temperature) {
    if (!flags.have_motor_temp || temp > flags.max_motor) {
      flags.max_motor = temp;
      flags.have_motor_temp = true;
    }
  }
  for (float temp : motors.mos_temperature) {
    if (!flags.have_mos_temp || temp > flags.max_mos) {
      flags.max_mos = temp;
      flags.have_mos_temp = true;
    }
  }
  return flags;
}

void CollectHeartbeat(const engineai_sdk::Robot& session, std::vector<Alert>* alerts, Row* row) {
  const auto hb = session.node_heartbeat().latest();
  if (!hb) {
    Add(alerts, "heartbeat", "no_sample", "no sample");
    row->detail = "no sample";
    return;
  }
  std::ostringstream detail;
  detail << "node=" << hb->node_name << " status=" << hb->node_status << " err=" << hb->error_code;
  if (!hb->error_message.empty()) {
    detail << " message=" << hb->error_message;
  }
  if (hb->node_status == "error" || hb->error_code != 0 || !hb->error_message.empty()) {
    Add(alerts, "heartbeat", "node_error", detail.str());
  }
  row->detail = detail.str();
  NoteStale("heartbeat", session.node_heartbeat().last_receive_time(), robot_session_examples::kHeartbeatMaxAge,
            alerts, &row->detail);
}

void CollectMotion(const engineai_sdk::Robot& session, std::vector<Alert>* alerts, Row* row) {
  const auto motion = session.motion().current_motion();
  if (!motion) {
    Add(alerts, "motion", "no_sample", "no sample");
    row->detail = "no sample";
    return;
  }
  row->detail = *motion;
  NoteStale("motion", session.motion().last_receive_time(), robot_session_examples::kMotionMaxAge, alerts,
            &row->detail);
}

void CollectImu(const engineai_sdk::Robot& session, std::vector<Alert>* alerts, Row* row) {
  const auto imu = session.imu().latest();
  if (!imu) {
    Add(alerts, "imu", "no_sample", "no sample");
    row->detail = "no sample";
    return;
  }
  std::ostringstream detail;
  detail << "rpy=(" << imu->rpy.x << ", " << imu->rpy.y << ", " << imu->rpy.z << ")";
  row->detail = detail.str();
  NoteStale("imu", session.imu().last_receive_time(), robot_session_examples::kStateMaxAge, alerts, &row->detail);
}

void CollectPower(const engineai_sdk::Robot& session, std::vector<Alert>* alerts, Row* row) {
  const auto power = session.power().latest();
  if (!power) {
    Add(alerts, "power", "no_sample", "no sample");
    row->detail = "no sample";
    return;
  }
  const auto& board = power->power_info;
  std::ostringstream detail;
  detail << "enable=" << board.enable << " pct=" << board.percentage << " V=" << board.voltage
         << " A=" << board.current << " err=" << board.error_code;
  if (board.error_code != 0) {
    Add(alerts, "power", "power_fault", "error_code=" + std::to_string(board.error_code));
  }
  if (std::isfinite(board.percentage) && board.percentage >= 0.0 && board.percentage <= 100.0) {
    if (board.percentage <= kCriticalSocPercent) {
      Add(alerts, "power", "critical_battery", "percentage=" + std::to_string(board.percentage));
    } else if (board.percentage <= kLowSocPercent) {
      Add(alerts, "power", "low_battery", "percentage=" + std::to_string(board.percentage));
    }
  }
  if (power->battery_info) {
    const auto& battery = *power->battery_info;
    detail << " soc=" << static_cast<unsigned>(battery.soc) << " temp_c=" << battery.temp;
    if (std::isfinite(battery.temp) && battery.temp >= kMaxBatteryTempC) {
      Add(alerts, "power", "battery_over_temperature", "temp_c=" + std::to_string(battery.temp));
    }
    if (battery.error_word != 0) {
      Add(alerts, "power", "battery_error", "error_word=" + std::to_string(battery.error_word));
    }
  } else {
    detail << " bms=unavailable";
  }
  row->detail = detail.str();
  NoteStale("power", session.power().last_receive_time(), robot_session_examples::kStateMaxAge, alerts, &row->detail);
}

void CollectMotors(const engineai_sdk::Robot& session, std::vector<Alert>* alerts, Row* row) {
  const auto motors = session.motor_diagnostics().latest();
  if (!motors) {
    Add(alerts, "motors", "no_sample", "no sample");
    row->detail = "no sample";
    return;
  }
  const MotorFlags flags = ClassifyMotors(*motors);
  if (!flags.offline.empty()) {
    Add(alerts, "motors", "motor_offline", "indices=" + IndexList(flags.offline));
  }
  if (!flags.over_temp.empty()) {
    Add(alerts, "motors", "motor_over_temperature", "indices=" + IndexList(flags.over_temp));
  }
  if (!flags.fault.empty()) {
    Add(alerts, "motors", "motor_fault", "indices=" + IndexList(flags.fault));
  }
  if (!flags.warning.empty()) {
    Add(alerts, "motors", "motor_warning", "indices=" + IndexList(flags.warning));
  }
  std::ostringstream detail;
  detail << "n=" << motors->offline.size() << " offline=" << flags.offline.size()
         << " over_temp=" << flags.over_temp.size() << " fault=" << flags.fault.size()
         << " warning=" << flags.warning.size();
  if (flags.have_motor_temp) {
    detail << " max_motor_c=" << flags.max_motor;
  }
  if (flags.have_mos_temp) {
    detail << " max_mos_c=" << flags.max_mos;
  }
  row->detail = detail.str();
  NoteStale("motors", session.motor_diagnostics().last_receive_time(), robot_session_examples::kStateMaxAge, alerts,
            &row->detail);
}

void PrintTable(const std::vector<Row>& rows, const std::vector<Alert>& alerts) {
  const bool color = isatty(STDOUT_FILENO) == 1;
  std::cout << std::left << std::setw(12) << "item" << std::setw(8) << "status" << "detail\n";
  for (const auto& row : rows) {
    std::cout << std::setw(12) << row.item;
    if (row.alert && color) {
      std::cout << "\033[31m";
    }
    std::cout << std::setw(8) << (row.alert ? "ALERT" : "ok");
    if (row.alert && color) {
      std::cout << "\033[0m";
    }
    std::cout << row.detail << "\n";
  }
  std::cout << "alerts=" << alerts.size() << "\n";
  for (const auto& alert : alerts) {
    std::cout << "alert item=" << alert.item << " code=" << alert.code << " " << alert.detail << "\n";
  }
}

void PrintJson(bool session_ok, const std::string& session_error, int domain_id, const std::string& interface_name,
               const engineai_sdk::Robot* session, const std::vector<Alert>& alerts) {
  std::cout << "{\n";
  std::cout << "  \"session_ok\": " << (session_ok ? "true" : "false") << ",\n";
  std::cout << "  \"session_error\": " << (session_error.empty() ? "null" : JsonString(session_error)) << ",\n";
  std::cout << "  \"dds_domain_id\": " << domain_id << ",\n";
  std::cout << "  \"network_interface\": " << JsonString(interface_name) << ",\n";
  std::cout << "  \"alert_count\": " << alerts.size() << ",\n";
  std::cout << "  \"alerts\": [";
  for (size_t i = 0; i < alerts.size(); ++i) {
    if (i != 0) {
      std::cout << ",";
    }
    std::cout << "\n    {\"item\": " << JsonString(alerts[i].item) << ", \"code\": " << JsonString(alerts[i].code)
              << ", \"detail\": " << JsonString(alerts[i].detail) << "}";
  }
  if (!alerts.empty()) {
    std::cout << "\n  ";
  }
  std::cout << "],\n";

  if (session == nullptr) {
    std::cout << "  \"heartbeat\": null,\n  \"motion\": null,\n  \"imu\": null,\n  \"power\": null,\n  \"motors\": null\n}\n";
    return;
  }

  if (const auto hb = session->node_heartbeat().latest()) {
    std::cout << "  \"heartbeat\": {\"sample\": true, \"node\": " << JsonString(hb->node_name)
              << ", \"status\": " << JsonString(hb->node_status) << ", \"error_code\": " << hb->error_code
              << ", \"error_message\": " << JsonString(hb->error_message);
    PrintFreshness(session->node_heartbeat().last_receive_time(), robot_session_examples::kHeartbeatMaxAge);
    std::cout << "},\n";
  } else {
    std::cout << "  \"heartbeat\": {\"sample\": false},\n";
  }

  if (const auto motion = session->motion().current_motion()) {
    std::cout << "  \"motion\": {\"sample\": true, \"name\": " << JsonString(*motion);
    PrintFreshness(session->motion().last_receive_time(), robot_session_examples::kMotionMaxAge);
    std::cout << "},\n";
  } else {
    std::cout << "  \"motion\": {\"sample\": false},\n";
  }

  if (const auto imu = session->imu().latest()) {
    std::cout << "  \"imu\": {\"sample\": true, \"roll\": " << JsonNumber(imu->rpy.x)
              << ", \"pitch\": " << JsonNumber(imu->rpy.y) << ", \"yaw\": " << JsonNumber(imu->rpy.z);
    PrintFreshness(session->imu().last_receive_time(), robot_session_examples::kStateMaxAge);
    std::cout << "},\n";
  } else {
    std::cout << "  \"imu\": {\"sample\": false},\n";
  }

  if (const auto power = session->power().latest()) {
    const auto& board = power->power_info;
    std::cout << "  \"power\": {\"sample\": true, \"enable\": " << (board.enable ? "true" : "false")
              << ", \"percentage\": " << JsonNumber(board.percentage) << ", \"voltage\": " << JsonNumber(board.voltage)
              << ", \"current\": " << JsonNumber(board.current) << ", \"error_code\": " << board.error_code;
    if (power->battery_info) {
      const auto& battery = *power->battery_info;
      std::cout << ", \"battery\": {\"soc\": " << static_cast<unsigned>(battery.soc)
                << ", \"temp_c\": " << JsonNumber(battery.temp) << ", \"error_word\": " << battery.error_word << "}";
    } else {
      std::cout << ", \"battery\": null";
    }
    PrintFreshness(session->power().last_receive_time(), robot_session_examples::kStateMaxAge);
    std::cout << "},\n";
  } else {
    std::cout << "  \"power\": {\"sample\": false},\n";
  }

  if (const auto motors = session->motor_diagnostics().latest()) {
    const MotorFlags flags = ClassifyMotors(*motors);
    auto print_codes = [&](const std::vector<int>& indices) {
      for (size_t i = 0; i < indices.size(); ++i) {
        if (i != 0) {
          std::cout << ", ";
        }
        const unsigned code = DriverCode(motors->error_code[static_cast<size_t>(indices[i])]);
        std::cout << "{\"index\": " << indices[i] << ", \"code\": " << JsonString(HexCode(code)) << "}";
      }
    };
    std::cout << "  \"motors\": {\"sample\": true, \"count\": " << motors->offline.size() << ", \"offline\": [";
    for (size_t i = 0; i < flags.offline.size(); ++i) {
      if (i != 0) {
        std::cout << ", ";
      }
      std::cout << flags.offline[i];
    }
    std::cout << "], \"over_temperature\": [";
    print_codes(flags.over_temp);
    std::cout << "], \"fault\": [";
    print_codes(flags.fault);
    std::cout << "], \"warning\": [";
    print_codes(flags.warning);
    std::cout << "]";
    if (flags.have_motor_temp) {
      std::cout << ", \"max_motor_c\": " << JsonNumber(flags.max_motor);
    }
    if (flags.have_mos_temp) {
      std::cout << ", \"max_mos_c\": " << JsonNumber(flags.max_mos);
    }
    PrintFreshness(session->motor_diagnostics().last_receive_time(), robot_session_examples::kStateMaxAge);
    std::cout << "}\n";
  } else {
    std::cout << "  \"motors\": {\"sample\": false}\n";
  }
  std::cout << "}\n";
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
  bool as_json = false;
  bool watch = false;
  bool once = false;
  bool show_diagnostics = false;
  double interval = 1.0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--json") {
      as_json = true;
    } else if (arg == "--watch") {
      watch = true;
    } else if (arg == "--once") {
      once = true;
    } else if (arg == "--show-diagnostics") {
      show_diagnostics = true;
    } else if (arg == "--interval" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &interval, "interval")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (watch && once) {
    std::cerr << "pass only one of --watch and --once\n";
    return 2;
  }
  if (interval <= 0.0) {
    std::cerr << "interval must be > 0\n";
    return 2;
  }

  const std::string interface =
      options.network_interface_name.empty() ? "(empty)" : options.network_interface_name;
  auto result = engineai_sdk::Robot::create(options);
  if (!result) {
    const std::string error = robot_session_examples::ErrName(result.error);
    std::vector<Alert> alerts{{"session", "session_failed", error}};
    if (as_json) {
      PrintJson(false, error, options.dds_domain_id, interface, nullptr, alerts);
    } else {
      PrintTable({{"session", true, "failed error=" + error}}, alerts);
    }
    return 1;
  }

  auto session = std::move(result.session);
  WaitForSnapshot(session.get());
  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(interval));
  bool clean = true;
  do {
    const bool any_sample = HasSnapshotSource(*session);
    std::vector<Alert> alerts;
    Row heartbeat{"heartbeat", false, ""};
    Row motion{"motion", false, ""};
    Row joints{"joints", false, ""};
    Row imu{"imu", false, ""};
    Row gamepad{"gamepad", false, ""};
    Row power{"power", false, ""};
    Row motors{"motors", false, ""};
    CollectHeartbeat(*session, &alerts, &heartbeat);
    CollectMotion(*session, &alerts, &motion);
    engineai_sdk::JointState joint;
    if (session->joint_state().latest(joint)) {
      joints.detail = "n=" + std::to_string(joint.position.size());
      NoteStale("joints", session->joint_state().last_receive_time(), robot_session_examples::kStateMaxAge, &alerts,
                &joints.detail);
    } else {
      Add(&alerts, "joints", "no_sample", "no sample");
      joints.detail = "no sample";
    }
    CollectImu(*session, &alerts, &imu);
    if (const auto pad = session->gamepad().latest()) {
      gamepad.detail = std::string("connected=") + (pad->hardware_connected ? "true" : "false");
      NoteStale("gamepad", session->gamepad().last_receive_time(), robot_session_examples::kStateMaxAge, &alerts,
                &gamepad.detail);
      if (!pad->hardware_connected) {
        Add(&alerts, "gamepad", "not_connected", "hardware_connected=false");
      }
    } else {
      Add(&alerts, "gamepad", "no_sample", "no sample");
      gamepad.detail = "no sample";
    }
    CollectPower(*session, &alerts, &power);
    CollectMotors(*session, &alerts, &motors);
    if (!any_sample && !session->joint_state().latest(joint) && !session->gamepad().latest()) {
      Add(&alerts, "session", "no_robot_sample", "no heartbeat, motion, imu, power, motor, joint, or gamepad sample");
    }
    heartbeat.alert = ItemAlert(alerts, "heartbeat");
    motion.alert = ItemAlert(alerts, "motion");
    joints.alert = ItemAlert(alerts, "joints");
    imu.alert = ItemAlert(alerts, "imu");
    gamepad.alert = ItemAlert(alerts, "gamepad");
    power.alert = ItemAlert(alerts, "power");
    motors.alert = ItemAlert(alerts, "motors");
    const bool session_alert = ItemAlert(alerts, "session");
    std::vector<Row> rows{{"session", session_alert, session->is_open() ? "ok" : "not open"},
                          heartbeat,
                          motion,
                          joints,
                          imu,
                          gamepad,
                          power,
                          motors};
    if (show_diagnostics) {
      if (const auto diag = session->motor_diagnostics().latest()) {
        const MotorFlags flags = ClassifyMotors(*diag);
        rows.push_back(Row{"diag", false, "offline=" + IndexList(flags.offline) + " over_temp=" + IndexList(flags.over_temp)});
      }
    }
    if (!as_json) {
      std::cout << "dds_domain_id=" << options.dds_domain_id << "\n";
      std::cout << "network_interface=" << interface << "\n";
      PrintTable(rows, alerts);
    } else {
      PrintJson(session->is_open(), "", options.dds_domain_id, interface, session.get(), alerts);
    }
    if (!any_sample) {
      robot_session_examples::PrintNoRobotDataHint();
    }
    clean = alerts.empty();
    if (!watch || robot_session_examples::g_stop) {
      break;
    }
    std::this_thread::sleep_for(period);
  } while (!robot_session_examples::g_stop);

  session->close();
  return clean ? 0 : 1;
}
