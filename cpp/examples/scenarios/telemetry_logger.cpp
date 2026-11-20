// telemetry_logger: write polled robot samples to CSV or JSONL.
// timestamp_ms is the local poll time. joint_since_receive_ms is how long ago the joint
// sample arrived; joint_new is 1 when it differs from the previous row's sample
// (the last receive time is kept and compared).
//
//   ./telemetry_logger --domain-id 0 --duration 5 --rate 10 --output /tmp/robot.csv

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

struct FieldSet {
  bool joint = true;
  bool motor = true;
  bool imu = true;
  bool motion = true;
  bool power = true;
  bool diagnostics = true;
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       --duration SEC --output PATH [--rate HZ] [--format csv|jsonl]\n"
            << "       [--fields joint,motor,imu,motion,power,diagnostics]\n"
            << "  Default rate=10 format=csv fields=all.\n";
}

std::string Join(const std::vector<double>& values) {
  std::ostringstream out;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      out << "|";
    }
    out << values[i];
  }
  return out.str();
}

bool Has(const FieldSet& fields, const std::string& name) {
  if (name == "joint") return fields.joint;
  if (name == "motor") return fields.motor;
  if (name == "imu") return fields.imu;
  if (name == "motion") return fields.motion;
  if (name == "power") return fields.power;
  if (name == "diagnostics") return fields.diagnostics;
  return false;
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

  double duration = 0.0;
  double rate = 10.0;
  std::string output;
  std::string format = "csv";
  FieldSet fields;
  bool fields_set = false;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--duration" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &duration, "duration")) {
        return 2;
      }
    } else if (arg == "--rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &rate, "rate")) {
        return 2;
      }
    } else if (arg == "--output" && i + 1 < rest.size()) {
      output = rest[++i];
    } else if (arg == "--format" && i + 1 < rest.size()) {
      format = rest[++i];
    } else if (arg == "--fields" && i + 1 < rest.size()) {
      fields.joint = false;
      fields.motor = false;
      fields.imu = false;
      fields.motion = false;
      fields.power = false;
      fields.diagnostics = false;
      fields_set = true;
      std::stringstream stream(rest[++i]);
      std::string item;
      while (std::getline(stream, item, ',')) {
        if (item == "joint") fields.joint = true;
        else if (item == "motor") fields.motor = true;
        else if (item == "imu") fields.imu = true;
        else if (item == "motion") fields.motion = true;
        else if (item == "power") fields.power = true;
        else if (item == "diagnostics") fields.diagnostics = true;
        else {
          std::cerr << "unknown field " << item << "\n";
          return 2;
        }
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (!fields_set) {
    fields = {true, true, true, true, true, true};
  }
  if (duration <= 0.0 || rate <= 0.0 || output.empty() || (format != "csv" && format != "jsonl")) {
    PrintUsage(argv[0]);
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::WaitForRobotSample(session.get())) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }
  std::ofstream out(output);
  if (!out) {
    std::cerr << "cannot open " << output << "\n";
    session->close();
    return 1;
  }
  if (format == "csv") {
    out << "timestamp_ms";
    if (fields.joint) out << ",missing_joint,joint_since_receive_ms,joint_new,joint_n,joint_q";
    if (fields.motor) out << ",missing_motor,motor_n";
    if (fields.imu) out << ",missing_imu,roll,pitch,yaw";
    if (fields.motion) out << ",missing_motion,motion";
    if (fields.power) out << ",missing_power,power_pct";
    if (fields.diagnostics) out << ",missing_diagnostics,motor_offline_n";
    out << "\n";
  }

  int rows = 0;
  int invalid_numbers = 0;
  int missing_joint = 0;
  int missing_motor = 0;
  int missing_imu = 0;
  int missing_motion = 0;
  int missing_power = 0;
  int missing_diag = 0;
  bool have_power = false;
  bool have_roll = false;
  double min_power = 0.0;
  double max_power = 0.0;
  double min_roll = 0.0;
  double max_roll = 0.0;
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(duration));
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate));
  robot_session_examples::CatchSigInt();
  std::optional<engineai_sdk::ReceiveTime> prev_joint_time;

  while (!robot_session_examples::g_stop && std::chrono::steady_clock::now() < deadline) {
    const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                           .count();
    // Time first, then the value: the value is at least as new as the time.
    const auto joint_time = fields.joint ? session->joint_state().last_receive_time() : std::nullopt;
    const bool joint_new = joint_time && joint_time != prev_joint_time;
    prev_joint_time = joint_time;
    const long long joint_since_receive_ms =
        joint_time ? std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - *joint_time)
                         .count()
                   : -1;
    engineai_sdk::JointState joint;
    engineai_sdk::JointState motor;
    const bool got_joint = fields.joint && session->joint_state().latest(joint);
    const bool got_motor = fields.motor && session->motor_state().latest(motor);
    const auto imu = fields.imu ? session->imu().latest() : std::nullopt;
    const auto motion = fields.motion ? session->motion().current_motion() : std::nullopt;
    const auto power = fields.power ? session->power().latest() : std::nullopt;
    const auto diag = fields.diagnostics ? session->motor_diagnostics().latest() : std::nullopt;
    if (fields.joint && !got_joint) ++missing_joint;
    if (fields.motor && !got_motor) ++missing_motor;
    if (fields.imu && !imu) ++missing_imu;
    if (fields.motion && !motion) ++missing_motion;
    if (fields.power && !power) ++missing_power;
    if (fields.diagnostics && !diag) ++missing_diag;
    if (power && std::isfinite(power->power_info.percentage)) {
      const double pct = power->power_info.percentage;
      if (!have_power || pct < min_power) min_power = pct;
      if (!have_power || pct > max_power) max_power = pct;
      have_power = true;
    }
    if (imu && std::isfinite(imu->rpy.x)) {
      if (!have_roll || imu->rpy.x < min_roll) min_roll = imu->rpy.x;
      if (!have_roll || imu->rpy.x > max_roll) max_roll = imu->rpy.x;
      have_roll = true;
    }
    int offline = 0;
    if (diag) {
      for (uint8_t flag : diag->offline) {
        if (flag != 0) ++offline;
      }
    }

    if (format == "csv") {
      out << stamp;
      if (fields.joint) out << "," << (got_joint ? 0 : 1) << "," << joint_since_receive_ms << "," << (joint_new ? 1 : 0) << "," << (got_joint ? joint.position.size() : 0) << "," << (got_joint ? Join(joint.position) : "");
      if (fields.motor) out << "," << (got_motor ? 0 : 1) << "," << (got_motor ? motor.position.size() : 0);
      if (fields.imu) {
        out << "," << (imu ? 0 : 1) << ",";
        if (imu) out << imu->rpy.x << "," << imu->rpy.y << "," << imu->rpy.z;
        else out << ",,";
      }
      if (fields.motion) out << "," << (motion ? 0 : 1) << "," << (motion ? *motion : "");
      if (fields.power) out << "," << (power ? 0 : 1) << "," << (power ? power->power_info.percentage : 0.0);
      if (fields.diagnostics) out << "," << (diag ? 0 : 1) << "," << offline;
      out << "\n";
    } else {
      out << "{\"timestamp_ms\": " << stamp;
      if (fields.joint) out << ", \"missing_joint\": " << (got_joint ? "false" : "true") << ", \"joint_since_receive_ms\": " << joint_since_receive_ms << ", \"joint_new\": " << (joint_new ? "true" : "false") << ", \"joint_n\": " << (got_joint ? joint.position.size() : 0);
      if (fields.motor) out << ", \"missing_motor\": " << (got_motor ? "false" : "true") << ", \"motor_n\": " << (got_motor ? motor.position.size() : 0);
      if (fields.imu) {
        out << ", \"missing_imu\": " << (imu ? "false" : "true");
        if (imu) {
          if (!std::isfinite(imu->rpy.x)) ++invalid_numbers;
          if (!std::isfinite(imu->rpy.y)) ++invalid_numbers;
          if (!std::isfinite(imu->rpy.z)) ++invalid_numbers;
          out << ", \"roll\": " << robot_session_examples::JsonFinite(imu->rpy.x)
              << ", \"pitch\": " << robot_session_examples::JsonFinite(imu->rpy.y)
              << ", \"yaw\": " << robot_session_examples::JsonFinite(imu->rpy.z);
        }
      }
      if (fields.motion) {
        out << ", \"missing_motion\": " << (motion ? "false" : "true") << ", \"motion\": "
            << robot_session_examples::JsonQuote(motion ? *motion : "");
      }
      if (fields.power) {
        const double pct = power ? power->power_info.percentage : 0.0;
        if (power && !std::isfinite(pct)) ++invalid_numbers;
        out << ", \"missing_power\": " << (power ? "false" : "true") << ", \"power_pct\": "
            << robot_session_examples::JsonFinite(pct);
      }
      if (fields.diagnostics) out << ", \"missing_diagnostics\": " << (diag ? "false" : "true") << ", \"motor_offline_n\": " << offline;
      out << "}\n";
    }
    if (!out) {
      break;
    }
    ++rows;
    std::this_thread::sleep_for(period);
  }
  out.flush();
  const bool write_ok = static_cast<bool>(out);
  out.close();
  session->close();
  if (!write_ok) {
    std::cerr << "write failed saved=" << rows << "\n";
    return 1;
  }
  std::cout << "output=" << output << " format=" << format << " rows=" << rows
            << " invalid_numbers=" << invalid_numbers << "\n";
  std::cout << "fields=";
  const char* names[] = {"joint", "motor", "imu", "motion", "power", "diagnostics"};
  bool first = true;
  for (const char* name : names) {
    if (!Has(fields, name)) continue;
    if (!first) std::cout << ",";
    first = false;
    std::cout << name;
  }
  std::cout << "\n";
  std::cout << "missing joint=" << missing_joint << " motor=" << missing_motor << " imu=" << missing_imu
            << " motion=" << missing_motion << " power=" << missing_power << " diagnostics=" << missing_diag << "\n";
  if (have_power) {
    std::cout << "power_pct min=" << min_power << " max=" << max_power << "\n";
  }
  if (have_roll) {
    std::cout << "imu_roll min=" << min_roll << " max=" << max_roll << "\n";
  }
  return rows > 0 ? 0 : 1;
}
