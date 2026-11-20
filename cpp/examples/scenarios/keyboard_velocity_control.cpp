// keyboard_velocity_control: low-speed WASD velocity on this terminal.
// The terminal reports key repeats, not key-up. A key is treated as released
// when no repeat arrives within --deadman seconds, and zero velocity is sent.
//
//   ./keyboard_velocity_control --domain-id 0

#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

class TerminalRaw {
 public:
  TerminalRaw() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) {
      return;
    }
    termios raw = saved_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      active_ = true;
    }
  }
  ~TerminalRaw() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
  }
  bool active() const { return active_; }

 private:
  termios saved_{};
  bool active_ = false;
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--max-x V] [--max-y V] [--max-yaw-rate W] [--deadman SEC] [--rate HZ] [--duration SEC]\n"
            << "  W/S forward/back, A/D lateral, Q/E yaw, Space zero, Esc quit.\n"
            << "  Defaults: max-x=0.5 max-y=0.5 max-yaw-rate=0.5 deadman=0.4 rate=20 duration=30.\n"
            << "  duration 0 runs until Esc. Exit, timeout, and Ctrl+C send zero velocity.\n"
            << "  Forward, lateral, and yaw map to linear_velocity[0], linear_velocity[1], and yaw_velocity.\n"
            << "  On PM01 lower_body_balance, linear_velocity[0] around 0.5 is where stepping starts.\n";
}

bool Send(engineai_sdk::Robot* session, double x, double y, double yaw) {
  engineai_sdk::BodyVelocityCommand cmd;
  cmd.x = x;
  cmd.y = y;
  cmd.yaw_rate = yaw;
  return robot_session_examples::CheckOk(session->base_velocity().send(cmd), "base_velocity.send");
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

  double max_x = 0.5;
  double max_y = 0.5;
  double max_yaw = 0.5;
  double deadman = 0.4;
  double rate = 20.0;
  double duration = 30.0;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--max-x" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &max_x, "max-x")) {
        return 2;
      }
    } else if (arg == "--max-y" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &max_y, "max-y")) {
        return 2;
      }
    } else if (arg == "--max-yaw-rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &max_yaw, "max-yaw-rate")) {
        return 2;
      }
    } else if (arg == "--deadman" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &deadman, "deadman")) {
        return 2;
      }
    } else if (arg == "--rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &rate, "rate")) {
        return 2;
      }
    } else if (arg == "--duration" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &duration, "duration")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (max_x < 0.0 || max_y < 0.0 || max_yaw < 0.0 || deadman <= 0.0 || rate < 5.0 || duration < 0.0) {
    std::cerr << "max values must be >= 0, deadman > 0, rate >= 5, duration >= 0\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::WaitForMotionSample(session.get())) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }

  TerminalRaw terminal;
  if (!terminal.active()) {
    std::cerr << "stdin is not a terminal; keyboard control needs a local TTY\n";
    session->close();
    return 2;
  }

  std::cout << "Keyboard velocity control\n"
            << "  W/S    forward / backward\n"
            << "  A/D    strafe left / right\n"
            << "  Q/E    turn left / right\n"
            << "  Space  send zero velocity now\n"
            << "  Esc or Ctrl+C  quit and send zero velocity\n"
            << "This run exits after " << duration
            << " s. Holding a key does not extend it. Pass --duration 0 to control until you quit.\n"
            << "The terminal repeats keys and has no key-up event. After " << deadman
            << " s without a repeat, the key counts as released and zero velocity is sent.\n"
            << "kOk only means the command was sent, not that the robot is moving at that velocity.\n"
            << "motion=" << robot_session_examples::MotionOrNone(*session) << " max_x=" << max_x << " max_y=" << max_y
            << " max_yaw_rate=" << max_yaw << " rate_hz=" << rate << " duration_s=" << duration << "\n"
            << "The next line refreshes in place and keeps only the latest status.\n";
  robot_session_examples::CatchSigInt();

  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate));
  const auto deadman_for = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(deadman));
  const bool limited = duration > 0.0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                                std::chrono::duration<double>(duration));
  auto last_print = std::chrono::steady_clock::now();
  auto hold_until = std::chrono::steady_clock::time_point::min();
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string keys = "-";
  bool send_ok = true;

  while (send_ok && !robot_session_examples::g_stop && (!limited || std::chrono::steady_clock::now() < deadline)) {
    bool saw_key = false;
    double next_x = 0.0;
    double next_y = 0.0;
    double next_yaw = 0.0;
    std::string next_keys;
    char ch = 0;
    while (read(STDIN_FILENO, &ch, 1) == 1) {
      if (ch == 27 || ch == 3) {
        robot_session_examples::g_stop = 1;
        break;
      }
      if (ch == ' ') {
        hold_until = std::chrono::steady_clock::time_point::min();
        keys = "space";
        saw_key = false;
        next_x = next_y = next_yaw = 0.0;
        next_keys.clear();
        continue;
      }
      char bit = 0;
      if (ch == 'w' || ch == 'W') {
        next_x = max_x;
        bit = 'W';
      } else if (ch == 's' || ch == 'S') {
        next_x = -max_x;
        bit = 'S';
      } else if (ch == 'a' || ch == 'A') {
        next_y = max_y;
        bit = 'A';
      } else if (ch == 'd' || ch == 'D') {
        next_y = -max_y;
        bit = 'D';
      } else if (ch == 'q' || ch == 'Q') {
        next_yaw = max_yaw;
        bit = 'Q';
      } else if (ch == 'e' || ch == 'E') {
        next_yaw = -max_yaw;
        bit = 'E';
      }
      if (bit != 0) {
        saw_key = true;
        next_keys.push_back(bit);
      }
    }
    if (robot_session_examples::g_stop) {
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (saw_key) {
      x = next_x;
      y = next_y;
      yaw = next_yaw;
      keys = next_keys;
      hold_until = now + deadman_for;
    } else if (now > hold_until) {
      x = 0.0;
      y = 0.0;
      yaw = 0.0;
      keys = "-";
    }
    send_ok = Send(session.get(), x, y, yaw);
    if (now - last_print >= std::chrono::milliseconds(100)) {
      std::cout << "\rkeys=" << keys << " x=" << x << " y=" << y << " yaw_rate=" << yaw << " rate_hz=" << rate
                << " motion=" << robot_session_examples::MotionOrNone(*session) << "\033[K" << std::flush;
      last_print = now;
    }
    std::this_thread::sleep_for(period);
  }

  const bool stopped = Send(session.get(), 0.0, 0.0, 0.0);
  std::cout << "\n";
  if (send_ok && !robot_session_examples::g_stop && limited) {
    std::cout << "Reached duration=" << duration << " s. Sending zero velocity and exiting.\n";
  }
  std::cout << "stop_sent=" << (stopped ? "yes" : "no") << " motion=" << robot_session_examples::MotionOrNone(*session)
            << "\n";
  robot_session_examples::PauseBeforeClose();
  session->close();
  return send_ok && stopped ? 0 : 1;
}
