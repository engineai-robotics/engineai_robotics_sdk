// led_control: set lamp patterns by name or value.
// One set_mode, a named sequence, or every LedColor.
// There is no API to read the current lamp. kOk means this process sent the request.
//
//   ./led_control --domain-id 0 --list
//   ./led_control --domain-id 0 --color blink_green
//   ./led_control --domain-id 0 --sequence all --cycles 1

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

struct LedEntry {
  const char* name;
  engineai_sdk::LedColor color;
  const char* meaning;
};

constexpr LedEntry kModes[] = {
    {"blink_red", engineai_sdk::LedColor::kBlinkRed, "blink red"},
    {"blink_green", engineai_sdk::LedColor::kBlinkGreen, "blink green"},
    {"blink_blue", engineai_sdk::LedColor::kBlinkBlue, "blink blue"},
    {"blink_white", engineai_sdk::LedColor::kBlinkWhite, "blink white"},
    {"constant_on_white", engineai_sdk::LedColor::kConstantOnWhite, "constant white"},
    {"constant_on_green", engineai_sdk::LedColor::kConstantOnGreen, "constant green"},
    {"breathe_white", engineai_sdk::LedColor::kBreatheWhite, "breathe white"},
    {"water_white", engineai_sdk::LedColor::kWaterWhite, "water white"},
    {"breathe_red", engineai_sdk::LedColor::kBreatheRed, "breathe red"},
    {"blink_orange", engineai_sdk::LedColor::kBlinkOrange, "blink orange"},
    {"constant_on_orange", engineai_sdk::LedColor::kConstantOnOrange, "constant orange"},
    {"blink_red_and_white", engineai_sdk::LedColor::kBlinkRedAndWhite, "blink red and white"},
    {"constant_on_red", engineai_sdk::LedColor::kConstantOnRed, "constant red"},
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--list] [--color NAME|N] [--sequence NAME,NAME|all] [--cycles N] [--period-ms MS]\n"
            << "  --sequence all or --cycles plays every lamp mode. --cycles 0 runs until Ctrl+C.\n"
            << "  Sets a lamp request. The SDK cannot read the current lamp.\n"
            << "  Fault and safety indications on the robot may hide the request.\n";
}

void PrintModes() {
  for (const auto& mode : kModes) {
    std::cout << mode.name << " value=" << static_cast<int>(mode.color) << " " << mode.meaning << "\n";
  }
}

void PrintCoverageWarning() {
  std::cout << "note: set_mode reports only that this process sent the request. "
               "There is no API to read the current lamp. "
               "Fault and safety indications may hide this request until they clear.\n";
}

bool ParseColor(const std::string& text, engineai_sdk::LedColor* out) {
  for (const auto& mode : kModes) {
    if (text == mode.name) {
      *out = mode.color;
      return true;
    }
  }
  char* end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  for (const auto& mode : kModes) {
    if (static_cast<long>(mode.color) == value) {
      *out = mode.color;
      return true;
    }
  }
  return false;
}

const char* ColorName(engineai_sdk::LedColor color) {
  for (const auto& mode : kModes) {
    if (mode.color == color) {
      return mode.name;
    }
  }
  return "unknown";
}

bool ParseSequence(const std::string& text, std::vector<engineai_sdk::LedColor>* out) {
  if (text == "all") {
    for (const auto& mode : kModes) {
      out->push_back(mode.color);
    }
    return true;
  }
  std::string token;
  for (size_t i = 0; i <= text.size(); ++i) {
    if (i == text.size() || text[i] == ',') {
      if (token.empty()) {
        return false;
      }
      engineai_sdk::LedColor color;
      if (!ParseColor(token, &color)) {
        return false;
      }
      out->push_back(color);
      token.clear();
    } else if (text[i] != ' ') {
      token.push_back(text[i]);
    }
  }
  return !out->empty();
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

  bool list = false;
  std::string color_text;
  std::string sequence_text;
  int cycles = 1;
  bool cycles_set = false;
  int period_ms = 1000;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--list") {
      list = true;
    } else if (arg == "--color" && i + 1 < rest.size()) {
      color_text = rest[++i];
    } else if (arg == "--sequence" && i + 1 < rest.size()) {
      sequence_text = rest[++i];
    } else if (arg == "--cycles" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &cycles, "cycles")) {
        return 2;
      }
      cycles_set = true;
    } else if (arg == "--period-ms" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &period_ms, "period-ms")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (list) {
    PrintModes();
  }
  if (color_text.empty() && sequence_text.empty()) {
    if (list && !cycles_set) {
      return 0;
    }
    if (!cycles_set) {
      std::cerr << "pass --list, --color, --sequence, or --cycles\n";
      PrintUsage(argv[0]);
      return 2;
    }
    sequence_text = "all";
  }
  if (cycles < 0 || period_ms <= 0) {
    std::cerr << "period-ms must be > 0\n";
    return 2;
  }

  std::vector<engineai_sdk::LedColor> modes;
  if (!color_text.empty()) {
    engineai_sdk::LedColor color;
    if (!ParseColor(color_text, &color)) {
      std::cerr << "unknown color '" << color_text << "'\n";
      PrintModes();
      return 2;
    }
    modes.push_back(color);
  }
  if (!sequence_text.empty()) {
    if (!ParseSequence(sequence_text, &modes)) {
      std::cerr << "unknown sequence '" << sequence_text << "'\n";
      PrintModes();
      return 2;
    }
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::WaitForRobotSample(session.get())) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }
  PrintCoverageWarning();
  robot_session_examples::CatchSigInt();
  int cycle = 0;
  while (!robot_session_examples::g_stop && (cycles == 0 || cycle < cycles)) {
    for (engineai_sdk::LedColor mode : modes) {
      if (robot_session_examples::g_stop) {
        break;
      }
      const auto err = session->led().set_mode(mode);
      std::cout << "led.set_mode name=" << ColorName(mode) << " value=" << static_cast<int>(mode)
                << " -> " << robot_session_examples::ErrName(err) << "\n";
      if (err != engineai_sdk::SessionError::kOk) {
        session->close();
        return 1;
      }
      if (modes.size() > 1 || cycles != 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
      }
    }
    ++cycle;
  }
  robot_session_examples::PauseBeforeClose();
  session->close();
  return 0;
}
