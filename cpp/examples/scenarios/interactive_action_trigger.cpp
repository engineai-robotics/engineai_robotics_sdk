// interactive_action_trigger: one key starts one existing motion-plan action.
// Keyboard: 1=wave, 2=handshake, 3=reset, Space=cancel, Esc=quit.
// Gamepad: A=wave, B=handshake, X=reset, Y=cancel, Start=quit.
// The gamepad map is a local example. It is not a verified check against onboard button functions.
//
//   ./interactive_action_trigger --domain-id 0 --model pm01 --input keyboard --list-bindings

#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"
#include "support/motion_plan_runner.hpp"

namespace {

struct Binding {
  int code = 0;
  std::string action;
};

struct InputState {
  std::string mode = "keyboard";
  std::vector<Binding> bindings;
  bool quit = false;
  std::array<int, 12> buttons{};
  bool have_buttons = false;
  bool was_connected = false;
  bool wait_release = true;
};

class TerminalRaw {
 public:
  explicit TerminalRaw(bool enable) {
    if (!enable || !isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) {
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
            << "       --model pm01|t800|t800pro --input keyboard|gamepad\n"
            << "       [--bindings PATH] [--actions DIR] [--timeout SEC] [--poll-rate HZ] [--list-bindings]\n"
            << "  Default bindings: examples/config/interactive_bindings.yaml\n"
            << "  Requires current motion lower_body_balance. A new action is refused while a plan is still running.\n";
}

int GamepadCode(const std::string& name) {
  if (name == "lb") return static_cast<int>(engineai_sdk::GamepadButton::kLb);
  if (name == "rb") return static_cast<int>(engineai_sdk::GamepadButton::kRb);
  if (name == "a") return static_cast<int>(engineai_sdk::GamepadButton::kA);
  if (name == "b") return static_cast<int>(engineai_sdk::GamepadButton::kB);
  if (name == "x") return static_cast<int>(engineai_sdk::GamepadButton::kX);
  if (name == "y") return static_cast<int>(engineai_sdk::GamepadButton::kY);
  if (name == "back") return static_cast<int>(engineai_sdk::GamepadButton::kBack);
  if (name == "start") return static_cast<int>(engineai_sdk::GamepadButton::kStart);
  return -1;
}

void DefaultBindings(InputState* state) {
  if (state->mode == "gamepad") {
    state->bindings = {{static_cast<int>(engineai_sdk::GamepadButton::kA), "wave"},
                       {static_cast<int>(engineai_sdk::GamepadButton::kB), "handshake"},
                       {static_cast<int>(engineai_sdk::GamepadButton::kX), "reset"}};
    return;
  }
  state->bindings = {{'1', "wave"}, {'2', "handshake"}, {'3', "reset"}};
}

bool LoadBindings(const std::string& path, InputState* state) {
  if (!robot_session_examples::FileExists(path)) {
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node table = root[state->mode];
    if (!table || !table.IsMap()) {
      return false;
    }
    std::vector<Binding> bindings;
    for (const auto& item : table) {
      const std::string key = item.first.as<std::string>();
      const std::string action = item.second.as<std::string>();
      if (action != "wave" && action != "handshake" && action != "reset") {
        continue;
      }
      int code = -1;
      if (state->mode == "keyboard") {
        if (key.size() == 1) {
          code = key[0];
        }
      } else {
        code = GamepadCode(key);
      }
      if (code >= 0) {
        bindings.push_back(Binding{code, action});
      }
    }
    if (bindings.empty()) {
      return false;
    }
    state->bindings = std::move(bindings);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

void PrintBindings(const InputState& state) {
  std::cout << "input=" << state.mode << "\n";
  for (const auto& binding : state.bindings) {
    if (state.mode == "keyboard") {
      std::cout << "key=" << static_cast<char>(binding.code) << " action=" << binding.action << "\n";
    } else {
      std::cout << "button=" << binding.code << " action=" << binding.action << "\n";
    }
  }
  if (state.mode == "keyboard") {
    std::cout << "key=space action=cancel\nkey=esc action=quit\n";
  } else {
    std::cout << "button=Y action=cancel\nbutton=Start action=quit\n";
    std::cout << "note: gamepad mapping is a local example and is not verified against onboard button functions\n";
  }
}

const Binding* FindBinding(const InputState& state, int code) {
  for (const auto& binding : state.bindings) {
    if (binding.code == code) {
      return &binding;
    }
  }
  return nullptr;
}

void PollInput(engineai_sdk::Robot* session, void* user, bool* cancel) {
  auto* state = static_cast<InputState*>(user);
  if (state->mode == "keyboard") {
    char ch = 0;
    while (read(STDIN_FILENO, &ch, 1) == 1) {
      if (ch == ' ' || ch == 27 || ch == 3) {
        *cancel = true;
        if (ch != ' ') {
          state->quit = true;
        }
        std::cout << "cancel requested\n";
      } else if (const Binding* binding = FindBinding(*state, ch)) {
        std::cout << "rejected action=" << binding->action << " reason=planner busy\n";
      }
    }
    return;
  }
  const auto pad = session->gamepad().latest();
  if (!pad) {
    return;
  }
  if (!pad->hardware_connected) {
    if (state->was_connected) {
      std::cout << "gamepad disconnected; cancel requested\n";
      *cancel = true;
    }
    state->was_connected = false;
    state->wait_release = true;
    return;
  }
  if (!state->was_connected) {
    state->wait_release = true;
    state->was_connected = true;
  }
  const int y = static_cast<int>(engineai_sdk::GamepadButton::kY);
  const int start = static_cast<int>(engineai_sdk::GamepadButton::kStart);
  if (pad->digital_states[static_cast<size_t>(y)] != 0 || pad->digital_states[static_cast<size_t>(start)] != 0) {
    *cancel = true;
    if (pad->digital_states[static_cast<size_t>(start)] != 0) {
      state->quit = true;
    }
    std::cout << "cancel requested\n";
  }
  for (const auto& binding : state->bindings) {
    if (pad->digital_states[static_cast<size_t>(binding.code)] != 0) {
      std::cout << "rejected action=" << binding.action << " reason=planner busy\n";
    }
  }
  state->buttons = pad->digital_states;
  state->have_buttons = true;
}

std::string PressedAction(InputState* state, engineai_sdk::Robot* session) {
  if (state->mode == "keyboard") {
    static char last = 0;
    static auto last_time = std::chrono::steady_clock::time_point::min();
    char ch = 0;
    std::string action;
    while (read(STDIN_FILENO, &ch, 1) == 1) {
      if (ch == 27 || ch == 3) {
        state->quit = true;
        return {};
      }
      if (ch == ' ') {
        std::cout << "cancel ignored reason=no active request\n";
        continue;
      }
      const auto now = std::chrono::steady_clock::now();
      const bool repeat = ch == last && now - last_time < std::chrono::milliseconds(250);
      last = ch;
      last_time = now;
      if (repeat) {
        continue;
      }
      if (const Binding* binding = FindBinding(*state, ch)) {
        action = binding->action;
      }
    }
    return action;
  }

  const auto pad = session->gamepad().latest();
  if (!pad) {
    std::cout << "gamepad=(no sample). Check that the robot is publishing gamepad data.\n";
    return {};
  }
  if (!pad->hardware_connected) {
    std::cout << "gamepad hardware_connected=false. Connect the pad and confirm the robot publishes it.\n";
    state->was_connected = false;
    state->wait_release = true;
    state->have_buttons = false;
    return {};
  }
  if (!state->was_connected) {
    state->wait_release = true;
    state->was_connected = true;
    state->buttons = pad->digital_states;
    state->have_buttons = true;
    std::cout << "gamepad connected; release held buttons before a new action\n";
    return {};
  }
  bool any_down = false;
  std::string action;
  for (const auto& binding : state->bindings) {
    const int now = pad->digital_states[static_cast<size_t>(binding.code)];
    const int prev = state->have_buttons ? state->buttons[static_cast<size_t>(binding.code)] : 0;
    if (now != 0) {
      any_down = true;
      if (!state->wait_release && prev == 0 && action.empty()) {
        action = binding.action;
      }
    }
  }
  const int start = pad->digital_states[static_cast<size_t>(engineai_sdk::GamepadButton::kStart)];
  if (start != 0) {
    state->quit = true;
  }
  if (!any_down) {
    state->wait_release = false;
  }
  state->buttons = pad->digital_states;
  state->have_buttons = true;
  return action;
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

  InputState state;
  std::string model;
  std::string bindings_path = "examples/config/interactive_bindings.yaml";
  std::string actions_dir = "examples/config";
  double timeout_s = 30.0;
  double poll_rate = 20.0;
  bool list_bindings = false;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--model" && i + 1 < rest.size()) {
      model = rest[++i];
    } else if (arg == "--input" && i + 1 < rest.size()) {
      state.mode = rest[++i];
    } else if (arg == "--bindings" && i + 1 < rest.size()) {
      bindings_path = rest[++i];
    } else if (arg == "--actions" && i + 1 < rest.size()) {
      actions_dir = rest[++i];
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &timeout_s, "timeout")) {
        return 2;
      }
    } else if (arg == "--poll-rate" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadFiniteDouble(rest[++i], &poll_rate, "poll-rate")) {
        return 2;
      }
    } else if (arg == "--list-bindings") {
      list_bindings = true;
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (state.mode != "keyboard" && state.mode != "gamepad") {
    std::cerr << "--input must be keyboard or gamepad\n";
    return 2;
  }
  if (timeout_s <= 0.0 || poll_rate <= 0.0) {
    std::cerr << "timeout and poll-rate must be > 0\n";
    return 2;
  }
  DefaultBindings(&state);
  if (!LoadBindings(bindings_path, &state)) {
    std::cout << "bindings file not loaded, using built-in map\n";
  }
  PrintBindings(state);
  if (list_bindings) {
    return 0;
  }
  if (model.empty()) {
    std::cerr << "Missing required --model\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::RequireCurrentMotion(session.get(), "lower_body_balance")) {
    session->close();
    return 1;
  }
  TerminalRaw terminal(state.mode == "keyboard");
  if (state.mode == "keyboard" && !terminal.active()) {
    std::cerr << "stdin is not a terminal\n";
    session->close();
    return 2;
  }
  std::cout << "ready model=" << model << " motion=" << robot_session_examples::MotionOrNone(*session) << "\n";
  robot_session_examples::CatchSigInt();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / poll_rate));

  while (!state.quit && !robot_session_examples::g_stop) {
    const std::string action = PressedAction(&state, session.get());
    if (state.quit || robot_session_examples::g_stop) {
      break;
    }
    if (!action.empty()) {
      if (!robot_session_examples::MotionIs(*session, "lower_body_balance")) {
        std::cout << "rejected action=" << action << " reason=current motion is not lower_body_balance\n";
      } else if (!robot_session_examples::IsFresh(session->motion().last_receive_time(),
                                                  robot_session_examples::kMotionMaxAge) ||
                 !robot_session_examples::IsFresh(session->joint_state().last_receive_time(),
                                                  robot_session_examples::kStateMaxAge)) {
        std::cout << "rejected action=" << action << " reason=missing or stale motion / joint_state sample\n";
      } else if (const auto planner = session->joint_motion_planner().latest_state();
                 planner && planner->status != engineai_sdk::JointMotionPlanStatus::kIdle) {
        std::cout << "rejected action=" << action << " reason=planner " << robot_session_examples::PlanStatusName(planner->status)
                  << "\n";
      } else {
        std::string path;
        std::string error;
        if (!robot_session_examples::ResolveActionYaml(actions_dir, model, action, &path, &error)) {
          std::cout << "rejected action=" << action << " reason=" << error << "\n";
        } else {
          robot_session_examples::MotionPlanFile plan;
          if (!robot_session_examples::LoadMotionPlanFile(path, &plan, &error)) {
            std::cout << "rejected action=" << action << " reason=" << error << "\n";
          } else {
            std::cout << "trigger action=" << action << " motion=" << robot_session_examples::MotionOrNone(*session) << "\n";
            robot_session_examples::PrintPlanPreview(path, plan, 1.0);
            robot_session_examples::MotionPlanRunRequest request;
            request.timeout_s = timeout_s;
            const auto code =
                robot_session_examples::RunMotionPlan(session.get(), plan, request, PollInput, &state);
            std::cout << "result=" << robot_session_examples::PlanRunName(code) << " action=" << action << "\n";
          }
        }
      }
    }
    std::this_thread::sleep_for(period);
  }
  std::cout << "quit\n";
  session->close();
  return 0;
}
