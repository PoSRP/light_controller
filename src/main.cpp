/**
 *
 **/

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

// #define USING_THREAD
#ifdef USING_THREAD
#include <thread>
#endif

#include "boost/sml.hpp"
namespace sml = boost::sml;

#if __has_include("wiringPi.h")
#include "wiringPi.h"
#define ON_RPI
#endif

namespace logger {
struct fsm_logger {
  template <class SM, class TEvent>
  void log_process_event(const TEvent &) {
    printf("%s[event] %s\n",
           sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TEvent>());
  }
  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard &, const TEvent &, bool result) {
    printf("%s[guard] %s %s %s\n",
           sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TGuard>(),
           sml::aux::get_type_name<TEvent>(),
           (result ? "[OK]" : "[REJECTED]"));
  }
  template <class SM, class TAction, class TEvent>
  void log_action(const TAction &, const TEvent &) {
    printf("%s[action] %s %s\n",
           sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TAction>(),
           sml::aux::get_type_name<TEvent>());
  }
  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState &src, const TDstState &dst) {
    printf("%s[transition] %s -> %s\n",
           sml::aux::get_type_name<SM>(),
           src.c_str(),
           dst.c_str());
  }
};
}  // namespace logger

namespace ctrl {
// Declared before the hw namespace because it's all in one file
enum TIMESLOT { LONG, SHORT };
}  // namespace ctrl

namespace hw {
enum LEVEL { HIGH, LOW };
enum INPUT_MODE { PULL_UP, PULL_DOWN };

template <typename T>
concept component = requires {
  T::setup;
  T::on;
  T::off;
};

template <int Pin>
struct output final {
  static constexpr auto setup = [] {
#ifdef ON_RPI
    pinMode(uint8_t(Pin), OUTPUT);
#endif
  };

  static constexpr auto on = [] {
#ifdef ON_RPI
    digitalWrite(uint8_t(Pin), HIGH);
#endif
  };

  static constexpr auto off = [] {
#ifdef ON_RPI
    digitalWrite(uint8_t(Pin), LOW);
#endif
  };
};

template <int Pin, INPUT_MODE Mode>
struct input final {
  inline static bool last_value{false};

  static constexpr auto setup = [] {
#ifdef ON_RPI
    pinMode(Pin, INPUT);
    pullUpDnControl(Pin, Mode);
#endif
  };

  static constexpr auto on = [](const auto &event) {
    return event() == LEVEL::HIGH;
  };

  static constexpr auto off = [](const auto &event) {
    return event() == LEVEL::LOW;
  };

  static constexpr auto toggled = [] {
    bool is_pressed;
#ifdef ON_RPI
    is_pressed = digitalRead(Pin);
#else
    is_pressed = rand() % 10000 ? true : false;
#endif
    if (last_value != is_pressed) {
      last_value = is_pressed;
      return true;
    } else {
      return false;
    }
  };
};

}  // namespace hw

namespace ctrl {

// STATE VARIABLES
static std::atomic<int64_t> start_time_minutes{};
static std::atomic<bool> task_running{false};
#ifdef USING_THREAD
static std::thread task_thread;
#endif
static TIMESLOT active_timeslot = TIMESLOT::LONG;

// CONSTANTS
static const std::string long_on_time  = "18:00";
static const std::string short_on_time = "12:00";

// HARDWARE
using In_OnOff  = hw::input<8, hw::INPUT_MODE::PULL_DOWN>;
using In_Mode   = hw::input<9, hw::INPUT_MODE::PULL_DOWN>;
using Out_Light = hw::output<10>;

// EVENTS
struct turn_on {
  std::string time_on;
};
struct turn_off {};
struct change_on_time {};

// EVENT GUARDS
struct turn_on_guard {
  bool operator()(const turn_on &e) const {
    if (e.time_on.length() < 5) {
      printf("  Start time field too short: %s\n", e.time_on.c_str());
      return false;
    }
    if (e.time_on.find(":") == std::string::npos &&
        e.time_on.find(".") == std::string::npos) {
      printf("  Missing start time separator: %s\n", e.time_on.c_str());
      return false;
    }

    const auto on_hour   = e.time_on.substr(0, 2);
    const auto on_minute = e.time_on.substr(3, 2);

    auto is_number = [](const std::string &s) {
      return s.find_first_not_of("0123456789") == std::string::npos;
    };

    if (!is_number(on_hour) || !is_number(on_minute)) {
      printf("  Non-number in start time field: %s\n", e.time_on.c_str());
      return false;
    }
    if (std::stoi(on_hour) < 0 || std::stoi(on_hour) >= 24) {
      printf("  Start time hour outside bounds: %s\n", on_hour.c_str());
      return false;
    }
    if (std::stoi(on_minute) < 0 || std::stoi(on_minute) >= 60) {
      printf("  Start time minute outside bounds: %s\n", on_minute.c_str());
      return false;
    }

    return true;
  }
} turn_on_guard;

// STATES
class on;
class off;

// TASKS
void iterate_task() {
  std::time_t now;
  std::time(&now);
  std::tm *curtime      = std::localtime(&now);
  const auto now_time   = curtime->tm_hour * 60L + curtime->tm_min;
  const auto start_time = start_time_minutes.load();

  std::string dur_time_s;
  if (active_timeslot == TIMESLOT::LONG) {
    dur_time_s = long_on_time;
  } else if (active_timeslot == TIMESLOT::SHORT) {
    dur_time_s = short_on_time;
  } else {
    assert(false);
  }

  const auto duration_hour   = std::stoi(dur_time_s.substr(0, 2));
  const auto duration_minute = std::stoi(dur_time_s.substr(3, 2));
  const auto duration_time   = duration_hour * 24 + duration_minute;
  const auto stop_time       = (start_time + duration_time) % (24L * 60L);
  const auto stop_next_day   = (stop_time < start_time);

  if (stop_next_day) {
    if (start_time < now_time)
      Out_Light::on();
    else
      Out_Light::off();
  } else {
    if (now_time < stop_time)
      Out_Light::on();
    else
      Out_Light::off();
  }
}

#ifdef USING_THREAD
void timer_task() {
  while (task_running.load()) {
    iterate_task();
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
  }
}
#endif

// ACTIONS
struct on_action {
  void operator()(const turn_on &a) {
    printf("  Starting with 'on_time=%s'\n", a.time_on.c_str());

#ifdef USING_THREAD
    if (!task_running.load()) {
      task_running.exchange(true);
      task_thread = std::thread(&timer_task);
      printf("  Task thread started\n");
    }
#endif

    const auto on_hour   = std::stoi(a.time_on.substr(0, 2));
    const auto on_minute = std::stoi(a.time_on.substr(3, 2));
    start_time_minutes   = on_hour * 60 + on_minute;
  };
} on_action;

struct off_action {
  void operator()() {
#ifdef USING_THREAD
    if (task_running.load()) {
      task_running.exchange(false);
      task_thread.join();
      printf("  Task thread joined\n");
    }
#endif
  };
} off_action;

struct change_on_time_action {
  void operator()() {
    if (active_timeslot == TIMESLOT::SHORT)
      active_timeslot = TIMESLOT::LONG;
    else
      active_timeslot = TIMESLOT::SHORT;
  }
} change_on_time_action;

// TABLE
struct fsm {
  constexpr fsm() {
    In_OnOff::setup();
    In_Mode::setup();
    Out_Light::setup();
  }

  auto operator()() const noexcept {
    using namespace sml;
    // clang-format off
    return make_transition_table(
    // STATE ------ EVENT ---------------- GUARD ---------- ACTION ---------------- STATE ----- //
      *state<off> + event<turn_on>        [turn_on_guard] / on_action             = state<on>,
      state<on>   + event<turn_off>                       / off_action            = state<off>,
    // ---------------------------------------------------------------------------------------- //
      state<on>   + event<change_on_time>                 / change_on_time_action = state<on>);
    // ---------------------------------------------------------------------------------------- //
    // clang-format on
  }
};
}  // namespace ctrl

int main(int argc, char *argv[]) {
#ifndef ON_RPI
  // Initialize rand for faking inputs
  std::srand(std::chrono::system_clock::now().time_since_epoch().count());
#endif

  using namespace ctrl;
  using namespace logger;
  using namespace std::chrono_literals;

  fsm_logger logger;
  sml::sm<fsm, sml::logger<fsm_logger>> sm{logger};

  auto args = std::vector<std::string>(argv, argv + argc);
  assert(args.size() == 2);

  sm.process_event(turn_on{args[1]});
  assert(sm.is(sml::state<on>));

  while (1) {
    if (In_OnOff::toggled() && sm.is(sml::state<on>))
      sm.process_event(turn_off{});
    else if (In_OnOff::toggled() && sm.is(sml::state<off>))
      sm.process_event(turn_on{args[1]});

    if (In_Mode::toggled())
      sm.process_event(change_on_time{});

#ifndef USING_THREAD
    ctrl::iterate_task();
#endif
    usleep(1000);  // Only to avoid 100% core load
  }

  sm.process_event(turn_off{});
  return 0;
}
