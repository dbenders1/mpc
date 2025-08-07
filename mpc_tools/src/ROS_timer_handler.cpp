
#include <mpc_tools/ROS_timer_handler.h>

namespace Helpers {

RosTimerHandler::RosTimerHandler(ros::Timer timer)
    : timer_(timer),
      is_running_(false),
      n_runs_since_reset_(0),
      n_runs_since_start_(0),
      n_runs_total_(0),
      ignore_next_count_up_(false),
      max_control_loop_calls_(-1),
      max_control_loop_calls_flag_(false) {
  timer_.stop();
}

void RosTimerHandler::start(int max_control_loop_calls) {
  max_control_loop_calls_ = max_control_loop_calls;
  n_runs_since_start_ = 0;

  timer_.start();
  is_running_ = true;
}

void RosTimerHandler::stop() {
  timer_.stop();
  is_running_ = false;
}

bool RosTimerHandler::isRunning() { return is_running_; }

void RosTimerHandler::countUp() {
  n_runs_since_reset_ += 1;
  n_runs_since_start_ += 1;

  countUpNotCompleted();
}

void RosTimerHandler::countUpNotCompleted() {
  if (ignore_next_count_up_) {
    ignore_next_count_up_ = false;
    return;
  }

  n_runs_total_ += 1;

  if (max_control_loop_calls_ != -1 &&
      n_runs_since_start_ >= max_control_loop_calls_) {
    stop();
    max_control_loop_calls_flag_ = true;
  }
}

void RosTimerHandler::resetCount(bool ignore_next_count_up) {
  n_runs_since_reset_ = 0;

  if (ignore_next_count_up) {
    ignore_next_count_up_ = true;
  }
}

void RosTimerHandler::resetFlag() { max_control_loop_calls_flag_ = false; }

int RosTimerHandler::getCountSinceReset() { return n_runs_since_reset_; }

int RosTimerHandler::getCountSinceStart() { return n_runs_since_start_; }

int RosTimerHandler::getCountTotal() { return n_runs_total_; }

bool RosTimerHandler::getFlag() { return max_control_loop_calls_flag_; }

};  // namespace Helpers
