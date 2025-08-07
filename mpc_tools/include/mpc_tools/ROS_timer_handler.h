#ifndef MPC_TOOLS_ROS_TIMER_HANDLER_H
#define MPC_TOOLS_ROS_TIMER_HANDLER_H

#include <ros/node_handle.h>

#include <boost/bind.hpp>

namespace Helpers {

class RosTimerHandler {
 public:
  RosTimerHandler(ros::Timer timer);
  ~RosTimerHandler() {};

  template <class T>
  void createAndStartTimer(ros::NodeHandle &nh, const double &freq,
                           void (T::*callback)(const ros::TimerEvent &),
                           T *obj) {
    // timer_ = nh.createTimer(ros::Duration(1.0 / freq), boost::bind(callback,
    // obj, _1));
    is_running_ = true;
  }

  void start(int max_control_loop_calls = -1);

  void stop();

  bool isRunning();

  void countUp();

  void countUpNotCompleted();

  void resetCount(bool ignore_next_count_up = false);

  void resetFlag();

  int getCountSinceReset();

  int getCountSinceStart();

  int getCountTotal();

  bool getFlag();

 private:
  // Timer related
  ros::Timer timer_;
  bool is_running_;

  // Control loop related
  int n_runs_since_reset_;  // Keeps track of amount of control loop calls
                            // during runtime since last reset
  int n_runs_since_start_;  // Keeps track of amount of control loop calls
                            // during runtime since last timer start call
  int n_runs_total_;        // Keeps track of total amount of control loop calls
                            // during runtime
  bool ignore_next_count_up_;  // Ignores the count for n_runs_since_reset_ and
                               // n_runs_since_start_ in the next countUp() call
  int max_control_loop_calls_;  // Maximum amount of times control loop should
                                // be called before automatically stopping the
                                // timer
  bool max_control_loop_calls_flag_;
};

};  // namespace Helpers

#endif
