#ifndef MPC_CORE_CONTROLLER
#define MPC_CORE_CONTROLLER

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/controller_base.h>
#include <mpc_base/interface_base.h>
#include <mpc_base/module_handler_base.h>
#include <mpc_base/solver_base.h>
#include <mpc_msgs/MpcHeader.h>
#include <mpc_tools/ROS_timer_handler.h>
#include <mpc_tools/benchmarker.h>
#include <mpc_tools/data_saver_json.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Temperature.h>

#include <mutex>

class Controller : public ControllerBase {
 public:
  Controller(ros::NodeHandle& nh, ConfigurationMPC* ptr_config,
             SolverBase* ptr_solver, InterfaceBase* ptr_interface,
             ModuleHandlerBase* ptr_module_handler,
             DataSaverJson* ptr_data_saver_json);
  ~Controller();

  void runNode(const ros::TimerEvent& event);
  void controlLoopStart();
  void controlLoopEnd();
  void controlLoopEndNotCompleted();
  void controlLoop() override;
  void printInfoAtNoSuccess(int& exit_code);

  void setInterface(InterfaceBase* ptr_interface) override;
  void startTimer(int max_control_loop_calls = -1) override;
  void stopTimer() override;
  bool timerIsRunning() override;
  int getCountSinceReset() override;
  int getCountTotal() override;
  void setMpcHeader(mpc_msgs::MpcHeader& mpc_header) override;
  void resetTimerFlag() override;
  void OnReset() override;
  void OnObjectiveReached() override;
  void OnStateReceived() override;
  double getTime() override;
  void setTime(const double t) override;

  // Basic initialisation
  ros::NodeHandle nh_;
  ConfigurationMPC* ptr_config_;
  SolverBase* ptr_solver_;
  InterfaceBase* ptr_interface_;
  ModuleHandlerBase* ptr_module_handler_;
  DataSaverJson* ptr_data_saver_json_;

  // String to fill for profiling code
  std::string control_loop_profiling_string_;

  // ROS message for publishing control loop timing properties
  mpc_msgs::MpcHeader mpc_header_;

  // Publishers
  ros::Publisher pub_computation_times_;

  // msg for loop time
  sensor_msgs::Temperature computation_time_msg_;

  // Timer class
  Helpers::RosTimerHandler timer_;

  // Lightweight classes for profiling the controller
  Helpers::Benchmarker optimization_benchmarker_, control_loop_benchmarker_,
      module_benchmarkers_;

  // Mutex to avoid race conditions
  std::mutex mutex_;

  // Data positions in solver data
  int x_data_position_;
  int y_data_position_;

  // Boolean variables
  bool objective_reached_ = false;
  bool updated_state_received_ = false;
  bool state_received_ = false;

  // Stored variables
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  int exit_code_ = 0;
  int previous_exit_code_ = 0;

  // Loop info
  bool first_loop_ = true;

  // Time info
  double t_ = -1.0;
};

#endif
