#ifndef MPC_CORE_INTERFACE_H
#define MPC_CORE_INTERFACE_H

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/controller_base.h>
#include <mpc_base/interface_base.h>
#include <mpc_base/module_handler_base.h>
#include <mpc_base/solver_base.h>
#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/robot_region.h>
#include <ros/node_handle.h>

#include <mutex>

class Interface : public InterfaceBase {
 public:
  Interface(ros::NodeHandle &nh, ControllerBase *ptr_controller,
            ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
            RobotRegion *ptr_robot_region, DataSaverJson *ptr_data_saver_json,
            std::mutex *mutex);

  ~Interface() {};

  // ROS nodehandle
  ros::NodeHandle nh_;

  // MPC controller pointer
  ControllerBase *ptr_controller_;

  // Configuration struct
  ConfigurationMPC *ptr_config_;

  // Solver pointer
  SolverBase *ptr_solver_;

  // Robot region pointer
  RobotRegion *ptr_robot_region_;

  // Data saver to JSON pointer
  DataSaverJson *ptr_data_saver_json_;

  // Mutex
  std::mutex *ptr_mutex_;
};

#endif
