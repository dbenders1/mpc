#ifndef MPC_H
#define MPC_H

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/controller_base.h>
#include <mpc_base/interface_base.h>
#include <mpc_base/solver_base.h>
#include <mpc_core/controller.h>
#include <mpc_core/functions/update_mpc_configuration.h>
#include <mpc_core/reconfigure_callback.h>
#include <mpc_falcon/falcon_interface.h>
#include <mpc_modules/loader/module_handler.h>
#include <mpc_solver/falcon/dynamic_reconfigure_files/reconfigure_callback_options.h>
#include <mpc_solver/falcon/falcon_solver_interface.h>
#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/robot_region.h>
#include <ros/node_handle.h>

#include <Eigen/Core>
#include <memory>

class Mpc {
 public:
  Mpc(const int layer_idx = 0);
  ~Mpc();

  bool setInterval(const double dt);
  bool setLastCommand(const Eigen::VectorXd &last_command);
  bool setState(const double t, const Eigen::VectorXd &state);
  bool run();
  Eigen::VectorXd getCommand();

 private:
  ros::NodeHandle nh_;
  ConfigurationMPC config_mpc_;
  std::unique_ptr<SolverBase> ptr_solver_;
  std::unique_ptr<RobotRegion> ptr_robot_region_;
  std::unique_ptr<DataSaverJson> ptr_data_saver_json_;
  std::unique_ptr<ModuleHandlerBase> ptr_module_handler_;
  std::unique_ptr<ControllerBase> ptr_controller_;
  std::unique_ptr<InterfaceBase> ptr_interface_;
};

#endif  // MPC_H
