#include "mpc_core/interface.h"

Interface::Interface(ros::NodeHandle &nh, ControllerBase *ptr_controller,
                     ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
                     RobotRegion *ptr_robot_region,
                     DataSaverJson *ptr_data_saver_json, std::mutex *mutex)
    : nh_(nh),
      ptr_controller_(ptr_controller),
      ptr_config_(ptr_config),
      ptr_solver_(ptr_solver),
      ptr_robot_region_(ptr_robot_region),
      ptr_data_saver_json_(ptr_data_saver_json),
      ptr_mutex_(mutex) {};
