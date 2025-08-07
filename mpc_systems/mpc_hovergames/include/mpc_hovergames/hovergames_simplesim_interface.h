#ifndef HOVERGAMES_SIMPLESIM_INTERFACE_H
#define HOVERGAMES_SIMPLESIM_INTERFACE_H

#include <mpc_hovergames/hovergames_drone_interface.h>
#include <ros/publisher.h>

#include "simple_sim/DroneHovergamesControl.h"

class HovergamesSimpleSimInterface : public HovergamesDroneInterface {
 public:
  // Constructor and destructor
  HovergamesSimpleSimInterface(
      ros::NodeHandle &nh, ControllerBase *ptr_controller,
      ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
      ModuleHandlerBase *ptr_module_handler, RobotRegion *ptr_robot_region,
      DataSaverJson *ptr_data_saver_json, std::mutex *mutex);
  ~HovergamesSimpleSimInterface() {};

  // Drone interface methods
  void OnStateCallback(const nav_msgs::Odometry &msg) override;
  void OnComputeActuation() override;
  void OnActuate() override;
  void OnDeployObjectiveReachedStrategy() override;
  void OnDeployEmergencyStrategy() override;
  void OnActuateBrake() override;

 private:
  // ROS publishers and subscribers
  ros::Publisher command_pub_0_;

  // ROS messages
  simple_sim::DroneHovergamesControl control_msg_;
};

#endif  // HOVERGAMES_SIMPLESIM_INTERFACE_H
