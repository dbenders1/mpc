#ifndef HOVERGAMES_PX4_INTERFACE_H
#define HOVERGAMES_PX4_INTERFACE_H

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <mpc_hovergames/hovergames_drone_interface.h>
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <std_srvs/Trigger.h>

class HovergamesPX4Interface : public HovergamesDroneInterface {
 public:
  // Constructor and destructor
  HovergamesPX4Interface(ros::NodeHandle &nh, ControllerBase *ptr_controller,
                         ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
                         ModuleHandlerBase *ptr_module_handler,
                         RobotRegion *ptr_robot_region,
                         DataSaverJson *ptr_data_saver_json, std::mutex *mutex);
  ~HovergamesPX4Interface() {};

  // Drone interface methods
  void OnStateCallback(const nav_msgs::Odometry &msg) override;
  void OnComputeActuation() override;
  void OnActuate() override;
  void OnDeployObjectiveReachedStrategy() override;
  void OnDeployEmergencyStrategy() override;
  void OnActuateBrake() override;

  // Hovergames PX4 interface methods
  bool EnableControlCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
  bool DisableControlCallback(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res);
  void batteryVoltageCallback(const sensor_msgs::BatteryState &msg);
  void hoverThrustEstimateCallback(const mavros_msgs::HoverThrustEstimate &msg);
  void imuCallback(const sensor_msgs::Imu &msg);
  void timeReferenceCallback(const sensor_msgs::TimeReference &msg);

 private:
  // ROS publishers and subscribers
  ros::Publisher attitude_pub_, position_pub_, acceleration_pub_,
      position_guess_pub_, time_ref_pub_;
  ros::Subscriber hover_thrust_sub_, battery_voltage_sub_, imu_sub_,
      time_ref_sub_;

  // ROS service servers and clients
  ros::ServiceServer enable_control_server_, disable_control_server_;
  ros::ServiceClient mission_finished_client_;

  // ROS messages
  mavros_msgs::AttitudeTarget att_target_msg_;
  mavros_msgs::PositionTarget pos_target_msg_;
  nav_msgs::Odometry position_guess_msg_;
  sensor_msgs::Imu acceleration_msg_;
  sensor_msgs::TimeReference time_ref_msg_;

  // Battery voltage and hover thrust
  double battery_voltage_ = 16.0, hover_thrust_ = 0.674, acc_z_ = 9.81;

  // Actuate brake variables
  double state_x_, state_y_, state_z_, state_yaw_;

  // Thrust scaling
  double thrust_scaling_ = 1.0;
};

#endif  // HOVERGAMES_PX4_INTERFACE_H
