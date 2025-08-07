#ifndef MPC_MODULES_OBJECTIVES_REFERENCE_TRAJECTORY_H
#define MPC_MODULES_OBJECTIVES_REFERENCE_TRAJECTORY_H

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>
#include <mpc_modules/types/module.h>
#include <mpc_modules/types/realtime_data.h>
#include <mpc_msgs/MpcHeader.h>
#include <mpc_msgs/TrajectoryOverHorizon.h>
#include <mpc_tools/ros_visuals.h>
#include <ros/ros.h>

#include <memory>  // std::unique_ptr
#include <vector>  // std::vector

class ReferenceTrajectory : public ControllerModule {
 public:
  ReferenceTrajectory(std::string name, ros::NodeHandle &nh,
                      ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
                      DataSaverJson *ptr_data_saver_json);

  bool ObjectiveReached(const RealTimeData &data) override;
  void OnDataReceived(RealTimeData &data, std::string data_name) override;
  bool ReadyForControl(const RealTimeData &data) override;
  void Update(RealTimeData &data) override;
  void SetParameters(const RealTimeData &data) override;
  void PublishData(const mpc_msgs::MpcHeader &mpc_header) override;
  void ExportDataJson(const int count_since_start,
                      const int count_total) override;
  void CreateVisualizations() override;
  void PublishVisualizations() override;
  void OnReset() override;

  void SetWarmStart(
      std::unique_ptr<std::vector<std::vector<double>>> &custom_plan);

 private:
  // ROS recording publishers and messages
  ros::Publisher reference_trajectory_rec_pub_;
  mpc_msgs::TrajectoryOverHorizon reference_trajectory_msg_;

  // Reference trajectory (x,u)
  std::vector<std::vector<double>> received_ref_traj_u_;
  std::vector<std::vector<double>> received_ref_traj_x_;
  std::vector<std::vector<double>> ref_traj_u_;
  std::vector<std::vector<double>> ref_traj_x_;

  // Indication whether control loop can be started
  bool first_ref_traj_received_;

  // Number of inputs to check if slack is used
  int nu_;

  // Counter of amount of times the controller has run => to shift the reference
  // trajectory forward in time
  int n_shift_;

  // HMPC variables
  int n_it_valid_ = 0, n_it_valid_max_ = 0;

  // ROS visuals publisher
  std::unique_ptr<ROSMarkerPublisher> reference_trajectory_marker_pub_,
      reference_trajectory_ground_marker_pub_;
};

#endif  // REFERENCE_TRAJECTORY_H
