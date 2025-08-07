#ifndef MPC_MODULES_OBJECTIVES_GOAL_ORIENTED
#define MPC_MODULES_OBJECTIVES_GOAL_ORIENTED

#include <geometry_msgs/PoseStamped.h>
#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>
#include <mpc_modules/types/module.h>
#include <mpc_modules/types/realtime_data.h>
#include <mpc_msgs/GoalPose.h>
#include <mpc_msgs/MpcHeader.h>
#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/ros_visuals.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <deque>

class GoalOriented : public ControllerModule {
 public:
  GoalOriented(std::string name, ros::NodeHandle& nh,
               ConfigurationMPC* ptr_config, SolverBase* ptr_solver,
               DataSaverJson* ptr_data_saver_json);

  bool ObjectiveReached(const RealTimeData& data) override;
  void OnDataReceived(RealTimeData& data, std::string data_name) override;
  bool ReadyForControl(const RealTimeData& data) override;
  void Update(RealTimeData& data) override;
  void SetParameters(const RealTimeData& data) override;
  void PublishData(const mpc_msgs::MpcHeader& mpc_header) override;
  void ExportDataJson(const int count_since_start,
                      const int count_total) override;
  void CreateVisualizations() override;
  void PublishVisualizations() override;
  void OnReset() override;

  // Module specific functions
  void GoalCallback(geometry_msgs::PoseStamped goal);

 private:
  // ROS publishers and subscribers
  ros::Subscriber goal_sub_;

  // ROS recording publishers and messages
  ros::Publisher goal_position_rec_pub_;
  mpc_msgs::GoalPose goal_position_msg_;

  // Data indices in the solver matrices
  int x_position_, y_position_, z_position_, yaw_position_;
  int goal_x_position_, goal_y_position_, goal_z_position_, goal_yaw_position_;

  // Variables to keep track of pre-defined goals
  // Pre-defined goals are used to achieve deterministic behavior
  std::vector<geometry_msgs::PoseStamped> predefined_goals_;
  int n_predefined_goals_ = 0, predefined_goal_index_ = -1;
  bool reached_predefined_goal_ = false;

  // Variables to keep track of received goals
  bool received_new_goal_ = false, reached_received_goal_ = true;
  geometry_msgs::PoseStamped received_goal_;

  // Variables to update goals
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  geometry_msgs::PoseStamped current_goal_;
  int module_update_cnt_ = 0;

  // ROS visuals publisher
  std::unique_ptr<ROSMarkerPublisher> goal_position_marker_pub_,
      goal_position_ground_marker_pub_;
};

#endif  // MPC_MODULES_OBJECTIVES_GOAL_ORIENTED
