#ifndef MPC_MODULES_OBJECTIVES_GOAL_ORIENTED
#define MPC_MODULES_OBJECTIVES_GOAL_ORIENTED

#include <ros/ros.h>
#include <deque>
#include <geometry_msgs/PoseStamped.h>

#include <mpc_msgs/GoalPose.h>
#include <mpc_msgs/MpcHeader.h>

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>

#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/ros_visuals.h>

#include <mpc_modules/types/module.h>
#include <mpc_modules/types/realtime_data.h>

class GoalOriented : public ControllerModule
{
public:
    GoalOriented(std::string name, ros::NodeHandle &nh, ConfigurationMPC *ptr_config, SolverBase *ptr_solver, DataSaverJson* ptr_data_saver_json);

    bool ObjectiveReached(const RealTimeData& data) override;
    void OnDataReceived(RealTimeData& data, std::string data_name) override;
    bool ReadyForControl(const RealTimeData& data) override;
    void Update(RealTimeData& data) override;
    void SetParameters(const RealTimeData& data) override;
    void PublishData(const mpc_msgs::MpcHeader &mpc_header) override;
    void ExportDataJson(const int count_since_start, const int count_total) override;
    void CreateVisualizations() override;
    void PublishVisualizations() override;
    void OnReset() override;

    // Module specific functions
    void GoalCallback(geometry_msgs::PoseStamped goal);

private:
    // ROS publishers and subscribers
    ros::Subscriber goal_sub_;

    // ROS messages
    geometry_msgs::PoseStamped stored_goal_;

    // ROS recording publishers and messages
    ros::Publisher goal_position_rec_pub_;
    mpc_msgs::GoalPose goal_position_msg_;

    // Data indices in the solver matrices
    int x_position_, y_position_, z_position_, yaw_position_;
    int goal_x_position_, goal_y_position_, goal_z_position_, goal_yaw_position_;

    // Check if we have a goal that we need to track
    bool received_goal_ = false;
    bool first_check_ = true;

    // ROS visuals publisher
    std::unique_ptr<ROSMarkerPublisher> goal_position_marker_pub_, goal_position_ground_marker_pub_;
};

#endif // MPC_MODULES_OBJECTIVES_GOAL_ORIENTED
