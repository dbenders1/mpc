#include "mpc_modules/objectives/goal_oriented.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>


GoalOriented::GoalOriented(std::string name, ros::NodeHandle &nh, ConfigurationMPC *ptr_config, SolverBase *ptr_solver, DataSaverJson* ptr_data_saver_json): ControllerModule(ModuleType::OBJECTIVE, name, nh, ptr_config, ptr_solver, ptr_data_saver_json)
{
    MPC_WARN_ALWAYS("Initializing " << name_ << " module");

    // Subscriber for goal data
    goal_sub_ = nh.subscribe(ptr_config_->goal_topic_, 1, &GoalOriented::GoalCallback, this);

    // Initialize ROS recording publishers and messages
    if (ptr_config_->record_goal_position_)
    {
        goal_position_rec_pub_ = nh.advertise<mpc_msgs::GoalPose>(ptr_config_->goal_position_rec_topic_, 1);
        goal_position_msg_.header.frame_id = ptr_config_->robot_target_frame_;
    }

    // Check the solver data positions needed for this module
    std::vector<int> indexes = ptr_solver_->returnDataPositionsState({"x","y","z","psi"});
    x_position_ = indexes[0];
    y_position_ = indexes[1];
    z_position_ = indexes[2];
    yaw_position_ = indexes[3];

    // Check objective positions in solver
    indexes = ptr_solver_->returnDataPositionsObjective({"goal_x","goal_y","goal_z","goal_yaw"});
    goal_x_position_ = indexes[0];
    goal_y_position_ = indexes[1];
    goal_z_position_ = indexes[2];
    goal_yaw_position_ = indexes[3];

    // Initialize ROS visuals
    goal_position_marker_pub_.reset(new ROSMarkerPublisher(nh_, ptr_config_->goal_position_vis_topic_.c_str(), ptr_config_->robot_target_frame_, 1));
    goal_position_ground_marker_pub_.reset(new ROSMarkerPublisher(nh_, ptr_config_->goal_position_ground_vis_topic_.c_str(), ptr_config_->robot_target_frame_, 1));

    MPC_WARN_ALWAYS(name_ << " module initialized");
}

bool GoalOriented::ObjectiveReached(const RealTimeData& data)
{
    MPC_INFO("GoalOriented::ObjectiveReached()");

    return false;
}

void GoalOriented::OnDataReceived(RealTimeData& data, std::string data_name)
{
    PROFILE_AND_LOG_INFO("GoalOriented::OnDataReceived()");

    // If started but no way point, set position as way point until new way point has arrived.
}

bool GoalOriented::ReadyForControl(const RealTimeData& data)
{
    MPC_INFO("GoalOriented::ReadyForControl()");

    return true;
}

void GoalOriented::Update(RealTimeData& data)
{
    PROFILE_AND_LOG_INFO("GoalOriented::Update()");

    // WE can do here is goal reach publish true??
    // Note: +1 is used, because stage_vars_ contains the variables of the previous run and we need the current initial state
    if (!received_goal_ && first_check_)
    {
    stored_goal_.pose.position.x = ptr_solver_->stage_vars_(x_position_,1);
    stored_goal_.pose.position.y = ptr_solver_->stage_vars_(y_position_,1);
    stored_goal_.pose.position.z = ptr_solver_->stage_vars_(z_position_,1);

    tf2::Quaternion q;
    double yaw = ptr_solver_->stage_vars_(yaw_position_,1);
    q.setRPY(0.0, 0.0, yaw);

    stored_goal_.pose.orientation.x = q.x();
    stored_goal_.pose.orientation.y = q.y();
    stored_goal_.pose.orientation.z = q.z();
    stored_goal_.pose.orientation.w = q.w();


    first_check_ = false;
    }
}

void GoalOriented::SetParameters(const RealTimeData& data)
{
    MPC_INFO("GoalOriented::SetParameters()");

    tf2::Quaternion q(stored_goal_.pose.orientation.x,
    stored_goal_.pose.orientation.y,
    stored_goal_.pose.orientation.z,
    stored_goal_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    for (int i = 0; i < ptr_solver_->nbar_; ++i)
    {
    ptr_solver_->par_objectives_(goal_x_position_,i) = stored_goal_.pose.position.x;
    ptr_solver_->par_objectives_(goal_y_position_,i) = stored_goal_.pose.position.y;
    ptr_solver_->par_objectives_(goal_z_position_,i) = stored_goal_.pose.position.z;
    ptr_solver_->par_objectives_(goal_yaw_position_,i) = yaw;
    }
}

void GoalOriented::PublishData(const mpc_msgs::MpcHeader &mpc_header)
{
    if (ptr_config_->record_goal_position_)
    {
        PROFILE_AND_LOG_INFO("GoalOriented::PublishData");

        goal_position_msg_.header.stamp = ros::Time::now();
        goal_position_msg_.mpc_header = mpc_header;
        goal_position_msg_.pose = stored_goal_.pose;
        goal_position_rec_pub_.publish(goal_position_msg_);
    }
}

void GoalOriented::ExportDataJson(const int count_since_start, const int count_total)
{
    PROFILE_AND_LOG_INFO("GoalOriented::ExportDataJson()");
}

void GoalOriented::CreateVisualizations() 
{
    MPC_INFO("GoalOriented::CreateVisualizations()");

    // Create goal position marker if desired
    if (ptr_config_->draw_goal_position_ground_air_ >= 2)
    {
        ROSPointMarker &goal_position = goal_position_marker_pub_->getNewPointMarker("CYLINDER");
        goal_position.setColor(ptr_config_->goal_position_marker_color_[0], ptr_config_->goal_position_marker_color_[1], ptr_config_->goal_position_marker_color_[2], ptr_config_->goal_position_marker_color_[3]);
        goal_position.setScale(ptr_config_->goal_position_marker_scale_[0], ptr_config_->goal_position_marker_scale_[1], ptr_config_->goal_position_marker_scale_[2]);
        goal_position.addPointMarker(Eigen::Vector3d(stored_goal_.pose.position.x, stored_goal_.pose.position.y, stored_goal_.pose.position.z));
    }

    // Create goal position ground marker if desired
    if (ptr_config_->draw_goal_position_ground_air_ == 1 || ptr_config_->draw_goal_position_ground_air_ >= 3)
    {
        ROSPointMarker &goal_position_ground = goal_position_ground_marker_pub_->getNewPointMarker("CYLINDER");
        goal_position_ground.setColor(ptr_config_->goal_position_marker_color_[0], ptr_config_->goal_position_marker_color_[1], ptr_config_->goal_position_marker_color_[2], ptr_config_->ground_projection_marker_alpha_);
        goal_position_ground.setScale(ptr_config_->goal_position_marker_scale_[0], ptr_config_->goal_position_marker_scale_[1], ptr_config_->goal_position_marker_scale_[2]);
        goal_position_ground.addPointMarker(Eigen::Vector3d(stored_goal_.pose.position.x, stored_goal_.pose.position.y, ptr_config_->ground_projection_marker_z_));
    }
}

void GoalOriented::PublishVisualizations()
{
    MPC_INFO("GoalOriented::PublishVisualizations()");

    goal_position_marker_pub_->publish();
    goal_position_ground_marker_pub_->publish();
}

void GoalOriented::OnReset()
{
    // Nothing to do here
}

void GoalOriented::GoalCallback(geometry_msgs::PoseStamped goal)
{
    PROFILE_AND_LOG_INFO("GoalOriented::GoalCallback");

    stored_goal_ = goal;

    received_goal_ = true;
}
