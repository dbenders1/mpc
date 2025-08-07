#include "mpc_modules/objectives/goal_oriented.h"

#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/math.h>
#include <mpc_tools/printing.h>
#include <mpc_tools/traj_gen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

GoalOriented::GoalOriented(std::string name, ros::NodeHandle& nh,
                           ConfigurationMPC* ptr_config, SolverBase* ptr_solver,
                           DataSaverJson* ptr_data_saver_json)
    : ControllerModule(ModuleType::OBJECTIVE, name, nh, ptr_config, ptr_solver,
                       ptr_data_saver_json) {
  MPC_WARN_ALWAYS("Initializing " << name_ << " module");

  // Subscriber for goal data
  goal_sub_ = nh.subscribe(ptr_config_->goal_topic_, 1,
                           &GoalOriented::GoalCallback, this);

  // Initialize ROS recording publishers and messages
  if (ptr_config_->record_goal_position_) {
    goal_position_rec_pub_ = nh.advertise<mpc_msgs::GoalPose>(
        ptr_config_->goal_position_rec_topic_, 1);
    goal_position_msg_.header.frame_id = ptr_config_->robot_target_frame_;
  }

  // Check the solver data positions needed for this module
  std::vector<int> indexes =
      ptr_solver_->returnDataPositionsState({"x", "y", "z", "psi"});
  x_position_ = indexes[0];
  y_position_ = indexes[1];
  z_position_ = indexes[2];
  yaw_position_ = indexes[3];

  // Check objective positions in solver
  indexes = ptr_solver_->returnDataPositionsObjective(
      {"goal_x", "goal_y", "goal_z", "goal_yaw"});
  goal_x_position_ = indexes[0];
  goal_y_position_ = indexes[1];
  goal_z_position_ = indexes[2];
  goal_yaw_position_ = indexes[3];

  // Initialize the goals matrix
  // Design choice: either we initialize all goals, or we receive them, no
  // combination
  if (ptr_config_->auto_send_goals_) {
    if (ptr_config_->goals_trajectory_ == "none") {
      MPC_INFO("No goals trajectory specified, using discrete goals instead");
      n_predefined_goals_ = ptr_config_->goals_x_.size();
      int n_y_goals = ptr_config_->goals_y_.size();
      int n_z_goals = ptr_config_->goals_z_.size();
      int n_yaw_goals = ptr_config_->goals_yaw_.size();
      // Print error if there is at least one difference in number of goals
      if (n_predefined_goals_ != n_y_goals ||
          n_predefined_goals_ != n_z_goals ||
          n_predefined_goals_ != n_yaw_goals) {
        MPC_WARN(
            "The number of goals in x, y, z and yaw must be equal. "
            "Received: "
            << "n_x_goals: " << n_predefined_goals_ << ", " << "n_y_goals: "
            << n_y_goals << ", " << "n_z_goals: " << n_z_goals << ", "
            << "n_yaw_goals: " << n_yaw_goals
            << ". Ignoring goals initialization");
      } else {
        predefined_goals_.resize(n_predefined_goals_);
        for (int i = 0; i < n_predefined_goals_; ++i) {
          predefined_goals_[i].pose.position.x = ptr_config_->goals_x_[i];
          predefined_goals_[i].pose.position.y = ptr_config_->goals_y_[i];
          predefined_goals_[i].pose.position.z = ptr_config_->goals_z_[i];
          tf2::Quaternion q;
          double yaw = ptr_config_->goals_yaw_[i];
          q.setRPY(0.0, 0.0, yaw);
          predefined_goals_[i].pose.orientation.x = q.x();
          predefined_goals_[i].pose.orientation.y = q.y();
          predefined_goals_[i].pose.orientation.z = q.z();
          predefined_goals_[i].pose.orientation.w = q.w();
        }
      }
    } else if (ptr_config_->goals_trajectory_ == "circle_r1_f0dot1_cw" ||
               ptr_config_->goals_trajectory_ == "circle_r1_f0dot1_ccw" ||
               ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot1_cw" ||
               ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot1_ccw" ||
               ptr_config_->goals_trajectory_ == "circle_r1_f0dot3_cw" ||
               ptr_config_->goals_trajectory_ == "circle_r1_f0dot3_ccw" ||
               ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot3_cw" ||
               ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot3_ccw") {
      // Predefined goals trajectory
      MPC_INFO("Using goals trajectory: " << ptr_config_->goals_trajectory_);
      Eigen::MatrixXd traj;
      if (ptr_config_->goals_trajectory_ == "circle_r1_f0dot1_cw") {
        traj = circleTrajectory(1.0, 0.1, ptr_solver_->dt_, true);
      } else if (ptr_config_->goals_trajectory_ == "circle_r1_f0dot1_ccw") {
        traj = circleTrajectory(1.0, 0.1, ptr_solver_->dt_, false);
      } else if (ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot1_cw") {
        traj = lemniscateTrajectory(1.0, 0.1, ptr_solver_->dt_, true);
      } else if (ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot1_ccw") {
        traj = lemniscateTrajectory(1.0, 0.1, ptr_solver_->dt_, false);
      } else if (ptr_config_->goals_trajectory_ == "circle_r1_f0dot3_cw") {
        traj = circleTrajectory(1.0, 0.3, ptr_solver_->dt_, true);
      } else if (ptr_config_->goals_trajectory_ == "circle_r1_f0dot3_ccw") {
        traj = circleTrajectory(1.0, 0.3, ptr_solver_->dt_, false);
      } else if (ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot3_cw") {
        traj = lemniscateTrajectory(1.0, 0.3, ptr_solver_->dt_, true);
      } else if (ptr_config_->goals_trajectory_ == "lemniscate_r1_f0dot3_ccw") {
        traj = lemniscateTrajectory(1.0, 0.3, ptr_solver_->dt_, false);
      }
      n_predefined_goals_ = traj.cols();
      predefined_goals_.resize(n_predefined_goals_);
      for (int i = 0; i < n_predefined_goals_; ++i) {
        predefined_goals_[i].pose.position.x = traj(0, i);
        predefined_goals_[i].pose.position.y = traj(1, i);
        predefined_goals_[i].pose.position.z = traj(2, i);
        predefined_goals_[i].pose.orientation.x = 0.0;
        predefined_goals_[i].pose.orientation.y = 0.0;
        predefined_goals_[i].pose.orientation.z = 0.0;
        predefined_goals_[i].pose.orientation.w = 1.0;
      }
    } else {
      MPC_WARN("Unknown goals trajectory specified: "
               << ptr_config_->goals_trajectory_ << ".");
    }
  }

  // Initialize ROS visuals
  goal_position_marker_pub_.reset(
      new ROSMarkerPublisher(nh_, ptr_config_->goal_position_vis_topic_.c_str(),
                             ptr_config_->robot_target_frame_, 1));
  goal_position_ground_marker_pub_.reset(new ROSMarkerPublisher(
      nh_, ptr_config_->goal_position_ground_vis_topic_.c_str(),
      ptr_config_->robot_target_frame_, 1));

  MPC_WARN_ALWAYS(name_ << " module initialized");
}

bool GoalOriented::ObjectiveReached(const RealTimeData& data) {
  MPC_INFO("GoalOriented::ObjectiveReached()");

  return false;
}

void GoalOriented::OnDataReceived(RealTimeData& data, std::string data_name) {
  PROFILE_AND_LOG_INFO("GoalOriented::OnDataReceived()");

  // Update current position
  if (data_name == "Position") {
    current_position_ = data.pos_;
  }
}

bool GoalOriented::ReadyForControl(const RealTimeData& data) {
  MPC_INFO("GoalOriented::ReadyForControl()");

  return true;
}

void GoalOriented::Update(RealTimeData& data) {
  PROFILE_AND_LOG_INFO("GoalOriented::Update()");

  // In the first run, we just want to hover at the current position
  // In the rest of the runs, we either want to reach a pre-defined goal or a
  // received goal
  // Only update the current goal if there is a new goal available
  // If we have reached the last goal, we do not have to update the current goal
  if (module_update_cnt_ == 0) {
    current_goal_.pose.position.x = ptr_solver_->stage_vars_(x_position_, 1);
    current_goal_.pose.position.y = ptr_solver_->stage_vars_(y_position_, 1);
    current_goal_.pose.position.z = ptr_solver_->stage_vars_(z_position_, 1);

    tf2::Quaternion q;
    double yaw = ptr_solver_->stage_vars_(yaw_position_, 1);
    q.setRPY(0.0, 0.0, yaw);
    current_goal_.pose.orientation.x = q.x();
    current_goal_.pose.orientation.y = q.y();
    current_goal_.pose.orientation.z = q.z();
    current_goal_.pose.orientation.w = q.w();
  } else {
    if (ptr_config_->auto_send_goals_) {
      // Determine if we have reached the current goal if we didn't reach it
      // so far
      if (euclideanDistance(current_position_,
                            Eigen::Vector3d(current_goal_.pose.position.x,
                                            current_goal_.pose.position.y,
                                            current_goal_.pose.position.z)) <
              ptr_config_->goal_reached_threshold_ &&
          !reached_predefined_goal_) {
        reached_predefined_goal_ = true;
        if (module_update_cnt_ > 1)
          MPC_WARN("Reached predefined goal "
                   << predefined_goal_index_ << ": "
                   << current_goal_.pose.position.x << ", "
                   << current_goal_.pose.position.y << ", "
                   << current_goal_.pose.position.z);
      }
      // If we have reached the current goal and there are more goals to go,
      // update the current goal
      if (reached_predefined_goal_ &&
          predefined_goal_index_ < n_predefined_goals_ - 1) {
        predefined_goal_index_++;
        current_goal_.pose.position.x =
            predefined_goals_[predefined_goal_index_].pose.position.x;
        current_goal_.pose.position.y =
            predefined_goals_[predefined_goal_index_].pose.position.y;
        current_goal_.pose.position.z =
            predefined_goals_[predefined_goal_index_].pose.position.z;
        current_goal_.pose.orientation.x =
            predefined_goals_[predefined_goal_index_].pose.orientation.x;
        current_goal_.pose.orientation.y =
            predefined_goals_[predefined_goal_index_].pose.orientation.y;
        current_goal_.pose.orientation.z =
            predefined_goals_[predefined_goal_index_].pose.orientation.z;
        current_goal_.pose.orientation.w =
            predefined_goals_[predefined_goal_index_].pose.orientation.w;
        reached_predefined_goal_ = false;
        MPC_WARN("Moving to predefined goal "
                 << predefined_goal_index_ << ": "
                 << current_goal_.pose.position.x << ", "
                 << current_goal_.pose.position.y << ", "
                 << current_goal_.pose.position.z);
      }
    } else {
      // Determine if we have reached the current goal if we didn't reach it
      // so far
      if (euclideanDistance(current_position_,
                            Eigen::Vector3d(current_goal_.pose.position.x,
                                            current_goal_.pose.position.y,
                                            current_goal_.pose.position.z)) <
              ptr_config_->goal_reached_threshold_ &&
          !reached_received_goal_) {
        reached_received_goal_ = true;
        MPC_WARN("Reached received goal: "
                 << received_goal_.pose.position.x << ", "
                 << received_goal_.pose.position.y << ", "
                 << received_goal_.pose.position.z);
      }
      if (received_new_goal_) {
        // If we have received a new goal, set it as the current goal
        current_goal_ = received_goal_;
        received_new_goal_ = false;
        reached_received_goal_ = false;
        MPC_WARN("Moving to next received goal: "
                 << current_goal_.pose.position.x << ", "
                 << current_goal_.pose.position.y << ", "
                 << current_goal_.pose.position.z);
      }
    }
  }
  module_update_cnt_++;
}

void GoalOriented::SetParameters(const RealTimeData& data) {
  MPC_INFO("GoalOriented::SetParameters()");

  tf2::Quaternion q(
      current_goal_.pose.orientation.x, current_goal_.pose.orientation.y,
      current_goal_.pose.orientation.z, current_goal_.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  for (int i = 0; i < ptr_solver_->nbar_; ++i) {
    ptr_solver_->par_objectives_(goal_x_position_, i) =
        current_goal_.pose.position.x;
    ptr_solver_->par_objectives_(goal_y_position_, i) =
        current_goal_.pose.position.y;
    ptr_solver_->par_objectives_(goal_z_position_, i) =
        current_goal_.pose.position.z;
    ptr_solver_->par_objectives_(goal_yaw_position_, i) = yaw;
  }
}

void GoalOriented::PublishData(const mpc_msgs::MpcHeader& mpc_header) {
  if (ptr_config_->record_goal_position_) {
    PROFILE_AND_LOG_INFO("GoalOriented::PublishData");

    goal_position_msg_.header.stamp = ros::Time::now();
    goal_position_msg_.mpc_header = mpc_header;
    goal_position_msg_.pose = current_goal_.pose;
    goal_position_rec_pub_.publish(goal_position_msg_);
  }
}

void GoalOriented::ExportDataJson(const int count_since_start,
                                  const int count_total) {
  PROFILE_AND_LOG_INFO("GoalOriented::ExportDataJson()");
}

void GoalOriented::CreateVisualizations() {
  MPC_INFO("GoalOriented::CreateVisualizations()");

  // Create goal position marker if desired
  if (ptr_config_->draw_goal_position_ground_air_ >= 2) {
    ROSPointMarker& goal_position =
        goal_position_marker_pub_->getNewPointMarker("CYLINDER");
    goal_position.setColor(ptr_config_->goal_position_marker_color_[0],
                           ptr_config_->goal_position_marker_color_[1],
                           ptr_config_->goal_position_marker_color_[2],
                           ptr_config_->goal_position_marker_color_[3]);
    goal_position.setScale(ptr_config_->goal_position_marker_scale_[0],
                           ptr_config_->goal_position_marker_scale_[1],
                           ptr_config_->goal_position_marker_scale_[2]);
    goal_position.addPointMarker(Eigen::Vector3d(
        current_goal_.pose.position.x, current_goal_.pose.position.y,
        current_goal_.pose.position.z));
  }

  // Create goal position ground marker if desired
  if (ptr_config_->draw_goal_position_ground_air_ == 1 ||
      ptr_config_->draw_goal_position_ground_air_ >= 3) {
    ROSPointMarker& goal_position_ground =
        goal_position_ground_marker_pub_->getNewPointMarker("CYLINDER");
    goal_position_ground.setColor(ptr_config_->goal_position_marker_color_[0],
                                  ptr_config_->goal_position_marker_color_[1],
                                  ptr_config_->goal_position_marker_color_[2],
                                  ptr_config_->ground_projection_marker_alpha_);
    goal_position_ground.setScale(ptr_config_->goal_position_marker_scale_[0],
                                  ptr_config_->goal_position_marker_scale_[1],
                                  ptr_config_->goal_position_marker_scale_[2]);
    goal_position_ground.addPointMarker(Eigen::Vector3d(
        current_goal_.pose.position.x, current_goal_.pose.position.y,
        ptr_config_->ground_projection_marker_z_));
  }
}

void GoalOriented::PublishVisualizations() {
  MPC_INFO("GoalOriented::PublishVisualizations()");

  goal_position_marker_pub_->publish();
  goal_position_ground_marker_pub_->publish();
}

void GoalOriented::OnReset() {
  // Nothing to do here
}

void GoalOriented::GoalCallback(geometry_msgs::PoseStamped goal) {
  PROFILE_AND_LOG_INFO("GoalOriented::GoalCallback");

  if (!ptr_config_->auto_send_goals_) {
    received_new_goal_ = true;
    received_goal_ = goal;
  } else {
    MPC_WARN("Using predefined goals, ignoring received goal");
  }
}
