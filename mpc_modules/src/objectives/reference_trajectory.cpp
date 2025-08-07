#include <mpc_modules/objectives/reference_trajectory.h>
#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>

#include <Eigen/Eigen>

ReferenceTrajectory::ReferenceTrajectory(std::string name, ros::NodeHandle &nh,
                                         ConfigurationMPC *ptr_config,
                                         SolverBase *ptr_solver,
                                         DataSaverJson *ptr_data_saver_json)
    : ControllerModule(ModuleType::OBJECTIVE, name, nh, ptr_config, ptr_solver,
                       ptr_data_saver_json) {
  MPC_WARN_ALWAYS("Initializing " << name_ << " module");

  first_ref_traj_received_ = false;

  // If we are using slack we need to dismiss it
  bool use_slack = ptr_solver_->getConstantScalarBool("use_slack");
  if (use_slack) {
    nu_ = ptr_solver_->nu_ - 1;
  } else {
    nu_ = ptr_solver_->nu_;
  }

  ref_traj_u_.resize(ptr_solver_->nbar_);
  ref_traj_x_.resize(ptr_solver_->nbar_);
  for (int k = 0; k < ptr_solver_->nbar_; k++) {
    ref_traj_u_[k].resize(nu_);
    ref_traj_x_[k].resize(ptr_solver_->nx_);
  }

  // Make sure the later constructed initial guess is provided to the solver
  ptr_config_->use_custom_initial_guess_ = true;

  // Initialize ROS recording publishers and messages
  if (ptr_config_->record_reference_trajectory_) {
    reference_trajectory_rec_pub_ =
        nh.advertise<mpc_msgs::TrajectoryOverHorizon>(
            ptr_config_->reference_trajectory_rec_topic_, 1);
    reference_trajectory_msg_.header.frame_id =
        ptr_config_->robot_target_frame_;
    reference_trajectory_msg_.u_names.assign(
        nu_, "");  // names are not known in the module
    reference_trajectory_msg_.x_names.assign(
        ptr_solver_->nx_, "");  // names are not known in the module
    reference_trajectory_msg_.traj.resize((nu_ + ptr_solver_->nx_) *
                                          ptr_solver_->nbar_);
  }

  // ROS visuals reference trajectory publisher
  reference_trajectory_marker_pub_.reset(new ROSMarkerPublisher(
      nh, ptr_config_->reference_trajectory_vis_topic_.c_str(),
      ptr_config_->robot_target_frame_, 2 * ptr_solver_->n_ + 1));
  reference_trajectory_ground_marker_pub_.reset(new ROSMarkerPublisher(
      nh, ptr_config_->reference_trajectory_ground_vis_topic_.c_str(),
      ptr_config_->robot_target_frame_, 2 * ptr_solver_->n_ + 1));

  MPC_WARN_ALWAYS(name_ << " module initialized");
}

bool ReferenceTrajectory::ObjectiveReached(const RealTimeData &data) {
  // In hierarchical MPC, where this module is used as lower-level tracking
  // controller, the planner determines whether the objective is reached
  if (ptr_config_->n_layers_ > 1 && ptr_config_->layer_idx_ == 0) {
    return false;
  }
  // Otherwise: one might want to check for reaching the terminal set

  return true;
}

void ReferenceTrajectory::OnDataReceived(RealTimeData &data,
                                         std::string data_name) {
  if (data_name == "ReferenceTrajectory") {
    PROFILE_AND_LOG_INFO("ReferenceTrajectory::OnDataReceived()");

    // Keep track of receiving the first reference trajectory
    if (!first_ref_traj_received_) {
      MPC_INFO("Received first reference trajectory");
      first_ref_traj_received_ = true;
    }

    // Determine length of received reference trajectory
    int ref_size = data.ref_traj_x_.size();

    // Initialize input trajectory
    received_ref_traj_u_.resize(ref_size);
    for (int i = 0; i < ref_size; i++) {
      received_ref_traj_u_[i].resize(nu_);
    }

    // Initialize state trajectory
    received_ref_traj_x_.resize(ref_size);
    for (int i = 0; i < ref_size; i++) {
      received_ref_traj_x_[i].resize(ptr_solver_->nx_);
    }

    // Store stage trajectories (u,x)
    for (int k = 0; k < ref_size; k++) {
      for (int l = 0; l < nu_; l++) {
        received_ref_traj_u_[k][l] = data.ref_traj_u_[k][l];
      }
      for (int l = 0; l < ptr_solver_->nx_; l++) {
        received_ref_traj_x_[k][l] = data.ref_traj_x_[k][l];
      }
    }

    // Set data validity
    // Note: initially, this is the same as the MPC layer ratio
    n_it_valid_max_ = data.ref_traj_n_it_valid_;
    n_it_valid_ = n_it_valid_max_;
  }
}

bool ReferenceTrajectory::ReadyForControl(const RealTimeData &data) {
  return first_ref_traj_received_ && n_it_valid_ > 0;
}

void ReferenceTrajectory::Update(RealTimeData &data) {
  // Each controller iteration: shift forward reference trajectory one stage
  // more
  int n_shift_ = n_it_valid_max_ - n_it_valid_;

  /* Store selected part of received reference trajectories for current
   * controller iteration (for visualization later on) */
  // Insert reference inputs
  for (int k = 0; k < ptr_solver_->n_; k++) {
    for (int l = 0; l < nu_; l++) {
      // Store
      ref_traj_u_[k][l] = received_ref_traj_u_[k + n_shift_][l];
    }
  }

  // Insert dummy terminal reference inputs
  for (int l = 0; l < nu_; l++) {
    // Store
    ref_traj_u_[ptr_solver_->n_][l] = 0;
  }

  // Insert reference states
  for (int k = 0; k < ptr_solver_->nbar_; k++) {
    for (int l = 0; l < ptr_solver_->nx_; l++) {
      // Store
      ref_traj_x_[k][l] = received_ref_traj_x_[k + n_shift_][l];
    }
  }

  if (n_it_valid_ > 0) n_it_valid_ -= 1;
}

void ReferenceTrajectory::SetParameters(const RealTimeData &data) {
  /* Store selected part of received reference trajectories for current
   * controller iteration (for visualization later on) */
  // Insert reference inputs
  int param_idx = 0;
  for (int k = 0; k < ptr_solver_->n_; k++) {
    for (int l = 0; l < nu_; l++) {
      // Set parameter
      ptr_solver_->par_objectives_(param_idx++, k) = ref_traj_u_[k][l];
    }
    param_idx = 0;
  }

  // Insert dummy terminal reference inputs
  param_idx = 0;
  for (int l = 0; l < nu_; l++) {
    // Set parameter
    ptr_solver_->par_objectives_(param_idx++, ptr_solver_->n_) =
        ref_traj_u_[ptr_solver_->n_][l];
  }

  // Insert reference states
  param_idx = nu_;
  for (int k = 0; k < ptr_solver_->nbar_; k++) {
    for (int l = 0; l < ptr_solver_->nx_; l++) {
      // Set parameter
      ptr_solver_->par_objectives_(param_idx++, k) = ref_traj_x_[k][l];
    }
    param_idx = nu_;
  }
}

void ReferenceTrajectory::PublishData(const mpc_msgs::MpcHeader &mpc_header) {
  if (ptr_config_->record_reference_trajectory_) {
    PROFILE_AND_LOG_INFO("ReferenceTrajectory::PublishData");

    reference_trajectory_msg_.header.stamp = ros::Time::now();
    reference_trajectory_msg_.mpc_header = mpc_header;
    for (int k = 0; k < ptr_solver_->nbar_; k++) {
      for (int l = 0; l < nu_; l++) {
        reference_trajectory_msg_.traj[k * (nu_ + ptr_solver_->nx_) + l] =
            ref_traj_u_[k][l];
      }
      for (int l = 0; l < ptr_solver_->nx_; l++) {
        reference_trajectory_msg_.traj[k * (nu_ + ptr_solver_->nx_) + nu_ + l] =
            ref_traj_x_[k][l];
      }
    }
    reference_trajectory_rec_pub_.publish(reference_trajectory_msg_);
  }
}

void ReferenceTrajectory::ExportDataJson(const int count_since_start,
                                         const int count_total) {
  PROFILE_FUNCTION();
}

void ReferenceTrajectory::CreateVisualizations() {
  Eigen::Vector3d pose, previous_pose;

  // Create reference trajectory markers if desired
  if (ptr_config_->draw_reference_trajectory_ground_air_ >= 2) {
    ROSPointMarker &traj_points =
        reference_trajectory_marker_pub_->getNewPointMarker("CYLINDER");
    traj_points.setColor(ptr_config_->reference_trajectory_marker_color_[0],
                         ptr_config_->reference_trajectory_marker_color_[1],
                         ptr_config_->reference_trajectory_marker_color_[2],
                         ptr_config_->reference_trajectory_marker_color_[3]);
    traj_points.setScale(ptr_config_->reference_trajectory_marker_scale_[0],
                         ptr_config_->reference_trajectory_marker_scale_[1],
                         ptr_config_->reference_trajectory_marker_scale_[2]);

    ROSLine &line = reference_trajectory_marker_pub_->getNewLine();
    line.setColor(ptr_config_->reference_trajectory_marker_color_[0],
                  ptr_config_->reference_trajectory_marker_color_[1],
                  ptr_config_->reference_trajectory_marker_color_[2],
                  ptr_config_->reference_trajectory_marker_color_[3]);
    line.setScale(0.5 * ptr_config_->reference_trajectory_marker_scale_[0]);

    for (int k = 0; k < ptr_solver_->nbar_; k++) {
      pose << ref_traj_x_[k][0], ref_traj_x_[k][1], ref_traj_x_[k][2];
      traj_points.addPointMarker(Eigen::Vector3d(pose(0), pose(1), pose(2)));
      if (k > 0) {
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1),
                                     previous_pose(2)),
                     Eigen::Vector3d(pose(0), pose(1), pose(2)));
      }
      previous_pose = pose;
    }
  }

  // Create reference trajectory ground markers if desired
  if (ptr_config_->draw_reference_trajectory_ground_air_ == 1 ||
      ptr_config_->draw_reference_trajectory_ground_air_ >= 3) {
    ROSPointMarker &traj_points_ground =
        reference_trajectory_ground_marker_pub_->getNewPointMarker("CYLINDER");
    traj_points_ground.setColor(
        ptr_config_->reference_trajectory_marker_color_[0],
        ptr_config_->reference_trajectory_marker_color_[1],
        ptr_config_->reference_trajectory_marker_color_[2],
        ptr_config_->ground_projection_marker_alpha_);
    traj_points_ground.setScale(
        ptr_config_->reference_trajectory_marker_scale_[0],
        ptr_config_->reference_trajectory_marker_scale_[1],
        ptr_config_->reference_trajectory_marker_scale_[2]);

    ROSLine &line_ground =
        reference_trajectory_ground_marker_pub_->getNewLine();
    line_ground.setColor(ptr_config_->reference_trajectory_marker_color_[0],
                         ptr_config_->reference_trajectory_marker_color_[1],
                         ptr_config_->reference_trajectory_marker_color_[2],
                         ptr_config_->ground_projection_marker_alpha_);
    line_ground.setScale(0.5 *
                         ptr_config_->reference_trajectory_marker_scale_[0]);

    for (int k = 0; k < ptr_solver_->nbar_; k++) {
      pose << ref_traj_x_[k][0], ref_traj_x_[k][1], ref_traj_x_[k][2];
      traj_points_ground.addPointMarker(Eigen::Vector3d(
          pose(0), pose(1), ptr_config_->ground_projection_marker_z_));
      if (k > 0) {
        line_ground.addLine(
            Eigen::Vector3d(previous_pose(0), previous_pose(1),
                            ptr_config_->ground_projection_marker_z_),
            Eigen::Vector3d(pose(0), pose(1),
                            ptr_config_->ground_projection_marker_z_));
      }
      previous_pose = pose;
    }
  }
}

void ReferenceTrajectory::PublishVisualizations() {
  reference_trajectory_marker_pub_->publish();
  reference_trajectory_ground_marker_pub_->publish();
}

void ReferenceTrajectory::OnReset() {
  first_ref_traj_received_ = false;
  n_it_valid_ = 0;
}

void ReferenceTrajectory::SetWarmStart(
    std::unique_ptr<std::vector<std::vector<double>>> &custom_plan) {
  // Construct the initial guess
  custom_plan.reset(new std::vector<std::vector<double>>(
      ptr_solver_->nbar_, std::vector<double>(nu_ + ptr_solver_->nx_)));
  for (int k = 0; k < ptr_solver_->nbar_; k++) {
    for (int l = 0; l < nu_; l++) {
      custom_plan->at(k)[l] = ref_traj_u_[k][l];
    }
    for (int l = 0; l < ptr_solver_->nx_; l++) {
      custom_plan->at(k)[nu_ + l] = ref_traj_x_[k][l];
    }
  }
}
