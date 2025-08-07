#include <mpc_hovergames/hovergames_drone_interface.h>
#include <mpc_tools/eigen_std.h>
#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>

HovergamesDroneInterface::HovergamesDroneInterface(
    ros::NodeHandle &nh, ControllerBase *ptr_controller,
    ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
    ModuleHandlerBase *ptr_module_handler, RobotRegion *ptr_robot_region,
    DataSaverJson *ptr_data_saver_json, std::mutex *mutex)
    : Interface(nh, ptr_controller, ptr_config, ptr_solver, ptr_robot_region,
                ptr_data_saver_json, mutex),
      ptr_module_handler_(dynamic_cast<ModuleHandler *>(ptr_module_handler)) {
  MPC_WARN_ALWAYS("Initializing hovergames drone interface");

  // Indication whether first state is received
  first_state_received_ = false;

  // Subscriber for sensor data
  state_sub_ = nh.subscribe(ptr_config_->state_topic_, 1,
                            &HovergamesDroneInterface::StateCallback, this);

  // Initialize solver output
  u_steady_state_ = Eigen::Vector4d::Zero();
  u_steady_state_(3) = 9.81;
  u_solver_ = u_steady_state_;

  // Obtain relevant settings from solver
  use_input_rates_ = ptr_solver_->getConstantScalarBool("use_input_rates");
  use_slack_ = ptr_solver_->getConstantScalarBool("use_slack");
  MPC_INFO_STREAM("use_input_rates_: " << use_input_rates_);
  MPC_INFO_STREAM("use_slack_: " << use_slack_);

  // Obtain data positions
  if (use_input_rates_) {
    nx_ = ptr_solver_->nx_ - 3;
  } else {
    nx_ = ptr_solver_->nx_;
  }
  if (use_slack_) {
    slack_idx_ = ptr_solver_->returnDataPositionsState({"slack"})[0];
    nu_ = ptr_solver_->nu_ - 1;
  } else {
    nu_ = ptr_solver_->nu_;
  }

  std::vector<int> idc = ptr_solver_->returnDataPositionsState(
      {"phi_c", "theta_c", "psi_c", "thrust_c"});
  phi_c_idx_ = idc[0];
  theta_c_idx_ = idc[1];
  psi_c_idx_ = idc[2];
  thrust_c_idx_ = idc[3];

  idc = ptr_solver_->returnDataPositionsState(
      {"x", "y", "z", "vx", "vy", "vz", "phi", "theta", "psi", "thrust"});
  x_idx_ = idc[0];
  y_idx_ = idc[1];
  z_idx_ = idc[2];
  vx_idx_ = idc[3];
  vy_idx_ = idc[4];
  vz_idx_ = idc[5];
  phi_idx_ = idc[6];
  theta_idx_ = idc[7];
  psi_idx_ = idc[8];
  thrust_idx_ = idc[9];

  // Load in hoverconditions for the inputs, so we dont have to check for first
  // iteration in control loop
  ptr_solver_->last_updated_stage_vars_(thrust_c_idx_) = 9.81;
  ptr_solver_->last_updated_stage_vars_(thrust_idx_) = 9.81;

  ptr_solver_->stage_vars_.row(thrust_c_idx_) =
      9.81 * Eigen::VectorXd::Ones(ptr_solver_->n_ + 1);
  ptr_solver_->stage_vars_.row(thrust_idx_) =
      9.81 * Eigen::VectorXd::Ones(ptr_solver_->n_ + 1);

  /************ MPC layer-specific initialization **********/
  if (ptr_config_->layer_idx_ == 0) {
    // Obtain relevant settings from solver
    integrator_stepsize_ =
        ptr_solver_->getConstantScalarDouble("integrator_stepsize");
    integrator_steps_ = ptr_solver_->getConstantScalarInt("integrator_steps");
    MPC_INFO_STREAM("integrator_stepsize_: " << integrator_stepsize_);
    MPC_INFO_STREAM("integrator_steps_: " << integrator_steps_);

    // Initialize ROS recording publishers and messages
    if (ptr_config_->record_cur_state_) {
      cur_state_rec_pub_ = nh.advertise<mpc_msgs::Float64Array>(
          ptr_config_->cur_state_rec_topic_, 1);
      cur_state_msg_.header.frame_id = ptr_config_->robot_base_link_;
      cur_state_msg_.array.resize(ptr_solver_->nx_);
    }

    // ROS visuals current position (and corresponding robot region) publisher
    current_position_marker_pub_.reset(new ROSMarkerPublisher(
        nh_, ptr_config_->current_position_vis_topic_.c_str(),
        ptr_config_->robot_target_frame_, 1));
    current_position_ground_marker_pub_.reset(new ROSMarkerPublisher(
        nh_, ptr_config_->current_position_ground_vis_topic_.c_str(),
        ptr_config_->robot_target_frame_, 1));
    current_region_marker_pub_.reset(new ROSMarkerPublisher(
        nh_, ptr_config_->current_region_vis_topic_.c_str(),
        ptr_config_->robot_target_frame_, 1));

    if (ptr_config_->n_layers_ > 1) {
      // Obtain relevant settings from solver
      ratio_ = ptr_solver_->getConstantScalarInt("ratio");
      MPC_INFO_STREAM("ratio_: " << ratio_);

      // Get weighting matrices Q and R from Python, so it can be used for
      // logging it in the static data json recording
      eigenMatrixToStdVector(ptr_solver_->getConstantMatrixDouble("Q"), Q_);
      eigenMatrixToStdVector(ptr_solver_->getConstantMatrixDouble("R"), R_);

      // Start publisher for controller MPC layer to indicate when to start the
      // control loop
      start_mpc_layer_pub_ = nh.advertise<mpc_msgs::StartMpcLayer>(
          ptr_config_->start_mpc_layer_topic_, 1);

      // Subscriber for reference trajectory and constraints
      first_ref_received_ = false;
      ref_sub_ = nh.subscribe(ptr_config_->ref_topic_, 1,
                              &HovergamesDroneInterface::RefCallback, this);

      // Subscribers for PMPC objective reached and failure
      objective_reached_sub_ = nh.subscribe(
          ptr_config_->pmpc_obj_reached_topic_, 1,
          &HovergamesDroneInterface::PmpcObjectiveReachedCallback, this);
      pmpc_failure_sub_ =
          nh.subscribe(ptr_config_->pmpc_failure_topic_, 1,
                       &HovergamesDroneInterface::PmpcFailureCallback, this);

      // Set number of reference trajectory and obstacle avoidance constraints
      // stages to store
      n_stages_to_store_ = ratio_ + ptr_solver_->n_;

      // Initialize PMPC start message
      start_mpc_layer_msg_.header.frame_id = ptr_config_->robot_base_link_;
      start_mpc_layer_msg_.position.resize(3);
      start_mpc_layer_msg_.constraints.resize(
          ptr_config_->n_constraints_per_stage_ * 3);

      // Allocate memory for the reference trajectory in the real-time data
      ptr_module_handler_->data_.ref_traj_u_.resize(n_stages_to_store_);
      ptr_module_handler_->data_.ref_traj_x_.resize(n_stages_to_store_);
      for (int k = 0; k < n_stages_to_store_; k++) {
        ptr_module_handler_->data_.ref_traj_u_[k].resize(nu_);
        ptr_module_handler_->data_.ref_traj_x_[k].resize(nx_);
      }

      // Initialize the constraints and number of relevant constraints in the
      // real-time data Note: need to pass the reference constraints to the
      // module, so 2 times the horizon length
      ptr_module_handler_->data_.constraints_ = Eigen::MatrixXd::Zero(
          3 * ptr_config_->n_constraints_per_stage_, n_stages_to_store_);
      ptr_module_handler_->data_.n_relevant_constraints_ =
          Eigen::VectorXi::Zero(n_stages_to_store_);
    }
  } else if (ptr_config_->layer_idx_ == 1) {
    // Obtain relevant settings from solver
    use_tightened_obstacle_constraints_ = ptr_solver_->getConstantScalarBool(
        "use_tightened_obstacle_constraints");
    MPC_INFO_STREAM("use_tightened_obstacle_constraints_: "
                    << use_tightened_obstacle_constraints_);
    if (use_tightened_obstacle_constraints_) {
      c_o_ = ptr_solver_->getConstantScalarDouble("c_o");
      alpha_ = ptr_solver_->getConstantScalarDouble("alpha");
      MPC_INFO_STREAM("c_o_: " << c_o_);
      MPC_INFO_STREAM("alpha_: " << alpha_);

      // Get P_delta from Python, so it can be used for logging it in the static
      // data json recording
      eigenMatrixToStdVector(ptr_solver_->getConstantMatrixDouble("P_delta"),
                             P_delta_);
    }

    // Start subscriber for planner MPC layer if there are multiple MPC layers
    // to know when to start the control loop
    start_mpc_layer_sub_ =
        nh.subscribe(ptr_config_->start_mpc_layer_topic_, 1,
                     &HovergamesDroneInterface::StartMpcLayerCallback, this);

    // Subscriber for reference trajectory and constraints
    ref_pub_ = nh.advertise<mpc_msgs::ReferenceTrajectoryConstraints>(
        ptr_config_->ref_topic_, 1);

    // Publishers for objective reached and failure
    objective_reached_pub_ =
        nh.advertise<std_msgs::Empty>(ptr_config_->pmpc_obj_reached_topic_, 1);
    pmpc_failure_pub_ =
        nh.advertise<std_msgs::Empty>(ptr_config_->pmpc_failure_topic_, 1);

    // Initialize the variables to store and send the reference plan and
    // corresponding constraints NOTE: use n_stages_to_store_ =
    // n_pmpc_stages_to_send_ = n_states_to_constrain_+ptr_solver_->n_, since
    // TMPC tracks until steady-state in case of missing reference. Otherwise
    // min value of n_states_to_constrain_+1
    n_states_to_constrain_ =
        ptr_solver_->getConstantScalarDouble("n_states_to_constrain");
    MPC_INFO_STREAM("n_states_to_constrain_: " << n_states_to_constrain_);
    if (n_states_to_constrain_ >= 2) {
      n_stages_to_store_ = n_states_to_constrain_ + ptr_solver_->n_;
      n_pmpc_stages_to_send_ = n_states_to_constrain_ + ptr_solver_->n_;
    } else {
      MPC_ERROR_STREAM(
          "Hovergames drone interface initialization failed: "
          "n_states_to_constrain_ is"
          << n_states_to_constrain_ << ", but should be at least 2. Exiting");
      exit(1);
    }
    stored_inputs_ = Eigen::MatrixXd::Zero(nu_, n_stages_to_store_);
    stored_inputs_.row(thrust_c_idx_) =
        9.81 *
        Eigen::VectorXd::Ones(n_stages_to_store_);  // start with hover inputs

    stored_states_ =
        Eigen::MatrixXd::Zero(ptr_solver_->nx_, n_stages_to_store_);
    stored_states_.row(thrust_idx_ - nu_) =
        9.81 *
        Eigen::VectorXd::Ones(n_stages_to_store_);  // start with hover inputs

    stored_constraints_ = Eigen::MatrixXd::Zero(
        3 * ptr_config_->n_constraints_per_stage_, n_stages_to_store_);
    stored_n_relevant_constraints_ = Eigen::VectorXi::Zero(n_stages_to_store_);

    // Initialize reference plan message
    ref_msg_.header.frame_id = ptr_config_->robot_base_link_;
    ref_msg_.ref_traj.resize((nu_ + nx_) * n_pmpc_stages_to_send_);
    ref_msg_.ref_con.resize(3 * ptr_config_->n_constraints_per_stage_ *
                            n_pmpc_stages_to_send_);
    ref_msg_.n_relevant_con.resize(n_pmpc_stages_to_send_);

    // Initialize the constraints and number of relevant constraints in the
    // real-time data Note: need to store the generated constraints for
    // communication to TMPC (n_pmpc_stages_to_send_) and visualization
    // (n_states_to_constrain_+ptr_solver_->n_, of which ptr_solver_->n_ come
    // from the real-time data, the rest is stored)
    ptr_module_handler_->data_.constraints_ = Eigen::MatrixXd::Zero(
        3 * ptr_config_->n_constraints_per_stage_, ptr_solver_->n_);
    ptr_module_handler_->data_.n_relevant_constraints_ =
        Eigen::VectorXi::Zero(ptr_solver_->n_);
  } else {
    MPC_ERROR_STREAM(
        "Hovergames drone interface initialization failed: unknown MPC layer "
        "index: "
        << ptr_config_->layer_idx_ << ". Exiting");
    exit(1);
  }

  // ROS recording variables
  n_stages_ =
      n_states_to_constrain_ + ptr_solver_->n_;  // also used for visualization
  predicted_trajectory_.reserve(n_stages_);      // also used for visualization

  // Initialize ROS recording publishers and messages
  if (ptr_config_->record_pred_traj_) {
    pred_traj_rec_pub_ = nh.advertise<mpc_msgs::TrajectoryOverHorizon>(
        ptr_config_->pred_traj_rec_topic_, 1);
    pred_traj_msg_.header.frame_id = ptr_config_->robot_base_link_;
    pred_traj_msg_.u_names.assign(ptr_solver_->stage_vars_names_.begin(),
                                  ptr_solver_->stage_vars_names_.begin() + nu_);
    pred_traj_msg_.x_names.assign(
        ptr_solver_->stage_vars_names_.begin() + ptr_solver_->nu_,
        ptr_solver_->stage_vars_names_.end());
    pred_traj_msg_.traj.resize((nu_ + ptr_solver_->nx_) * n_stages_);
  }
  if (ptr_config_->record_slack_) {
    slack_rec_pub_ = nh.advertise<mpc_msgs::ScalarOverHorizon>(
        ptr_config_->slack_rec_topic_, 1);
    slack_msg_.header.frame_id = ptr_config_->robot_base_link_;
    slack_msg_.scalar.resize(
        n_stages_);  // TODO: this information is not stored if
                     // n_states_to_constrain > 1!
  }

  // ROS visuals variables
  n_regions_to_draw_ = ptr_config_->region_indices_to_draw_.size();
  region_poses_.reserve(n_regions_to_draw_);
  predicted_orientations_.reserve(n_stages_);
  predicted_region_poses_.reserve(n_stages_);
  for (size_t k = 0; k < predicted_region_poses_.size(); k++) {
    predicted_region_poses_[k].reserve(n_regions_to_draw_);
  }

  // ROS visuals predicted trajectory (and corresponding robot regions)
  // publisher
  predicted_positions_marker_pub_.reset(new ROSMarkerPublisher(
      nh_, ptr_config_->predicted_positions_vis_topic_.c_str(),
      ptr_config_->robot_target_frame_, 2 * n_stages_ - 1));
  predicted_positions_ground_marker_pub_.reset(new ROSMarkerPublisher(
      nh_, ptr_config_->predicted_positions_ground_vis_topic_.c_str(),
      ptr_config_->robot_target_frame_, 2 * n_stages_ - 1));
  predicted_regions_marker_pub_.reset(new ROSMarkerPublisher(
      nh_, ptr_config_->predicted_regions_vis_topic_.c_str(),
      ptr_config_->robot_target_frame_,
      ptr_config_->region_indices_to_draw_.size() * n_stages_));

  MPC_WARN_ALWAYS("Hovergames drone interface initialized");
}

void HovergamesDroneInterface::AtControlLoopStart() {
  PROFILE_AND_LOG_INFO("AtControlLoopStart");

  if (emergency_) return;

  // First publish current state, and create and publish ROS visuals valid for
  // this point in time
  if (ptr_config_->record_cur_state_ && ptr_config_->layer_idx_ == 0)
    PublishCurrentState();
  if (ptr_config_->layer_idx_ == 0) {
    CreateVisualizationsCurrentPosition();
  };
  MPC_INFO("Publishing visualizations");
  ptr_module_handler_->publishVisualizationsModules();
  PublishVisualizations();

  // Make sure to finish visualization before returning upon reaching the
  // objective
  if (objective_reached_) return;

  // If this is the PMPC layer, we care about fixed stages (trajectory and
  // constraints), so we need to do something with the stored vars
  if (ptr_config_->layer_idx_ > 0) {
    // To set the initial state, we take the first state that was free to be
    // optimised from the previous run Note: we don't care about the initial
    // input values
    ptr_solver_->last_updated_stage_vars_.tail(ptr_solver_->nx_) =
        stored_states_.col(n_states_to_constrain_);

    // We need to shift the stored reference trajectory and obstacle avoidance
    // constraints, such that the data corresponding to stage 0 of the previous
    // run, containing initial state, first optimized input and first
    // constructed obstacle avoidance constraint, (and possibly more shifted
    // stages) can be propagated to the current (and next) runs
    for (int k = 0; k < n_states_to_constrain_ - 1; k++) {
      stored_inputs_.col(k) = stored_inputs_.col(k + 1);
      stored_states_.col(k) = stored_states_.col(k + 1);
      stored_constraints_.col(k) = stored_constraints_.col(k + 1);
      stored_n_relevant_constraints_(k) = stored_n_relevant_constraints_(k + 1);
    }
  } else {
    // Forward-simulate the system for one timestep to start optimization
    // problem from there. Using previously computed (piecewise constant)
    // control input and last received state
    rk4_state_type_ x_cur(nx_);
    double t_cur = 0, t_final = integrator_steps_ * integrator_stepsize_;

    Eigen::Vector4d u_cur = ptr_solver_->stage_vars_.topLeftCorner<4, 1>();
    if (use_input_rates_) {
      u_cur.head(3) =
          u_cur.head(3) + ptr_solver_->stage_vars_.block<3, 1>(phi_c_idx_, 0);
    }
    rk4_sys_.u = std::vector<double>(
        u_cur.data(), u_cur.data() + u_cur.cols() * u_cur.rows());

    x_cur[0] = ptr_solver_->last_updated_stage_vars_(x_idx_);
    x_cur[1] = ptr_solver_->last_updated_stage_vars_(y_idx_);
    x_cur[2] = ptr_solver_->last_updated_stage_vars_(z_idx_);
    x_cur[3] = ptr_solver_->last_updated_stage_vars_(vx_idx_);
    x_cur[4] = ptr_solver_->last_updated_stage_vars_(vy_idx_);
    x_cur[5] = ptr_solver_->last_updated_stage_vars_(vz_idx_);
    x_cur[6] = ptr_solver_->last_updated_stage_vars_(phi_idx_);
    x_cur[7] = ptr_solver_->last_updated_stage_vars_(theta_idx_);
    x_cur[8] = ptr_solver_->last_updated_stage_vars_(psi_idx_);
    x_cur[9] = ptr_solver_->last_updated_stage_vars_(thrust_idx_);

    rk4_integrator_.integrate_const(rk4_sys_, x_cur, t_cur, t_final,
                                    integrator_stepsize_);

    ptr_solver_->last_updated_stage_vars_(x_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][0];
    ptr_solver_->last_updated_stage_vars_(y_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][1];
    ptr_solver_->last_updated_stage_vars_(z_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][2];
    ptr_solver_->last_updated_stage_vars_(vx_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][3];
    ptr_solver_->last_updated_stage_vars_(vy_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][4];
    ptr_solver_->last_updated_stage_vars_(vz_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][5];
    ptr_solver_->last_updated_stage_vars_(phi_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][6];
    ptr_solver_->last_updated_stage_vars_(theta_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][7];
    ptr_solver_->last_updated_stage_vars_(psi_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][8];
    ptr_solver_->last_updated_stage_vars_(thrust_idx_) =
        rk4_integrator_.observer_.x[integrator_steps_][9];
  }
  // Store the input in last_updated_stage_vars_ (last_updated_stage_vars_
  // overwrites [u0;x0])
  if (use_input_rates_) {
    ptr_solver_->last_updated_stage_vars_(phi_c_idx_) =
        ptr_solver_->stage_vars_(phi_c_idx_, 1);
    ptr_solver_->last_updated_stage_vars_(theta_c_idx_) =
        ptr_solver_->stage_vars_(theta_c_idx_, 1);
    ptr_solver_->last_updated_stage_vars_(psi_c_idx_) =
        ptr_solver_->stage_vars_(psi_c_idx_, 1);
  }
  ptr_solver_->last_updated_stage_vars_.segment<4>(0) =
      ptr_solver_->stage_vars_.block<4, 1>(0, 1);

  // Handle timing of PMPC and set reference trajectory and constraints
  // accordingly in TMPC
  if (ptr_config_->n_layers_ > 1 && ptr_config_->layer_idx_ == 0) {
    if (ptr_controller_->getCountTotal() % ratio_ == ratio_ - 1) {
      if (first_ref_received_) {
        CheckRef();
        if (schedule_planner_)
          StartMpcLayerPublish();  // only schedule the planner in a flow of
                                   // receiving references, not when finishing
                                   // the reference after a reference failure
        SetReferenceTrajectory();
        SetConstraints();
      } else {
        StartMpcLayerPublish();  // always schedule the planner if we didn't
                                 // receive a reference yet
        SetReferenceTrajectoryHover();
        SetConstraintsHover();
      }
    }
  }
}

void HovergamesDroneInterface::AfterUpdateModules() {
  PROFILE_AND_LOG_INFO("AfterUpdateModules");

  if (use_tightened_obstacle_constraints_)
    ptr_module_handler_->tightenConstraints(
        std::string("StaticPolyhedronConstraints"), c_o_, alpha_);

  // We need to store the obstacle avoidance constraints of the current run from
  // stage 1 onwards NOTE: +1 is used to let the indices start at 0 (containing
  // the data of stage 1) NOTE: duplicate last constraints for last stage, since
  // constraint of stage 1 is applied to equivalent of stage 0 in TMPC. Note
  // that this also assumes that line_segment_to_stage is set to 1 and that each
  // line segment is completely contained within the corresponding constraint
  // region
  if (ptr_config_->layer_idx_ == 1) {
    for (int k = n_states_to_constrain_ - 1; k < n_stages_to_store_ - 1; k++) {
      stored_constraints_.col(k) = ptr_module_handler_->data_.constraints_.col(
          k - n_states_to_constrain_ + 1);
      stored_n_relevant_constraints_(k) =
          ptr_module_handler_->data_.n_relevant_constraints_(
              k - n_states_to_constrain_ + 1);
    }
    // Duplicate last constraints
    for (int k = n_stages_to_store_ - 1; k < n_stages_to_store_; k++) {
      stored_constraints_.col(k) = ptr_module_handler_->data_.constraints_.col(
          k - n_states_to_constrain_);
      stored_n_relevant_constraints_(k) =
          ptr_module_handler_->data_.n_relevant_constraints_(
              k - n_states_to_constrain_);
    }
  }
}

void HovergamesDroneInterface::AfterSetParametersModules() {
  // In PMPC set the constraints for the complete horizon and tighten
  if (ptr_config_->n_layers_ > 1 && ptr_config_->layer_idx_ == 1) {
    PROFILE_AND_LOG_INFO("AfterSetParametersModules");

    Eigen::MatrixXd constraints_to_rec_and_vis = stored_constraints_;
    if (use_tightened_obstacle_constraints_) {
      for (Eigen::Index k = 0; k < stored_constraints_.cols(); k++) {
        for (Eigen::Index l = 0; l < ptr_config_->n_constraints_per_stage_;
             l++) {
          constraints_to_rec_and_vis(3 * l + 2, k) =
              constraints_to_rec_and_vis(3 * l + 2, k) - c_o_ * alpha_;
        }
      }
    }

    ptr_module_handler_->setConstraintsToRecAndVis(
        "StaticPolyhedronConstraints", constraints_to_rec_and_vis,
        stored_n_relevant_constraints_);
  }
}

void HovergamesDroneInterface::AfterOptimization() {
  PROFILE_AND_LOG_INFO("AfterOptimization");

  // We need to store the optimized inputs and states of the current run from
  // stage 0 onwards Note +1 is used to let the indices start at 0 (containing
  // the data of stage 0)
  if (ptr_config_->layer_idx_ == 1) {
    for (int k = n_states_to_constrain_ - 1; k < n_stages_to_store_; k++) {
      stored_inputs_.col(k) = ptr_solver_->stage_vars_.block(
          0, k - n_states_to_constrain_ + 1, nu_, 1);
      stored_states_.col(k) = ptr_solver_->stage_vars_.block(
          ptr_solver_->nu_, k - n_states_to_constrain_ + 1, ptr_solver_->nx_,
          1);
    }
  }

  // Store predicted positions
  for (int k = 0; k < n_stages_; k++) {
    Eigen::VectorXd trajectory_point(nu_ + ptr_solver_->nx_);
    if (n_states_to_constrain_ == 1) {
      trajectory_point << ptr_solver_->stage_vars_.block(0, k, nu_, 1),
          ptr_solver_->stage_vars_.block(x_idx_, k, ptr_solver_->nx_, 1);
      predicted_trajectory_.emplace_back(std::move(trajectory_point));
    } else if (n_states_to_constrain_ > 1) {
      trajectory_point << stored_inputs_.block(0, k, nu_, 1),
          stored_states_.block(0, k, ptr_solver_->nx_, 1);
      predicted_trajectory_.emplace_back(std::move(trajectory_point));
    }
  }

  // Publish relevant solver output
  if (ptr_config_->record_pred_traj_) PublishPredictedTrajectory();
  if (ptr_config_->record_slack_) PublishSlack();
}

void HovergamesDroneInterface::ComputeActuation() {
  PROFILE_AND_LOG_INFO("ComputeActuation");

  // Set emergency to false again, since this function only gets called after
  // successful optimization
  emergency_ = false;

  if (ptr_config_->layer_idx_ == 0) {
    u_solver_ = ptr_solver_->stage_vars_.topLeftCorner<4, 1>();
    if (use_input_rates_) {
      u_solver_[0] = u_solver_[0] + ptr_solver_->stage_vars_(phi_c_idx_, 0);
      u_solver_[1] = u_solver_[1] + ptr_solver_->stage_vars_(theta_c_idx_, 0);
      u_solver_[2] = u_solver_[2] + ptr_solver_->stage_vars_(psi_c_idx_, 0);
    }

    OnComputeActuation();
  } else if (ptr_config_->layer_idx_ == 1) {
    // Fill and publish reference trajectory and obstacle avoidance constraints
    // message Note: we only need to send the first n_pmpc_stages_to_send_ PMPC
    // stages
    ref_msg_.header.stamp = ros::Time::now();
    ref_msg_.count = tmpc_count_;

    ref_msg_.ref_traj.clear();
    Eigen::MatrixXd u_ref =
        stored_inputs_.block(0, 0, nu_, n_pmpc_stages_to_send_);
    Eigen::MatrixXd x_ref =
        stored_states_.block(0, 0, nx_, n_pmpc_stages_to_send_);
    if (use_input_rates_)
      u_ref.topRows(3) =
          u_ref.topRows(3) + stored_states_.block(phi_c_idx_ - ptr_solver_->nu_,
                                                  0, 3, n_pmpc_stages_to_send_);
    for (int k = 0; k < n_pmpc_stages_to_send_; k++) {
      for (int l = 0; l < nu_; l++) ref_msg_.ref_traj.emplace_back(u_ref(l, k));

      for (int l = 0; l < nx_; l++) ref_msg_.ref_traj.emplace_back(x_ref(l, k));
    }

    ref_msg_.ref_con.clear();
    ref_msg_.n_relevant_con.clear();
    // Print all stored inputs, states and constraints
    for (int k = 0; k < n_pmpc_stages_to_send_; k++) {
      for (int l = 0; l < nu_; l++)
        MPC_INFO("PMPC u_ref[" << k << "][" << l << "]: " << u_ref(l, k));
      for (int l = 0; l < nx_; l++)
        MPC_INFO("PMPC stored_states_[" << k << "][" << l
                                        << "]: " << stored_states_(l, k));
      for (int l = 0; l < ptr_config_->n_constraints_per_stage_; l++) {
        for (int m = 0; m < 3; m++)
          ref_msg_.ref_con.emplace_back(stored_constraints_(3 * l + m, k));
        MPC_INFO("PMPC stored_constraints_["
                 << k << "][" << l << "]: " << stored_constraints_(3 * l, k)
                 << ", " << stored_constraints_(3 * l + 1, k) << ", "
                 << stored_constraints_(3 * l + 2, k));
      }
      ref_msg_.n_relevant_con.emplace_back(stored_n_relevant_constraints_(k));
      MPC_INFO("PMPC stored_n_relevant_constraints_["
               << k << "]: " << stored_n_relevant_constraints_(k));
    }

    ref_pub_.publish(ref_msg_);
  }
}

void HovergamesDroneInterface::Actuate() {
  PROFILE_AND_LOG_INFO("Actuate");

  // Only continue when the layer is zero
  if (ptr_config_->layer_idx_ > 0) return;

  // In case of emergency: brake
  if (objective_reached_ || emergency_) {
    ActuateBrake();
    return;
  }

  // Actuate
  OnActuate();
}

void HovergamesDroneInterface::DeployObjectiveReachedStrategy() {
  PROFILE_AND_LOG_INFO("DeployObjectiveReachedStrategy");

  if (ptr_config_->layer_idx_ == 0) {
    ptr_config_->plan_ = false;
    objective_reached_ = true;

    OnDeployObjectiveReachedStrategy();
    ActuateBrake();

    ptr_controller_->OnObjectiveReached();
  } else if (ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1) {
    std_msgs::Empty msg;
    objective_reached_pub_.publish(msg);
  }
}

void HovergamesDroneInterface::DeployEmergencyStrategy() {
  PROFILE_AND_LOG_WARN("DeployEmergencyStrategy");

  // Note: no need to disable the PMPC, since it is called by the TMPC and run
  // once
  if (ptr_config_->layer_idx_ == 0) {
    if (ptr_controller_->getCountSinceReset() > 0) {
      emergency_ = true;
      MPC_WARN("Emergency");
    } else {
      MPC_WARN(
          "Count since reset = 0, so deploying emergency strategy once before "
          "going into emergency mode");
    }

    OnDeployEmergencyStrategy();
    ActuateBrake();
  } else if (ptr_config_->n_layers_ > 1 && ptr_config_->layer_idx_ == 1) {
    if (ptr_controller_->getCountSinceReset() > 0) {
      std_msgs::Empty msg;
      pmpc_failure_pub_.publish(msg);
      MPC_WARN("Send failure to TMPC");
    } else {
      MPC_WARN("Count since reset = 0, so do nothing yet");
    }
  }
}

void HovergamesDroneInterface::ActuateBrake() {
  PROFILE_AND_LOG_INFO("ActuateBrake");

  OnActuateBrake();
}

void HovergamesDroneInterface::Reset() {
  PROFILE_AND_LOG_WARN(
      "Reset - not implemented for hovergames drone interface!");
}

// Callback methods
void HovergamesDroneInterface::StateCallback(const nav_msgs::Odometry &msg) {
  PROFILE_AND_LOG_INFO("StateCallback");

  // Should be just storing state here!!
  last_state_msg_ = msg;

  // Calculate Euler angles from received quaternion
  tf2::Quaternion q_tf;
  tf2::convert(msg.pose.pose.orientation, q_tf);
  tf2Scalar roll, pitch, yaw;
  tf2::Matrix3x3(q_tf).getEulerYPR(yaw, pitch, roll);

  // Update the frame
  ptr_config_->robot_target_frame_ = msg.header.frame_id;

  // Update the states
  // Note: PX4 interface overwrites the velocities as need to be converted from
  // body to world frame
  ptr_solver_->last_updated_stage_vars_(x_idx_) = msg.pose.pose.position.x;
  ptr_solver_->last_updated_stage_vars_(y_idx_) = msg.pose.pose.position.y;
  ptr_solver_->last_updated_stage_vars_(z_idx_) = msg.pose.pose.position.z;
  ptr_solver_->last_updated_stage_vars_(vx_idx_) = msg.twist.twist.linear.x;
  ptr_solver_->last_updated_stage_vars_(vy_idx_) = msg.twist.twist.linear.y;
  ptr_solver_->last_updated_stage_vars_(vz_idx_) = msg.twist.twist.linear.z;
  ptr_solver_->last_updated_stage_vars_(phi_idx_) = roll;
  ptr_solver_->last_updated_stage_vars_(theta_idx_) = pitch;
  ptr_solver_->last_updated_stage_vars_(psi_idx_) = yaw;

  ptr_controller_->OnStateReceived();

  OnStateCallback(msg);
}

// Recording publishing methods
void HovergamesDroneInterface::PublishCurrentState() {
  PROFILE_AND_LOG_INFO("PublishCurrentState");

  cur_state_msg_.header.stamp = ros::Time::now();
  ptr_controller_->setMpcHeader(cur_state_msg_.mpc_header);
  std::copy(ptr_solver_->last_updated_stage_vars_.data() + ptr_solver_->nu_,
            ptr_solver_->last_updated_stage_vars_.data() +
                ptr_solver_->last_updated_stage_vars_.size(),
            cur_state_msg_.array.begin());
  cur_state_rec_pub_.publish(cur_state_msg_);
}

void HovergamesDroneInterface::PublishPredictedTrajectory() {
  PROFILE_AND_LOG_INFO("PublishPredictedTrajectory");

  pred_traj_msg_.header.stamp = ros::Time::now();
  ptr_controller_->setMpcHeader(pred_traj_msg_.mpc_header);
  for (int k = 0; k < n_stages_; k++)
    std::copy(
        predicted_trajectory_[k].data(),
        predicted_trajectory_[k].data() + predicted_trajectory_[k].size(),
        pred_traj_msg_.traj.begin() + k * predicted_trajectory_[k].size());
  pred_traj_rec_pub_.publish(pred_traj_msg_);
}

void HovergamesDroneInterface::PublishSlack() {
  PROFILE_AND_LOG_INFO("PublishSlack");

  slack_msg_.header.stamp = ros::Time::now();
  ptr_controller_->setMpcHeader(slack_msg_.mpc_header);
  for (int k = 0; k < n_stages_; k++) {
    if (use_slack_)
      slack_msg_.scalar[k] = ptr_solver_->stage_vars_(slack_idx_, k);
    else
      slack_msg_.scalar[k] = 0;
  }
  slack_rec_pub_.publish(slack_msg_);
}

// HMPC-specific methods
void HovergamesDroneInterface::StartMpcLayerPublish() {
  PROFILE_AND_LOG_INFO("StartMpcLayerPublish");

  // Fill message fields
  start_mpc_layer_msg_.header.stamp = ros::Time::now();
  start_mpc_layer_msg_.count = ptr_controller_->getCountTotal();
  start_mpc_layer_msg_.reset = reset_planner_;

  // Since only used to start the PMPC from hover: communicate current position
  // and obstacle avoidance constraints to PMPC
  for (int k = 0; k < 3; k++) {
    start_mpc_layer_msg_.position[k] =
        ptr_solver_->last_updated_stage_vars_(ptr_solver_->nu_ + k);
  }

  for (int l = 0; l < ptr_config_->n_constraints_per_stage_; l++) {
    for (int m = 0; m < 3; m++)
      start_mpc_layer_msg_.constraints[l * 3 + m] =
          ptr_module_handler_->data_.constraints_(3 * l + m, 0);
  }
  start_mpc_layer_msg_.n_relevant_constraints =
      ptr_module_handler_->data_.n_relevant_constraints_(0);

  // Publish
  start_mpc_layer_pub_.publish(start_mpc_layer_msg_);

  MPC_INFO("Published start MPC layer message with count: "
           << start_mpc_layer_msg_.count
           << " and reset: " << (int)start_mpc_layer_msg_.reset);
}

void HovergamesDroneInterface::StartMpcLayerCallback(
    const mpc_msgs::StartMpcLayer &msg) {
  PROFILE_AND_LOG_INFO("StartMpcLayerCallback");

  MPC_INFO("Received start MPC layer message with count: "
           << msg.count << " and reset: " << (int)msg.reset);

  tmpc_count_ = msg.count;

  if (msg.reset) {
    // In first PMPC run, make sure to start from the current position and
    // obstacle avoidance constraints
    for (int k = 0; k < ptr_solver_->n_ + 1; k++) {
      // States
      stored_states_(x_idx_ - ptr_solver_->nu_, k) = msg.position[0];
      stored_states_(y_idx_ - ptr_solver_->nu_, k) = msg.position[1];
      stored_states_(z_idx_ - ptr_solver_->nu_, k) = msg.position[2];

      // Obstacle avoidance constraints
      for (int l = 0; l < ptr_config_->n_constraints_per_stage_; l++) {
        for (int m = 0; m < 3; m++)
          stored_constraints_(3 * l + m, k) = msg.constraints[l * 3 + m];
      }
      stored_n_relevant_constraints_(k) = msg.n_relevant_constraints;
    }
  }

  // We are going to run the controller for one loop
  ptr_controller_->controlLoop();
}

void HovergamesDroneInterface::RefCallback(
    const mpc_msgs::ReferenceTrajectoryConstraints &msg) {
  PROFILE_AND_LOG_INFO("RefCallback");

  ref_msg_ = msg;

  // Only accept reference when it is received in time
  if (msg.count + ratio_ > ptr_controller_->getCountTotal()) {
    if (!first_ref_received_) {
      first_ref_received_ = true;
      n_pmpc_stages_to_receive_ = ref_msg_.ref_traj.size() / (nu_ + nx_);
    }

    if (reset_planner_) reset_planner_ = false;

    updated_ref_received_ = true;
    ref_stage_idx_start_ = 0;
  } else {
    MPC_WARN("Reference received too late, ref count ("
             << msg.count << ") + ratio_ (" << ratio_ << ") <= current count ("
             << ptr_controller_->getCountTotal() << ")");
  }
}

void HovergamesDroneInterface::SetReferenceTrajectoryHover() {
  PROFILE_AND_LOG_INFO("SetReferenceTrajectoryHover");

  // Insert current position with hovering input
  for (int k = 0; k < n_stages_to_store_; k++) {
    Eigen::Vector4d u_init = {0.0, 0.0, 0.0, 9.81};
    for (int l = 0; l < nu_; l++) {
      ptr_module_handler_->data_.ref_traj_u_[k][l] = u_init(l);
    }

    // We only want to track the position with zero velocities and zero angles
    // as start
    for (int l = 0; l < 3; l++) {
      ptr_module_handler_->data_.ref_traj_x_[k][l] =
          start_stage_vars_(ptr_solver_->nu_ + l);
    }

    for (int l = 3; l < nx_; l++) {
      ptr_module_handler_->data_.ref_traj_x_[k][l] = 0;
    }
    // We need stability in the thrust
    ptr_module_handler_->data_.ref_traj_x_[k][thrust_idx_ - ptr_solver_->nu_] =
        9.81;
  }

  // This data is valid for all prediction horizons during the next PMPC
  // interval
  ptr_module_handler_->data_.ref_traj_n_it_valid_ = ratio_;

  ptr_module_handler_->onDataReceivedObjective("ReferenceTrajectory");
}

void HovergamesDroneInterface::SetConstraintsHover() {
  PROFILE_AND_LOG_INFO("SetConstraintsHover");

  // Insert a square bounding box with pre-defined width around the start
  // position, and other dummy constraints
  for (int k = 0; k < n_stages_to_store_; k++) {
    // Square bounding box around start position
    ptr_module_handler_->data_.constraints_.block<3, 1>(0, k) = Eigen::Vector3d(
        1, 0, start_stage_vars_(x_idx_) + ptr_config_->bounding_box_width_ / 2);
    ptr_module_handler_->data_.constraints_.block<3, 1>(3, k) = Eigen::Vector3d(
        -1, 0,
        -(start_stage_vars_(x_idx_) - ptr_config_->bounding_box_width_ / 2));
    ptr_module_handler_->data_.constraints_.block<3, 1>(6, k) = Eigen::Vector3d(
        0, 1, start_stage_vars_(y_idx_) + ptr_config_->bounding_box_width_ / 2);
    ptr_module_handler_->data_.constraints_.block<3, 1>(9, k) = Eigen::Vector3d(
        0, -1,
        -(start_stage_vars_(y_idx_) - ptr_config_->bounding_box_width_ / 2));

    // Dummy constraints
    for (int l = 4; l < ptr_config_->n_constraints_per_stage_; l++) {
      ptr_module_handler_->data_.constraints_.block<3, 1>(3 * l, k) =
          Eigen::Vector3d(1, 0, 1000);
    }
    ptr_module_handler_->data_.n_relevant_constraints_(k) = 4;

    // This data is valid for all prediction horizons during the next PMPC
    // interval Note: at t = 0 the constraints are set, but do not have to be
    // set for the complete horizon (only for N-1 runs). This is no problem,
    // since constraints_n_it_valid_ gets overwritten every time new constraints
    // become available
    ptr_module_handler_->data_.constraints_n_it_valid_ = ratio_;
  }
  ptr_module_handler_->onDataReceivedConstraint("Constraints");
}

void HovergamesDroneInterface::SetReferenceTrajectory() {
  PROFILE_AND_LOG_INFO("SetReferenceTrajectory");

  /* Fill real-time data with reference trajectory valid for TMPC by
   * forward-simulation the received reference plan based on the sampling
   * difference between PMPC and TMPC */
  // NOTE: similar to functionality in SetConstraints(), the reference
  // trajectory set in this function is valid for the next n_stages_to_store_
  // TMPC stages, which is until the next received/shifted reference should be
  // processed
  rk4_state_type_ x_cur(nx_);
  const int integrator_steps_ = ptr_solver_->dt_ / integrator_stepsize_;
  double t_cur = 0;

  // NOTE: Since n_stages_to_store_ consists of the first ratio_ steps appended
  // by the remaining TMPC prediction horizon, first forward-simulate the first
  // ratio_ steps, then the remaining TMPC prediction horizon Determine number
  // of piecewise constant reference inputs to use for forward-simulation
  int n_ref_u = std::ceil((double)n_stages_to_store_ / ratio_);

  // Determine number of remaining steps to forward-simulate after last
  // reference stage
  int n_rem_steps = n_stages_to_store_ % ratio_;

  // Compute reference trajectory based on piecewise constant reference input
  // plan If next ratio_ steps fit in n_stages_to_store_: compute reference
  // trajectory for interval [i*ratio_, (i+1)*ratio_) based on reference segment
  // i If next ratio_ steps do not fit in n_stages_to_store_: compute reference
  // trajectory for interval [i*ratio_, i*ratio_+n_rem_steps) based on reference
  // segment i Clip result to last received steady-state reference stage
  int ref_stage_idx = ref_stage_idx_start_;  // initialize reference stage index
                                             // from correct starting index
  int n_steps = ratio_;  // initialize number of steps to forward-simulate
  double t_final =
      n_steps * integrator_steps_ *
      integrator_stepsize_;  // initialize total time to forward-simulate
  for (int i = 0; i < n_ref_u; i++) {
    // Determine length of current reference trajectory segment to fill
    if (i < n_ref_u - 1)
      n_steps = ratio_;
    else if (i == n_ref_u - 1 && n_rem_steps == 0)
      n_steps = ratio_;
    else if (i == n_ref_u - 1 && n_rem_steps > 0)
      n_steps = n_rem_steps;
    else
      break;

    // Only forward-simulate if last reference stage is not reached yet,
    // otherwise insert steady-state input-state pair
    if (ref_stage_idx < n_pmpc_stages_to_receive_ - 1) {
      // Forward-simulate model over next n_steps given reference state and
      // input
      t_final = n_steps * integrator_steps_ * integrator_stepsize_;
      for (int l = 0; l < nu_; l++)
        rk4_sys_.u[l] = ref_msg_.ref_traj[(nu_ + nx_) * ref_stage_idx + l];
      for (int l = 0; l < nx_; l++)
        x_cur[l] = ref_msg_.ref_traj[(nu_ + nx_) * ref_stage_idx + nu_ + l];
      rk4_integrator_.integrate_const(rk4_sys_, x_cur, t_cur, t_final,
                                      integrator_stepsize_);

      // Store in real-time data: [i*ratio_, i*ratio_+n_steps)
      for (int k = 0; k < n_steps; k++) {
        for (int l = 0; l < nu_; l++)
          ptr_module_handler_->data_.ref_traj_u_[i * ratio_ + k][l] =
              ref_msg_.ref_traj[(nu_ + nx_) * ref_stage_idx + l];
        for (int l = 0; l < nx_; l++)
          ptr_module_handler_->data_.ref_traj_x_[i * ratio_ + k][l] =
              rk4_integrator_.observer_.x[k * integrator_steps_][l];
      }

      // Increase reference stage index by one
      ref_stage_idx++;
    } else {
      for (int k = 0; k < n_steps; k++) {
        for (int l = 0; l < nu_; l++)
          ptr_module_handler_->data_.ref_traj_u_[i * ratio_ + k][l] =
              u_steady_state_(l);
        for (int l = 0; l < nx_; l++)
          ptr_module_handler_->data_.ref_traj_x_[i * ratio_ + k][l] =
              ref_msg_.ref_traj[(nu_ + nx_) * (n_pmpc_stages_to_receive_ - 1) +
                                nu_ + l];
      }
    }
  }

  // Print reference trajectory
  for (int k = 0; k < n_stages_to_store_; k++) {
    for (int l = 0; l < nu_; l++)
      MPC_INFO("TMPC ref_traj_u_["
               << k << "][" << l
               << "]: " << ptr_module_handler_->data_.ref_traj_u_[k][l]);
    for (int l = 0; l < nx_; l++)
      MPC_INFO("TMPC ref_traj_x_["
               << k << "][" << l
               << "]: " << ptr_module_handler_->data_.ref_traj_x_[k][l]);
  }

  // This data is valid for all prediction horizons during the next PMPC
  // interval
  ptr_module_handler_->data_.ref_traj_n_it_valid_ = ratio_;

  /* Call OnReferenceTrajectoryReceived to update reference trajectory module
   * for cost function calculation */
  ptr_module_handler_->onDataReceivedObjective("ReferenceTrajectory");
}

void HovergamesDroneInterface::SetConstraints() {
  PROFILE_AND_LOG_INFO("SetConstraints");

  // Fill real-time data with TMPC constraints, based on the received PMPC
  // constraints NOTE: similar to functionality in SetReferenceTrajectory(), the
  // constraints set in this function are valid for the next n_stages_to_store_
  // TMPC stages, which is until the next received/shifted reference should be
  // processed

  // Derive constraint sets for interval based on piecewise constant reference
  // constraint sets Obtain initial constraint set at 0 based on reference
  // segment 0 If next ratio_ steps fit in n_stages_to_store_: obtain constraint
  // set for interval [i*ratio_, (i+1)*ratio_) based on reference segment i If
  // next ratio_ steps do not fit in n_stages_to_store_: compute [i*ratio_,
  // i*ratio_+n_rem_steps) based on reference segment i Clip result to last
  // received reference stage constraint set
  int constraint_idx;
  for (int k = 0; k < n_stages_to_store_; k++) {
    k = std::max(k, 0);  // make sure k is not negative to handle the case k = 0
    constraint_idx = std::min(ref_stage_idx_start_ + (k - 1) / ratio_,
                              n_pmpc_stages_to_receive_ - 1);
    for (int l = 0; l < ptr_config_->n_constraints_per_stage_; l++) {
      for (int m = 0; m < 3; m++)
        ptr_module_handler_->data_.constraints_(3 * l + m, k) =
            ref_msg_.ref_con[constraint_idx *
                                 ptr_config_->n_constraints_per_stage_ * 3 +
                             l * 3 + m];
    }
    ptr_module_handler_->data_.n_relevant_constraints_(k) =
        ref_msg_.n_relevant_con[constraint_idx];
  }

  // Print constraints set
  for (int k = 0; k < n_stages_to_store_; k++) {
    for (int l = 0; l < ptr_config_->n_constraints_per_stage_; l++) {
      MPC_INFO("TMPC constraints_["
               << k << "][" << l << "]: "
               << ptr_module_handler_->data_.constraints_(3 * l, k) << ", "
               << ptr_module_handler_->data_.constraints_(3 * l + 1, k) << ", "
               << ptr_module_handler_->data_.constraints_(3 * l + 2, k));
    }
    MPC_INFO("TMPC n_relevant_constraints_["
             << k
             << "]: " << ptr_module_handler_->data_.n_relevant_constraints_(k));
  }

  // This data is valid for all prediction horizons during the next PMPC
  // interval
  ptr_module_handler_->data_.constraints_n_it_valid_ = ratio_;

  /* Call OnConstraintsReceived to update polyhedron constraints module for
   * polyhedron constraints insertion */
  ptr_module_handler_->onDataReceivedConstraint("Constraints");
}

void HovergamesDroneInterface::PmpcObjectiveReachedCallback(
    const std_msgs::Empty &msg) {
  PROFILE_AND_LOG_WARN("PmpcObjectiveReachedCallback");

  DeployObjectiveReachedStrategy();
}

void HovergamesDroneInterface::PmpcFailureCallback(const std_msgs::Empty &msg) {
  PROFILE_AND_LOG_WARN("PmpcFailureCallback");

  updated_ref_received_ = false;
}

void HovergamesDroneInterface::CheckRef() {
  PROFILE_AND_LOG_INFO("CheckRef");

  if (!first_ref_received_) {
    MPC_WARN("No reference received yet. Continuing");
    return;
  } else if (!updated_ref_received_) {
    ref_stage_idx_start_ += 1;
    if (ref_stage_idx_start_ >= n_pmpc_stages_to_receive_ - 1) {
      MPC_WARN(
          "Reached steady state. Trying to schedule PMPC again with current "
          "position and constraints");
      schedule_planner_ = true;
      reset_planner_ = true;
    } else {
      MPC_WARN(
          "No updated reference received. Continue to track last reference "
          "shifted by "
          << ref_stage_idx_start_ << " stage(s)");
      schedule_planner_ = false;
    }
  } else {
    ref_stage_idx_start_ = 0;
  }

  updated_ref_received_ = false;
}

// ROS visualization methods
void HovergamesDroneInterface::CreateVisualizations() {
  PROFILE_AND_LOG_INFO("CreateVisualizations");

  // Only create visualization for predicted trajectory here, the current
  // position needs to be visualized differently to allow for correct
  // information
  CreateVisualizationsPredictedPositions();
}

void HovergamesDroneInterface::PublishVisualizations() {
  PROFILE_AND_LOG_INFO("PublishVisualizations");

  if (ptr_config_->layer_idx_ == 0) {
    PublishVisualizationsCurrentPosition();
  };
  PublishVisualizationsPredictedTrajectory();
}

void HovergamesDroneInterface::CreateVisualizationsCurrentPosition() {
  PROFILE_AND_LOG_INFO("CreateVisualizationsCurrentPosition");

  if (!(ptr_config_->draw_current_position_ground_air_ > 0 ||
        ptr_config_->draw_current_region_))
    return;

  // Obtain current position
  Eigen::Vector3d pose = {ptr_solver_->last_updated_stage_vars_(x_idx_),
                          ptr_solver_->last_updated_stage_vars_(y_idx_),
                          ptr_solver_->last_updated_stage_vars_(z_idx_)};

  // Create current position marker if desired
  if (ptr_config_->draw_current_position_ground_air_ >= 2) {
    ROSPointMarker &current_position =
        current_position_marker_pub_->getNewPointMarker("CYLINDER");
    current_position.setColor(ptr_config_->current_position_marker_color_[0],
                              ptr_config_->current_position_marker_color_[1],
                              ptr_config_->current_position_marker_color_[2],
                              ptr_config_->current_position_marker_color_[3]);
    current_position.setScale(ptr_config_->current_position_marker_scale_[0],
                              ptr_config_->current_position_marker_scale_[1],
                              ptr_config_->current_position_marker_scale_[2]);
    current_position.addPointMarker(Eigen::Vector3d(pose(0), pose(1), pose(2)));
  }

  // Create current position ground projection marker if desired
  if (ptr_config_->draw_current_position_ground_air_ == 1 ||
      ptr_config_->draw_current_position_ground_air_ >= 3) {
    ROSPointMarker &current_position_ground =
        current_position_ground_marker_pub_->getNewPointMarker("CYLINDER");
    current_position_ground.setColor(
        ptr_config_->current_position_marker_color_[0],
        ptr_config_->current_position_marker_color_[1],
        ptr_config_->current_position_marker_color_[2],
        ptr_config_->ground_projection_marker_alpha_);
    current_position_ground.setScale(
        ptr_config_->current_position_marker_scale_[0],
        ptr_config_->current_position_marker_scale_[1],
        ptr_config_->current_position_marker_scale_[2]);
    current_position_ground.addPointMarker(Eigen::Vector3d(
        pose(0), pose(1), ptr_config_->ground_projection_marker_z_));
  }

  // Create current region marker if desired
  if (ptr_config_->draw_current_region_) {
    // Obtain current orientation
    double yaw = ptr_solver_->last_updated_stage_vars_(psi_idx_);

    // Store region poses
    for (const int &r_idx : ptr_config_->region_indices_to_draw_) {
      region_poses_.emplace_back(Eigen::Vector2d(
          ptr_robot_region_->discs_[r_idx].DiscX(pose(0), yaw),
          ptr_robot_region_->discs_[r_idx].DiscY(pose(1), yaw)));
    }

    // Draw all region poses as a cylinder
    if (ptr_config_->current_region_n_points_ == 1) {
      ROSPointMarker &current_region =
          current_region_marker_pub_->getNewPointMarker("CYLINDER");
      current_region.setColor(ptr_config_->current_position_marker_color_[0],
                              ptr_config_->current_position_marker_color_[1],
                              ptr_config_->current_position_marker_color_[2],
                              ptr_config_->current_position_marker_color_[3]);
      current_region.setScale(2 * ptr_robot_region_->DiscRadius(),
                              2 * ptr_robot_region_->DiscRadius(),
                              ptr_config_->current_region_marker_scale_[2]);

      for (size_t region_idx = 0; region_idx < n_regions_to_draw_;
           region_idx++) {
        current_region.addPointMarker(Eigen::Vector3d(
            region_poses_[region_idx](0), region_poses_[region_idx](1),
            ptr_config_->current_region_marker_z_));
      }
    }
    // Draw all region poses as circular lines
    else if (ptr_config_->current_region_n_points_ >= 10) {
      ROSLine &current_region = current_region_marker_pub_->getNewLine();
      current_region.setColor(ptr_config_->current_position_marker_color_[0],
                              ptr_config_->current_position_marker_color_[1],
                              ptr_config_->current_position_marker_color_[2],
                              ptr_config_->current_position_marker_color_[3]);
      current_region.setScale(ptr_config_->current_region_marker_scale_[0]);

      for (size_t region_idx = 0; region_idx < n_regions_to_draw_;
           region_idx++) {
        current_region.addCircularLine(region_poses_[region_idx],
                                       ptr_robot_region_->DiscRadius(),
                                       ptr_config_->current_region_marker_z_,
                                       ptr_config_->current_region_n_points_);
      }
    }

    // Clear region poses vector
    region_poses_.clear();
  }
}

void HovergamesDroneInterface::CreateVisualizationsPredictedPositions() {
  PROFILE_AND_LOG_INFO("CreateVisualizationsPredictedPositions");

  if (!ptr_config_->draw_predicted_positions_ground_air_ &&
      !ptr_config_->draw_predicted_regions_)
    return;

  // Store predicted orientations if desired
  for (int k = 0; k < n_stages_; k++) {
    if (ptr_config_->draw_predicted_regions_ && n_states_to_constrain_ == 1)
      predicted_orientations_.emplace_back(
          ptr_solver_->stage_vars_(psi_idx_, k));
    else if (ptr_config_->draw_predicted_regions_ && n_states_to_constrain_ > 1)
      predicted_orientations_.emplace_back(
          stored_states_(psi_idx_ - ptr_solver_->nu_, k));
  }

  // Create predicted position markers if desired
  if (ptr_config_->draw_predicted_positions_ground_air_ >= 2) {
    ROSPointMarker &traj_points =
        predicted_positions_marker_pub_->getNewPointMarker("CYLINDER");
    traj_points.setColor(ptr_config_->predicted_positions_marker_color_[0],
                         ptr_config_->predicted_positions_marker_color_[1],
                         ptr_config_->predicted_positions_marker_color_[2],
                         ptr_config_->predicted_positions_marker_color_[3]);
    traj_points.setScale(ptr_config_->predicted_positions_marker_scale_[0],
                         ptr_config_->predicted_positions_marker_scale_[1],
                         ptr_config_->predicted_positions_marker_scale_[2]);

    ROSLine &traj_lines = predicted_positions_marker_pub_->getNewLine();
    traj_lines.setColor(ptr_config_->predicted_positions_marker_color_[0],
                        ptr_config_->predicted_positions_marker_color_[1],
                        ptr_config_->predicted_positions_marker_color_[2],
                        ptr_config_->predicted_positions_marker_color_[3]);
    traj_lines.setScale(0.5 *
                        ptr_config_->predicted_positions_marker_scale_[0]);

    // Create position markers for and line markers between all stages in the
    // horizon
    for (int k = 0; k < n_stages_; k++) {
      traj_points.addPointMarker(predicted_trajectory_[k].segment(nu_, 3));
      if (k > 0) {
        traj_lines.addLine(predicted_trajectory_[k - 1].segment(nu_, 3),
                           predicted_trajectory_[k].segment(nu_, 3));
      }
    }
  }

  // Create predicted position ground projection markers if desired
  if (ptr_config_->draw_predicted_positions_ground_air_ == 1 ||
      ptr_config_->draw_predicted_positions_ground_air_ >= 3) {
    ROSPointMarker &traj_points_ground =
        predicted_positions_ground_marker_pub_->getNewPointMarker("CYLINDER");
    traj_points_ground.setColor(
        ptr_config_->predicted_positions_marker_color_[0],
        ptr_config_->predicted_positions_marker_color_[1],
        ptr_config_->predicted_positions_marker_color_[2],
        ptr_config_->ground_projection_marker_alpha_);
    traj_points_ground.setScale(
        ptr_config_->predicted_positions_marker_scale_[0],
        ptr_config_->predicted_positions_marker_scale_[1],
        ptr_config_->predicted_positions_marker_scale_[2]);

    ROSLine &traj_lines_ground =
        predicted_positions_ground_marker_pub_->getNewLine();
    traj_lines_ground.setColor(
        ptr_config_->predicted_positions_marker_color_[0],
        ptr_config_->predicted_positions_marker_color_[1],
        ptr_config_->predicted_positions_marker_color_[2],
        ptr_config_->ground_projection_marker_alpha_);
    traj_lines_ground.setScale(
        0.5 * ptr_config_->predicted_positions_marker_scale_[0]);

    // Create position markers for and line markers between all stages in the
    // horizon
    for (int k = 0; k < n_stages_; k++) {
      traj_points_ground.addPointMarker(Eigen::Vector3d(
          predicted_trajectory_[k](nu_), predicted_trajectory_[k](nu_ + 1),
          ptr_config_->ground_projection_marker_z_));
      if (k > 0) {
        traj_lines_ground.addLine(
            Eigen::Vector3d(predicted_trajectory_[k - 1](nu_),
                            predicted_trajectory_[k - 1](nu_ + 1),
                            ptr_config_->ground_projection_marker_z_),
            Eigen::Vector3d(predicted_trajectory_[k](nu_),
                            predicted_trajectory_[k](nu_ + 1),
                            ptr_config_->ground_projection_marker_z_));
      }
    }
  }

  // Create predicted region markers if desired
  if (ptr_config_->draw_predicted_regions_) {
    // Store predicted region poses
    for (int k = 0; k < n_stages_; k++) {
      std::vector<Eigen::Vector2d> region_poses;
      region_poses.reserve(n_regions_to_draw_);
      for (const int &r_idx : ptr_config_->region_indices_to_draw_) {
        region_poses.emplace_back(Eigen::Vector2d(
            ptr_robot_region_->discs_[r_idx].DiscX(
                predicted_trajectory_[k](nu_), predicted_orientations_[k]),
            ptr_robot_region_->discs_[r_idx].DiscY(
                predicted_trajectory_[k](nu_ + 1),
                predicted_orientations_[k])));
      }
      predicted_region_poses_.push_back(region_poses);
    }

    // Draw all region poses as a cylinder
    if (ptr_config_->predicted_regions_n_points_ == 1) {
      ROSPointMarker &traj_regions =
          predicted_regions_marker_pub_->getNewPointMarker("CYLINDER");
      traj_regions.setColor(ptr_config_->predicted_positions_marker_color_[0],
                            ptr_config_->predicted_positions_marker_color_[1],
                            ptr_config_->predicted_positions_marker_color_[2],
                            ptr_config_->predicted_positions_marker_color_[3]);
      traj_regions.setScale(2 * ptr_robot_region_->DiscRadius(),
                            2 * ptr_robot_region_->DiscRadius(),
                            ptr_config_->predicted_regions_marker_scale_[2]);

      // Iterate through horizon stages
      for (int k = 0; k < n_stages_; k++) {
        for (size_t region_idx = 0; region_idx < n_regions_to_draw_;
             region_idx++) {
          traj_regions.addPointMarker(
              Eigen::Vector3d(predicted_region_poses_[k][region_idx](0),
                              predicted_region_poses_[k][region_idx](1),
                              ptr_config_->predicted_regions_marker_z_));
        }
      }
    }
    // Draw all region poses as circular lines
    else if (ptr_config_->predicted_regions_n_points_ >= 10) {
      ROSLine &traj_regions = predicted_regions_marker_pub_->getNewLine();
      traj_regions.setColor(ptr_config_->predicted_positions_marker_color_[0],
                            ptr_config_->predicted_positions_marker_color_[1],
                            ptr_config_->predicted_positions_marker_color_[2],
                            ptr_config_->predicted_positions_marker_color_[3]);
      traj_regions.setScale(ptr_config_->predicted_regions_marker_scale_[0]);

      // Iterate through horizon stages
      for (int k = 0; k < n_stages_; k++) {
        for (size_t region_idx = 0; region_idx < n_regions_to_draw_;
             region_idx++) {
          traj_regions.addCircularLine(
              predicted_region_poses_[k][region_idx],
              ptr_robot_region_->DiscRadius(),
              ptr_config_->predicted_regions_marker_z_,
              ptr_config_->predicted_regions_n_points_);
        }
      }
    }

    predicted_orientations_.clear();
    for (size_t k = 0; k < predicted_region_poses_.size(); k++) {
      predicted_region_poses_[k].clear();
    }
    predicted_region_poses_.clear();
  }

  predicted_trajectory_.clear();
}

void HovergamesDroneInterface::PublishVisualizationsCurrentPosition() {
  PROFILE_AND_LOG_INFO("PublishVisualizationsCurrentPosition");

  current_position_marker_pub_->publish();
  current_position_ground_marker_pub_->publish();
  current_region_marker_pub_->publish();
}

void HovergamesDroneInterface::PublishVisualizationsPredictedTrajectory() {
  PROFILE_AND_LOG_INFO("PublishVisualizationsPredictedTrajectory");

  predicted_positions_marker_pub_->publish();
  predicted_positions_ground_marker_pub_->publish();
  predicted_regions_marker_pub_->publish();
}

// Data saving methods
void HovergamesDroneInterface::AddStaticData() {
  PROFILE_AND_LOG_INFO("AddStaticData");

  // Add static data
  ptr_data_saver_json_->AddStaticData("dt", ptr_solver_->dt_);
  ptr_data_saver_json_->AddStaticData("stepsize", integrator_stepsize_);
  ptr_data_saver_json_->AddStaticData("robot_radius",
                                      ptr_robot_region_->discs_[0].radius_);
  if (ptr_config_->n_layers_ > 1) {
    if (ptr_config_->layer_idx_ == 0) {
      ptr_data_saver_json_->AddStaticData("Q", Q_);
      ptr_data_saver_json_->AddStaticData("R", R_);
    }
    if (ptr_config_->layer_idx_ == 1) {
      ptr_data_saver_json_->AddStaticData("c_o", c_o_);
      ptr_data_saver_json_->AddStaticData("alpha", alpha_);
      ptr_data_saver_json_->AddStaticData("P_delta", P_delta_);
    }
  }
}
