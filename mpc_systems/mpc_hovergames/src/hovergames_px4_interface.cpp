#include <mpc_hovergames/hovergames_px4_interface.h>
#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

HovergamesPX4Interface::HovergamesPX4Interface(
    ros::NodeHandle &nh, ControllerBase *ptr_controller,
    ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
    ModuleHandlerBase *ptr_module_handler, RobotRegion *ptr_robot_region,
    DataSaverJson *ptr_data_saver_json, std::mutex *mutex)
    : HovergamesDroneInterface(nh, ptr_controller, ptr_config, ptr_solver,
                               ptr_module_handler, ptr_robot_region,
                               ptr_data_saver_json, mutex) {
  MPC_WARN_ALWAYS("Initializing hovergames px4 interface");

  // Subscriber for the hover thrust estimate
  hover_thrust_sub_ =
      nh.subscribe("/mavros/hover_thrust_estimate", 1,
                   &HovergamesPX4Interface::hoverThrustEstimateCallback, this);
  battery_voltage_sub_ =
      nh.subscribe("/mavros/battery", 1,
                   &HovergamesPX4Interface::batteryVoltageCallback, this);

  // Service client for transferring control to PX4 control interface
  mission_finished_client_ =
      nh.serviceClient<std_srvs::Trigger>("px4_mission_finished_ext_cont");

  // Initialize control message
  // Construct default attitude target message (used for attitude and thrust
  // commands)
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  // Fill command message
  att_target_msg_.type_mask = 7;  // binary: 0000 0111 => ignore everything
                                  // except attitude and thrust setpoints
  att_target_msg_.header.frame_id = ptr_config_->robot_base_link_;
  att_target_msg_.orientation.x = q.x();
  att_target_msg_.orientation.y = q.y();
  att_target_msg_.orientation.z = q.z();
  att_target_msg_.orientation.w = q.w();
  att_target_msg_.body_rate.x = NAN;
  att_target_msg_.body_rate.y = NAN;
  att_target_msg_.body_rate.z = NAN;
  att_target_msg_.thrust = hover_thrust_;

  // Initialize thrust scaling
  if (ptr_config_->sim_) thrust_scaling_ = 0.7;
  MPC_INFO("thrust_scaling_: " << thrust_scaling_);

  // Initialize position target message (in case of emergency)
  pos_target_msg_.header.frame_id = ptr_config_->robot_base_link_;
  pos_target_msg_.coordinate_frame = 1;
  pos_target_msg_.type_mask =
      2552;  // Binary for which message attributes not to use

  // Initialize acceleration target message
  acceleration_msg_.header.frame_id = ptr_config_->robot_base_link_;

  // Initialize position guess message
  position_guess_msg_.header.frame_id = ptr_config_->robot_base_link_;

  // Time reference handling
  if (ptr_config_->record_time_reference_) {
    time_ref_sub_ =
        nh.subscribe("/mavros/time_reference", 1,
                     &HovergamesPX4Interface::timeReferenceCallback, this);
    time_ref_pub_ = nh.advertise<sensor_msgs::TimeReference>(
        ptr_config_->time_reference_rec_topic_, 1);
    time_ref_msg_.header.frame_id = ptr_config_->robot_base_link_;
  }

  /************ MPC layer-specific initialization **********/
  if (ptr_config_->layer_idx_ == 0) {
    // Service servers for running node
    enable_control_server_ = nh.advertiseService(
        "/px4_ext_cont_enable", &HovergamesPX4Interface::EnableControlCallback,
        this);
    disable_control_server_ = nh.advertiseService(
        "/px4_ext_cont_disable",
        &HovergamesPX4Interface::DisableControlCallback, this);

    // Subscribers for sensor data
    imu_sub_ = nh.subscribe("/mavros/imu/data", 1,
                            &HovergamesPX4Interface::imuCallback, this);

    // Publishers for vehicle commands
    attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
        ptr_config_->control_command_topic_, 1);
    position_pub_ = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 1);

    // Sending out solver information
    acceleration_pub_ =
        nh.advertise<sensor_msgs::Imu>("/solver_info/acceleration_setpoint", 1);
    position_guess_pub_ =
        nh.advertise<nav_msgs::Odometry>("/solver_info/position_guess", 1);
  }
  /*********************************************************/

  MPC_WARN_ALWAYS("Hovergames px4 interface initialized");
}

void HovergamesPX4Interface::OnStateCallback(const nav_msgs::Odometry &msg) {
  PROFILE_FUNCTION();

  // Convert velocities from body to world frame
  tf2::Quaternion q_tf;
  tf2::convert(msg.pose.pose.orientation, q_tf);
  Eigen::Quaterniond q =
      Eigen::Quaterniond(q_tf.w(), q_tf.x(), q_tf.y(), q_tf.z());
  Eigen::Vector3d vel_body =
      Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z);
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d vel_world = R * vel_body;

  // Store velocities (overwrite the values stored in drone interface)
  ptr_solver_->last_updated_stage_vars_(vx_idx_) = vel_world(0);
  ptr_solver_->last_updated_stage_vars_(vy_idx_) = vel_world(1);
  ptr_solver_->last_updated_stage_vars_(vz_idx_) = vel_world(2);
  ptr_solver_->last_updated_stage_vars_(thrust_idx_) = acc_z_;

  // Store data for actuate brake
  state_x_ = ptr_solver_->last_updated_stage_vars_(x_idx_);
  state_y_ = ptr_solver_->last_updated_stage_vars_(y_idx_);
  state_z_ = ptr_solver_->last_updated_stage_vars_(z_idx_);
  state_yaw_ = ptr_solver_->last_updated_stage_vars_(psi_idx_);
}

void HovergamesPX4Interface::OnComputeActuation() {
  PROFILE_FUNCTION();

  // Compute quaternion
  tf2::Quaternion q;
  q.setRPY(u_solver_[0], u_solver_[1], u_solver_[2]);

  // The predicted orientation of the drone
  double roll = ptr_solver_->stage_vars_(phi_idx_, 0);
  double pitch = ptr_solver_->stage_vars_(theta_idx_, 0);
  double yaw = ptr_solver_->stage_vars_(psi_idx_, 0);

  // Quaternion for the predicted orientation of the drone
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q_predicted = yawAngle * pitchAngle * rollAngle;

  // Fill command message
  att_target_msg_.header.stamp = ros::Time::now();
  att_target_msg_.orientation.x = q.x();
  att_target_msg_.orientation.y = q.y();
  att_target_msg_.orientation.z = q.z();
  att_target_msg_.orientation.w = q.w();

  position_guess_msg_.pose.pose.position.x =
      ptr_solver_->stage_vars_(x_idx_, 0);
  position_guess_msg_.pose.pose.position.y =
      ptr_solver_->stage_vars_(y_idx_, 0);
  position_guess_msg_.pose.pose.position.z =
      ptr_solver_->stage_vars_(z_idx_, 0);
  position_guess_msg_.pose.pose.orientation.x = q_predicted.x();
  position_guess_msg_.pose.pose.orientation.y = q_predicted.y();
  position_guess_msg_.pose.pose.orientation.z = q_predicted.z();
  position_guess_msg_.pose.pose.orientation.w = q_predicted.w();
  position_guess_msg_.twist.twist.linear.x =
      ptr_solver_->stage_vars_(vx_idx_, 0);
  position_guess_msg_.twist.twist.linear.y =
      ptr_solver_->stage_vars_(vy_idx_, 0);
  position_guess_msg_.twist.twist.linear.z =
      ptr_solver_->stage_vars_(vz_idx_, 0);
}

void HovergamesPX4Interface::OnActuate() {
  PROFILE_FUNCTION();

  ////////////////////////////////////////////////////////////////////////////////////////
  // FORCE TO THRUST MAPPING
  ////////////////////////////////////////////////////////////////////////////////////////
  // Calculate thrust using latest information
  tf2::Quaternion q_tf;
  tf2::convert(last_state_msg_.pose.pose.orientation, q_tf);
  tf2Scalar roll_state, pitch_state, yaw_state;
  tf2::Matrix3x3(q_tf).getEulerYPR(yaw_state, pitch_state, roll_state);
  Eigen::Quaterniond q =
      Eigen::Quaterniond(q_tf.w(), q_tf.x(), q_tf.y(), q_tf.z());

  double acc_magnitude = ptr_solver_->stage_vars_(thrust_c_idx_, 0);
  roll_state = ptr_solver_->stage_vars_(phi_idx_, 0);
  pitch_state = ptr_solver_->stage_vars_(theta_idx_, 0);
  yaw_state = ptr_solver_->stage_vars_(psi_idx_, 0);

  double g = 9.8066;

  // Rotate unit vector in z direction to the body frame to know components of
  // force in world frame
  Eigen::AngleAxisd rollAngle(roll_state, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch_state, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw_state, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q_acc_magnitude = yawAngle * pitchAngle * rollAngle;

  Eigen::Vector3d acc_vector_world =
      q_acc_magnitude * Eigen::Vector3d(0, 0, 1) * acc_magnitude;

  // We need to subtract the gravitational force to get the acceleration
  // setpoint
  acc_vector_world(2) -= g;

  acceleration_msg_.linear_acceleration.x = acc_vector_world(0);
  acceleration_msg_.linear_acceleration.y = acc_vector_world(1);
  acceleration_msg_.linear_acceleration.z = acc_vector_world(2) + g;

  // Scale thrust assuming hover thrust produces standard gravity
  Eigen::Vector3d body_z =
      Eigen::Vector3d(acc_vector_world(0), acc_vector_world(1), g).normalized();
  double collective_thrust =
      thrust_scaling_ * (acc_vector_world(2) * (hover_thrust_ / g)) +
      hover_thrust_;

  // Project thrust to planned body attitude
  collective_thrust /= (Eigen::Vector3d(0, 0, 1).dot(body_z));
  Eigen::Vector3d thrust_sp = body_z * collective_thrust;

  // Turn thrust to real body frame
  // transformation from body frame to world frame
  // Received attitude
  // Eigen::Matrix3d R_b_to_w = q.toRotationMatrix();
  // Planned attitude
  Eigen::Matrix3d R_b_to_w = q_acc_magnitude.toRotationMatrix();

  // Transformation from world frame to body frame
  Eigen::Matrix3d R_w_to_b = R_b_to_w.transpose();

  // Thrust in body frame
  Eigen::Vector3d thrust_sp_body = R_w_to_b * Eigen::Vector3d(0, 0, 1);

  // Check difference between unit vector and thrust vector z world
  double scalar = thrust_sp_body(2) / thrust_sp(2);

  thrust_sp_body = thrust_sp_body / scalar;

  att_target_msg_.thrust = thrust_sp_body.norm();

  // Publish attitude and thrust commands
  att_target_msg_.header.stamp = ros::Time::now();
  attitude_pub_.publish(att_target_msg_);

  acceleration_msg_.header.stamp = ros::Time::now();
  acceleration_pub_.publish(acceleration_msg_);

  position_guess_msg_.header.stamp = ros::Time::now();
  position_guess_pub_.publish(position_guess_msg_);
}

void HovergamesPX4Interface::OnDeployObjectiveReachedStrategy() {
  PROFILE_FUNCTION();

  // Construct and publish position target message
  pos_target_msg_.header.stamp = ros::Time::now();
  pos_target_msg_.position.x = state_x_;
  pos_target_msg_.position.y = state_y_;
  pos_target_msg_.position.z = state_z_;
  pos_target_msg_.yaw = state_yaw_;
  position_pub_.publish(pos_target_msg_);
}

void HovergamesPX4Interface::OnDeployEmergencyStrategy() {
  PROFILE_FUNCTION();

  // Construct and publish position target message
  pos_target_msg_.header.stamp = ros::Time::now();
  pos_target_msg_.position.x = state_x_;
  pos_target_msg_.position.y = state_y_;
  pos_target_msg_.position.z = state_z_;
  pos_target_msg_.yaw = state_yaw_;
  position_pub_.publish(pos_target_msg_);
}

void HovergamesPX4Interface::OnActuateBrake() {
  PROFILE_FUNCTION();

  // Publish position target message
  pos_target_msg_.header.stamp = ros::Time::now();
  position_pub_.publish(pos_target_msg_);
}

// Methods specific to Hovergames PX4 interface
bool HovergamesPX4Interface::EnableControlCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  PROFILE_AND_LOG_INFO(
      "EnableControlCallback: enabling control loop by starting timer");

  // Start control loop when receiving the first state and check if already has
  // reference trajectory and constraints
  if (!first_ref_received_ && ptr_config_->n_layers_ > 1 &&
      ptr_config_->layer_idx_ == 0) {
    start_stage_vars_ = ptr_solver_->last_updated_stage_vars_;

    // Fill in refeence trajectory and constraints with hover values at current
    // state
    SetReferenceTrajectoryHover();
    SetConstraintsHover();
  }

  ptr_controller_->startTimer();
  res.success = true;
  res.message = "Enabled MPC control loop";

  return true;
}

bool HovergamesPX4Interface::DisableControlCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  PROFILE_AND_LOG_INFO(
      "DisableControlCallback: disabling control Loop by resetting solver and "
      "stopping timer");

  ptr_controller_->OnReset();
  ptr_controller_->stopTimer();
  res.success = true;
  res.message = "Disabled MPC control loop";

  return true;
}

void HovergamesPX4Interface::batteryVoltageCallback(
    const sensor_msgs::BatteryState &msg) {
  PROFILE_FUNCTION();

  battery_voltage_ = msg.voltage;
}

void HovergamesPX4Interface::hoverThrustEstimateCallback(
    const mavros_msgs::HoverThrustEstimate &msg) {
  PROFILE_FUNCTION();

  hover_thrust_ = msg.hover_thrust;
}

void HovergamesPX4Interface::imuCallback(const sensor_msgs::Imu &msg) {
  PROFILE_FUNCTION();

  // Store acceleration in body z direction
  acc_z_ = msg.linear_acceleration.z;
}

void HovergamesPX4Interface::timeReferenceCallback(
    const sensor_msgs::TimeReference &msg) {
  // First store current time stamp to reduce the delay of receiving the message
  // as much as possible
  time_ref_msg_.header.stamp = ros::Time::now();

  PROFILE_FUNCTION();

  time_ref_msg_.time_ref = msg.time_ref;
  time_ref_msg_.source = msg.source;
  time_ref_pub_.publish(time_ref_msg_);
}
