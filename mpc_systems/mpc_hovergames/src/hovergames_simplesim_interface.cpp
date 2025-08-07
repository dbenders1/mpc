#include <mpc_hovergames/hovergames_simplesim_interface.h>
#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>

HovergamesSimpleSimInterface::HovergamesSimpleSimInterface(
    ros::NodeHandle &nh, ControllerBase *ptr_controller,
    ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
    ModuleHandlerBase *ptr_module_handler, RobotRegion *ptr_robot_region,
    DataSaverJson *ptr_data_saver_json, std::mutex *mutex)
    : HovergamesDroneInterface(nh, ptr_controller, ptr_config, ptr_solver,
                               ptr_module_handler, ptr_robot_region,
                               ptr_data_saver_json, mutex) {
  MPC_WARN_ALWAYS("Initializing hovergames simple sim interface");

  /***************** Common initialization *****************/
  // Initialize control message
  control_msg_.header.frame_id = ptr_config_->robot_base_link_;
  control_msg_.u[0] = 0;
  control_msg_.u[1] = 0;
  control_msg_.u[2] = 0;
  control_msg_.u[3] = 9.81;
  /*********************************************************/

  /************ MPC layer-specific initialization **********/
  if (ptr_config_->layer_idx_ == 0) {
    // Publisher for vehicle command
    command_pub_0_ = nh_.advertise<simple_sim::DroneHovergamesControl>(
        ptr_config_->control_command_topic_, 1);
  }
  /*********************************************************/

  MPC_WARN_ALWAYS("Hovergames simple sim interface initialized");
}

void HovergamesSimpleSimInterface::OnStateCallback(
    const nav_msgs::Odometry &msg) {
  PROFILE_FUNCTION();

  // Start control loop when receiving the first state and check if already has
  // reference trajectory and constraints
  if (!first_state_received_) {
    first_state_received_ = true;

    if (!first_ref_received_ && ptr_config_->n_layers_ > 1 &&
        ptr_config_->layer_idx_ == 0) {
      start_stage_vars_ = ptr_solver_->last_updated_stage_vars_;

      // Fill in reference trajectory and constraints with hover values at
      // current state
      SetReferenceTrajectoryHover();
      SetConstraintsHover();
    }

    if (ptr_config_->layer_idx_ == 0) {
      ptr_controller_->startTimer();
    };
  }
}

void HovergamesSimpleSimInterface::OnComputeActuation() {
  PROFILE_FUNCTION();

  // Fill command message
  control_msg_.u[0] = u_solver_(0);
  control_msg_.u[1] = u_solver_(1);
  control_msg_.u[2] = u_solver_(2);
  control_msg_.u[3] = u_solver_(3);
}

void HovergamesSimpleSimInterface::OnActuate() {
  PROFILE_FUNCTION();

  // Publish control command
  control_msg_.header.stamp = ros::Time::now();
  command_pub_0_.publish(control_msg_);
}

void HovergamesSimpleSimInterface::OnDeployObjectiveReachedStrategy() {
  PROFILE_FUNCTION();

  // No steps necessary for the objective reached strategy in the simple sim
  // interface
}

void HovergamesSimpleSimInterface::OnDeployEmergencyStrategy() {
  PROFILE_FUNCTION();

  // No steps necessary for the emergency strategy in the simple sim interface
}

void HovergamesSimpleSimInterface::OnActuateBrake() {
  PROFILE_FUNCTION();

  // No steps necessary for braking in the simple sim interface
}
