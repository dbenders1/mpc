#ifndef FALCON_INTERFACE_H
#define FALCON_INTERFACE_H

#include <mpc_core/interface.h>
#include <mpc_modules/loader/module_handler.h>
#include <mpc_msgs/Float64Array.h>
#include <mpc_msgs/InterpData.h>
#include <mpc_msgs/ReferenceTrajectoryConstraints.h>
#include <mpc_msgs/ScalarOverHorizon.h>
#include <mpc_msgs/StartMpcLayer.h>
#include <mpc_msgs/TrajectoryOverHorizon.h>
#include <mpc_tools/RK4.h>
#include <mpc_tools/ros_visuals.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Dense>
#include <vector>

/* RK4 integration used in hierarchical MPC */
class FalconRK4System {
 public:
  FalconRK4System() {};
  ~FalconRK4System() {};
  Eigen::VectorXd rk4(const Eigen::VectorXd &x, const Eigen::VectorXd &u,
                      const double dt, bool use_observer_dynamics,
                      const Eigen::VectorXd &y) {
    Eigen::VectorXd k1 = f(x, u);
    if (use_observer_dynamics) k1 += f_add_observer(x, y);
    Eigen::VectorXd x_k2 = x + dt / 2 * k1;
    Eigen::VectorXd k2 = f(x_k2, u);
    if (use_observer_dynamics) k2 += f_add_observer(x_k2, y);
    Eigen::VectorXd x_k3 = x + dt / 2 * k2;
    Eigen::VectorXd k3 = f(x_k3, u);
    if (use_observer_dynamics) k3 += f_add_observer(x_k3, y);
    Eigen::VectorXd x_k4 = x + dt * k3;
    Eigen::VectorXd k4 = f(x_k4, u);
    if (use_observer_dynamics) k4 += f_add_observer(x_k4, y);
    return x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
  }

  Eigen::MatrixXd rk4_n(const Eigen::VectorXd &x, const Eigen::VectorXd &u,
                        const double dt, int n_steps,
                        bool use_observer_dynamics, const Eigen::VectorXd &y) {
    Eigen::MatrixXd x_rk4_after = Eigen::MatrixXd::Zero(n_steps + 1, x.size());
    x_rk4_after.row(0) = x.transpose();
    for (int i = 0; i < n_steps; ++i) {
      x_rk4_after.row(i + 1) =
          rk4(x_rk4_after.row(i).transpose(), u, dt, use_observer_dynamics, y)
              .transpose();
    }
    return x_rk4_after;
  }

  Eigen::VectorXd f(const Eigen::VectorXd &x, const Eigen::Vector4d &u) {
    double phi = x[3];
    double theta = x[4];
    double psi = x[5];
    Eigen::Vector3d v = x.segment<3>(6);
    Eigen::Vector3d wb = x.segment<3>(9);

    double t = (B_allocation_.row(0) * u)(0);
    Eigen::Vector3d tau = B_allocation_.block<3, 4>(1, 0) * u;

    Eigen::Matrix3d Rc = get_rot_matrix_coordinates(phi, theta, psi);
    Eigen::Matrix3d Rr = get_rot_matrix_rates(phi, theta);

    Eigen::VectorXd dxdt(12);
    // Eigen::VectorXd dxdt(16);
    dxdt.segment<3>(0) = v;
    dxdt.segment<3>(3) = Rr * wb;
    dxdt.segment<3>(6) =
        Eigen::Vector3d(0, 0, -g_) +
        Rc * (Eigen::Vector3d(0, 0, t / mass_) - kd_ * Rc.transpose() * v);
    dxdt.segment<3>(9) = inertia_.inverse() * (tau - wb.cross(inertia_ * wb));

    return dxdt + E_ * w_bias_;
  }

  Eigen::VectorXd f_add_observer(const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &y) {
    return L_ * (y - C_ * x);
  }

  Eigen::Vector4d motor_speeds_to_thrusts(const Eigen::Vector4d &motor_speeds) {
    return thrust_map_[0] * motor_speeds.array().square().matrix() +
           thrust_map_[1] * motor_speeds +
           thrust_map_[2] * Eigen::Vector4d::Ones();
  }

  void set_B_allocation(const Eigen::MatrixXd &B_allocation) {
    B_allocation_ = B_allocation;
  }

  void set_C(const Eigen::MatrixXd &C) { C_ = C; }

  void set_E(const Eigen::MatrixXd &E) { E_ = E; }

  void set_g(double g) { g_ = g; }

  void set_inertia(const Eigen::MatrixXd &inertia) { inertia_ = inertia; }

  void set_kd(const Eigen::MatrixXd &kd) { kd_ = kd; }

  void set_mass(double mass) { mass_ = mass; }

  void set_thrust_map(const Eigen::VectorXd &thrust_map) {
    thrust_map_ = thrust_map;
  }

  void set_w_bias(const Eigen::VectorXd &w_bias) { w_bias_ = w_bias; }

  void set_L(const Eigen::MatrixXd &L) { L_ = L; }

 private:
  Eigen::Matrix3d get_rot_matrix_coordinates(double phi, double theta,
                                             double psi) {
    Eigen::Matrix3d R_psi;
    R_psi << cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1;

    Eigen::Matrix3d R_theta;
    R_theta << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);

    Eigen::Matrix3d R_phi;
    R_phi << 1, 0, 0, 0, cos(phi), -sin(phi), 0, sin(phi), cos(phi);

    return R_psi * R_theta * R_phi;
  }

  Eigen::Matrix3d get_rot_matrix_rates(double phi, double theta) {
    Eigen::Matrix3d Rr;
    Rr << 1, sin(phi) * tan(theta), cos(phi) * tan(theta), 0, cos(phi),
        -sin(phi), 0, sin(phi) / cos(theta), cos(phi) / cos(theta);

    return Rr;
  }

  double g_, mass_;
  Eigen::VectorXd thrust_map_, w_bias_;
  Eigen::Matrix3d inertia_;  // to prevent Eigen compile error for cross product
                             // with 3x1 vector
  Eigen::MatrixXd B_allocation_, C_, E_, kd_, L_;
};

class FalconInterface : public Interface {
 public:
  // Constructor and destructor
  FalconInterface(ros::NodeHandle &nh, ControllerBase *ptr_controller,
                  ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
                  ModuleHandlerBase *ptr_module_handler,
                  RobotRegion *ptr_robot_region,
                  DataSaverJson *ptr_data_saver_json, std::mutex *mutex);
  ~FalconInterface() {};

  // Interface base methods
  void AtControlLoopStart() override;
  void AfterUpdateModules() override;
  void AfterSetParametersModules() override;
  void AfterOptimization() override;
  void ComputeActuation() override;
  void Actuate() override;
  void DeployObjectiveReachedStrategy() override;
  void DeployEmergencyStrategy() override;
  void Reset() override;
  void CreateVisualizations() override;
  void PublishVisualizations() override;
  void AddStaticData() override;
  bool setInterval(const double dt) override;
  bool setLastCommand(const Eigen::VectorXd &last_command) override;
  bool setState(const double t, const Eigen::VectorXd &state) override;
  bool run() override;
  Eigen::VectorXd getCommand() override;

  // Methods specific to falcon interface
  void ActuateBrake();
  Eigen::VectorXd observeState(const Eigen::VectorXd &y);

  // Solver solution methods
  void PublishCurrentState();
  void PublishPredictedTrajectory();
  void PublishSlack();

  // HMPC methods
  void StartMpcLayerPublish();
  void StartMpcLayerCallback(const mpc_msgs::StartMpcLayer &msg);
  void SetReferenceTrajectoryHover();
  void SetConstraintsHover();
  void RefCallback(const mpc_msgs::ReferenceTrajectoryConstraints &msg);
  void CheckRef();
  void SetReferenceTrajectory();
  void SetConstraints();
  void PmpcObjectiveReachedCallback(const std_msgs::Empty &msg);
  void PmpcFailureCallback(const std_msgs::Empty &msg);

  // ROS visualization methods
  void CreateVisualizationsCurrentPosition();
  void CreateVisualizationsPredictedPositions();
  void PublishVisualizationsCurrentPosition();
  void PublishVisualizationsPredictedPositions();

 private:
  // Modulehandler pointer to interface with real-time data
  ModuleHandler *ptr_module_handler_;

  // ROS publisher and subscribers
  ros::Publisher start_mpc_layer_pub_, ref_pub_, objective_reached_pub_,
      pmpc_failure_pub_;
  ros::Subscriber start_mpc_layer_sub_, ref_sub_, objective_reached_sub_,
      pmpc_failure_sub_;

  // ROS messages
  mpc_msgs::StartMpcLayer start_mpc_layer_msg_;
  mpc_msgs::ReferenceTrajectoryConstraints ref_msg_;

  // ROS recording publishers and messages
  ros::Publisher cur_state_rec_pub_, interp_data_pub_, nominal_reference_pub_,
      pred_traj_rec_pub_, slack_rec_pub_;
  mpc_msgs::Float64Array cur_state_msg_;
  mpc_msgs::InterpData interp_data_msg_;
  mpc_msgs::TrajectoryOverHorizon nominal_reference_msg_;
  mpc_msgs::TrajectoryOverHorizon pred_traj_msg_;
  mpc_msgs::ScalarOverHorizon slack_msg_;

  // MPC info
  int t0c_idx_, t1c_idx_, t2c_idx_, t3c_idx_;
  int x_idx_, y_idx_, z_idx_, phi_idx_, theta_idx_, psi_idx_, vx_idx_, vy_idx_,
      vz_idx_, wbx_idx_, wby_idx_, wbz_idx_;
  int layer_idx_ = 0;
  int N_ = 0;
  int nu_ = 0;
  int nx_ = 0;

  // Model info
  double t_hover_ = 0;

  // Steady-state input
  Eigen::Vector4d u_steady_state_;

  // Hover state
  Eigen::VectorXd x_hover_;

  // Slack
  bool use_slack_ = false;
  int slack_idx_;

  // Store first stage vars received
  Eigen::VectorXd start_stage_vars_;

  // Agi command
  double dt_control_loop_ = 0.01;
  int control_loop_idx_since_last_run_ = 0, n_control_loops_per_mpc_loop_ = 1;
  Eigen::VectorXd mpc_command_, u_;

  // Indicator variables for objective reached and emergency
  bool objective_reached_ = false;
  bool emergency_ = false;

  // RK4 integration
  FalconRK4System rk4_sys_;
  double integrator_stepsize_ = 0.01, integrator_steps_ = 1;

  // Constraints tightening parameters
  std::vector<std::vector<double>> Q_, R_, P_;
  std::vector<std::vector<double>> P_delta_;
  double rho_c_ = 0, w_bar_c_ = 0, epsilon_ = 0, alpha_ = 0, c_o_ = 0;
  Eigen::VectorXd w_bias_, s_pred_, tightening_pred_;
  bool use_tightened_obstacle_constraints_ = false;

  // Feedback law parameters
  Eigen::MatrixXd K_delta_;
  int n_rk4_states_to_rec_ = 4;
  Eigen::MatrixXd interp_states_to_rec_;

  // HMPC
  bool schedule_planner_ = true, reset_planner_ = true;  // TMPC-specific
  int tmpc_count_ = 0;                                   // PMPC-specific
  int ratio_;  // ratio between PMPC and TMPC sampling times, called beta in
               // paper

  // Variables needed for starting optimization taking into account equality
  // constraints on PMPC stages (to ensure continuous reference trajectory and
  // obstacle avoidance constraints for TMPC) Amount of initial state equality
  // constraints equals n_states_to_constrain_, meaning that index
  // n_states_to_constrain_-1 is the index of the initial stage of the current
  // run Note: these variables are also used for visualization in PMPC
  int n_states_to_constrain_ =
      1;  // at least the initial state is constrained, so minimum is 1
  int n_stages_to_store_ =
      0;  // TMPC: number of reference trajectory and obstacle avoidance
          // constraints stages to store, PMPC: number of input, state and
          // obstacle avoidance constraints stages to store for communication
          // from PMPC to TMPC and visualization
  int n_pmpc_stages_to_send_, n_pmpc_stages_to_receive_ = 0;
  Eigen::MatrixXd stored_inputs_, stored_states_, stored_constraints_;
  Eigen::VectorXi stored_n_relevant_constraints_;

  // HMPC failure variables
  bool updated_ref_received_ =
      false;  // indicator whether a new reference has been received in time
  int ref_stage_idx_start_ = 0;  // number of stages to shift last received
                                 // reference to track in current run

  // TMPC
  bool first_state_received_ = false;
  bool first_ref_received_ = false;

  // ROS visuals publishers
  std::unique_ptr<ROSMarkerPublisher> current_position_marker_pub_,
      current_position_ground_marker_pub_;
  std::unique_ptr<ROSMarkerPublisher> current_region_marker_pub_;
  std::unique_ptr<ROSMarkerPublisher> predicted_positions_marker_pub_,
      predicted_positions_ground_marker_pub_;
  std::unique_ptr<ROSMarkerPublisher> predicted_regions_marker_pub_;

  // ROS visuals storing variables
  int n_stages_;  // number of stages in the MPC, also used for communication
  size_t n_regions_to_draw_;
  std::vector<Eigen::Vector2d> region_poses_;
  std::vector<Eigen::VectorXd> predicted_trajectory_;
  std::vector<double> predicted_orientations_;
  std::vector<std::vector<Eigen::Vector2d>> predicted_region_poses_;

  // For using a custom nominal reference in getCommand()
  Eigen::VectorXd x_init_;
  int cmd_idx_ = 0;
};

#endif  // FALCON_INTERFACE_H
