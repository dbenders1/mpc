#ifndef HOVERGAMES_DRONE_INTERFACE_H
#define HOVERGAMES_DRONE_INTERFACE_H

#include <mpc_core/interface.h>
#include <mpc_modules/loader/module_handler.h>
#include <mpc_msgs/Float64Array.h>
#include <mpc_msgs/ReferenceTrajectoryConstraints.h>
#include <mpc_msgs/ScalarOverHorizon.h>
#include <mpc_msgs/StartMpcLayer.h>
#include <mpc_msgs/TrajectoryOverHorizon.h>
#include <mpc_tools/RK4.h>
#include <mpc_tools/ros_visuals.h>
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_msgs/Empty.h>

#include <Eigen/Core>
#include <cmath>
#include <mutex>
#include <vector>

/* RK4 integration used in hierarchical MPC */
typedef std::vector<double> rk4_state_type_;
template <class StateType>
class HovergamesSystem : public Helpers::RK4::System<StateType> {
 public:
  HovergamesSystem() {
    u.resize(4);
    u[0] = 0.0;
    u[1] = 0.0;
    u[2] = 0.0;
    u[3] = 0.0;
  };
  ~HovergamesSystem() {};

  std::vector<double> u;

  // x = [x;y;z;vx;vy;vz;phi;theta;psi]
  void operator()(const StateType &x, StateType &dxdt, double t) {
    const double A = -5.55;
    const double B = 5.55;

    const double A_yaw = -1.773;
    const double B_yaw = 1.773;

    const double A_t_average = -20;
    const double B_t_average = 20;

    // NOTE: these equations should match the equations in the corresponding
    // model in python_forces_code/dynamics.py!
    dxdt[0] = x[3];
    dxdt[1] = x[4];
    dxdt[2] = x[5];
    dxdt[3] =
        x[9] * (sin(x[6]) * sin(x[8]) + cos(x[6]) * sin(x[7]) * cos(x[8]));
    dxdt[4] =
        x[9] * (-sin(x[6]) * cos(x[8]) + cos(x[6]) * sin(x[7]) * sin(x[8]));
    dxdt[5] = x[9] * cos(x[6]) * cos(x[7]) - g;
    dxdt[6] = A * x[6] + B * u[0];
    dxdt[7] = A * x[7] + B * u[1];
    dxdt[8] = A_yaw * x[8] + B_yaw * u[2];
    dxdt[9] = A_t_average * x[9] + B_t_average * u[3];
  };

 private:
  const double g = 9.81;
};

class HovergamesDroneInterface : public Interface {
 public:
  // Constructor and destructor
  HovergamesDroneInterface(ros::NodeHandle &nh, ControllerBase *ptr_controller,
                           ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
                           ModuleHandlerBase *ptr_module_handler,
                           RobotRegion *ptr_robot_region,
                           DataSaverJson *ptr_data_saver_json,
                           std::mutex *mutex);
  ~HovergamesDroneInterface() {};

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

  // Methods specific to drone interface
  void StateCallback(const nav_msgs::Odometry &msg);
  void ActuateBrake();

  // Methods overriden by derived drone interface classes
  virtual void OnStateCallback(const nav_msgs::Odometry &msg) = 0;
  virtual void OnComputeActuation() = 0;
  virtual void OnActuate() = 0;
  virtual void OnDeployObjectiveReachedStrategy() = 0;
  virtual void OnDeployEmergencyStrategy() = 0;
  virtual void OnActuateBrake() = 0;

  // Solver solution methods
  void PublishCurrentState();
  void PublishPredictedTrajectory();
  void PublishSlack();

  // HMPC methods
  void StartMpcLayerPublish();
  void StartMpcLayerCallback(const mpc_msgs::StartMpcLayer &msg);
  void RefCallback(const mpc_msgs::ReferenceTrajectoryConstraints &msg);
  void SetReferenceTrajectoryHover();
  void SetConstraintsHover();
  void CheckRef();
  void SetReferenceTrajectory();
  void SetConstraints();
  void PmpcObjectiveReachedCallback(const std_msgs::Empty &msg);
  void PmpcFailureCallback(const std_msgs::Empty &msg);

  // ROS visualization methods
  void CreateVisualizationsCurrentPosition();
  void CreateVisualizationsPredictedPositions();
  void PublishVisualizationsCurrentPosition();
  void PublishVisualizationsPredictedTrajectory();

  // Available in derived classes
 protected:
  // State and reference trajectory received indication
  bool first_state_received_ = false, first_ref_received_ = false;

  // Data indices in the solver matrices
  int phi_c_idx_, theta_c_idx_, psi_c_idx_, thrust_c_idx_;
  int x_idx_, y_idx_, z_idx_, vx_idx_, vy_idx_, vz_idx_, phi_idx_, theta_idx_,
      psi_idx_, thrust_idx_, thrust_1_idx_;

  // Solver output
  Eigen::VectorXd u_solver_;

  // Store first stage vars received
  Eigen::VectorXd start_stage_vars_;

  // Stored state msg
  nav_msgs::Odometry last_state_msg_;

  // Not available in derived classes
 private:
  // Modulehandler pointer to interface with real-time data
  ModuleHandler *ptr_module_handler_;

  // ROS publishers and subscribers
  ros::Publisher start_mpc_layer_pub_, ref_pub_, objective_reached_pub_,
      pmpc_failure_pub_;
  ros::Subscriber state_sub_, start_mpc_layer_sub_, ref_sub_,
      objective_reached_sub_, pmpc_failure_sub_;

  // ROS messages
  mpc_msgs::StartMpcLayer start_mpc_layer_msg_;
  mpc_msgs::ReferenceTrajectoryConstraints ref_msg_;

  // ROS recording publishers and messages
  ros::Publisher cur_state_rec_pub_, pred_traj_rec_pub_, slack_rec_pub_;
  mpc_msgs::Float64Array cur_state_msg_;
  mpc_msgs::TrajectoryOverHorizon pred_traj_msg_;
  mpc_msgs::ScalarOverHorizon slack_msg_;

  // Steady-state input
  Eigen::Vector4d u_steady_state_;

  // For warm starting the planner with current position to make the start
  // feasible
  bool warm_start_ = true;

  // Determine if input rates are used and corresponding amount of actual system
  // states
  bool use_input_rates_ = false;
  int nx_;

  // Determine if slack is used and corresponding amount of actual system inputs
  bool use_slack_ = false;
  int slack_idx_;
  int nu_;

  // Constraints tightening parameters
  bool use_tightened_obstacle_constraints_ = false;
  double c_o_ = 0, alpha_ = 0;
  std::vector<std::vector<double>> Q_, R_;
  std::vector<std::vector<double>> P_delta_;

  // Indicator variables for objective reached and emergency
  bool objective_reached_ = false;
  bool emergency_ = false;

  // RK4 integration
  HovergamesSystem<rk4_state_type_> rk4_sys_;
  Helpers::RK4::Integrator<rk4_state_type_, HovergamesSystem<rk4_state_type_>>
      rk4_integrator_;
  double integrator_stepsize_ = 0.05, integrator_steps_ = 1;

  // HMPC
  bool schedule_planner_ = true, reset_planner_ = true;  // TMPC-specific
  int tmpc_count_ = 0;                                   // PMPC-specific
  // HMPC timing variables
  int ratio_;  // ratio between PMPC and TMPC sampling times, called beta in
               // paper
  int n_ref_;  // amount of steps to forward-simulate lower-layer MPC model for
               // constructing reference trajectory

  // HMPC data communication variables
  int n_pmpc_stages_to_send_, n_pmpc_stages_to_receive_ = 0;

  // HMPC failure variables
  bool updated_ref_received_ =
      false;  // indicator whether a new reference has been received in time
  int ref_stage_idx_start_ = 0;  // number of stages to shift last received
                                 // reference to track in current run

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
  Eigen::MatrixXd stored_inputs_, stored_states_, stored_constraints_;
  Eigen::VectorXi stored_n_relevant_constraints_;

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
};

#endif  // HOVERGAMES_DRONE_INTERFACE_H
