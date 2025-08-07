#ifndef MPC_MODULES_CONSTRAINTS_STATIC_POLYHEDRON_CONSTRAINTS_H
#define MPC_MODULES_CONSTRAINTS_STATIC_POLYHEDRON_CONSTRAINTS_H

#include <decomp_util/decomp_basis/data_type.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>
#include <decomp_util/decomp_geometry/polyhedron.h>
#include <decomp_util/ellipsoid_decomp.h>

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

// Dont move opencv include above decomp packages, this will give a typedef
// error!
#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>
#include <mpc_modules/types/module.h>
#include <mpc_modules/types/realtime_data.h>
#include <mpc_msgs/ConstraintsOverHorizon.h>
#include <mpc_msgs/MpcHeader.h>
#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/ros_visuals.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#define FLOAT_TOL 1e-8

enum class StaticPolyhedronConstraintsStatus {
  RESET = 0,
  BUSY,
  SUCCESS,
  FAILURE
};

/**
 * @brief Class for using static polyhedron obstacle avoidance.
 * Constraints are based on a convex decomposition of halfspace constraints
 * given for the current vehicle position with respect to each occupied cell in
 * the occupancy grid. See https://github.com/sikang/DecompUtil for more
 * information.
 */

class StaticPolyhedronConstraints : public ControllerModule {
 public:
  StaticPolyhedronConstraints(std::string name, ros::NodeHandle &nh,
                              ConfigurationMPC *ptr_config,
                              SolverBase *ptr_solver,
                              DataSaverJson *ptr_data_saver_json,
                              RealTimeData &data);

 protected:
  // Controller module methods
  void OnDataReceived(RealTimeData &data, std::string data_name) override;
  bool ReadyForControl(const RealTimeData &data) override;
  void Update(RealTimeData &data) override;
  void TightenConstraints(Eigen::VectorXd tightening_pred) override;
  void SetParameters(const RealTimeData &data) override;
  void SetConstraintsToRecAndVis(
      const Eigen::MatrixXd &constraints,
      const Eigen::VectorXi &n_relevant_constraints) override;
  void PublishData(const mpc_msgs::MpcHeader &mpc_header) override;
  void ExportDataJson(const int count_since_start,
                      const int count_total) override;
  void CreateVisualizations() override;
  void PublishVisualizations() override;
  void OnReset() override;

  // Methods specific to static polyhedron constraints module
  void OccGridMapCallBack(nav_msgs::OccupancyGrid::ConstPtr occ_grid_map_ptr);
  void getLocalOccupiedCells(nav_msgs::OccupancyGrid::ConstPtr occ_grid_map_ptr,
                             vec_Vec2f path);
  void CreateVisualizations(const Eigen::MatrixXd &constraints,
                            const Eigen::VectorXi &n_relevant_constraints,
                            int max_n_stages);
  void ComputeVertices(const Eigen::VectorXd &constraints);

  // Old functions for DECOMP_OLD definition
  bool GetObstacleContours(const nav_msgs::OccupancyGrid &occ_grid_map,
                           std::vector<std::vector<cv::Point>> &contours);
  bool GetOccupiedGridCells(const nav_msgs::OccupancyGrid &occ_grid_map);

 private:
  /**
   * @brief Compute the line segments using predicted trajectory from previous
   * solver run
   *
   * @param solver_data_state Matrix containing all the state data in the solver
   * @return true
   * @return false
   */
  bool ComputeLineSegments();

  /**
   * @brief Constraint generation function for all steps in the prediction
   * horizon
   *
   * @return true
   * @return false
   */
  bool ComputeConstraints();

  // Internal variables needed for calculations
  StaticPolyhedronConstraintsStatus status_;

  // ROS publishers and subscribers
  ros::Subscriber local_map_sub_;
  ros::Publisher local_occ_grid_pub_;

  // ROS recording publishers and messages
  ros::Publisher constraints_rec_pub_;
  mpc_msgs::ConstraintsOverHorizon constraints_msg_;

  // We get the solver data as matrix or vector, and need to identify which
  // positions we need
  int x_position_;
  int y_position_;
  int psi_position_;
  int disc0_0_a1_position_;

  // State callback
  nav_msgs::Path pred_traj_;
  bool line_segments_updated_;

  // Old map
  std::vector<std::vector<cv::Point>> contours_;

  // Map processing
  int occupied_threshold_;
  vec_Vec2f occ_pos_;
  bool map_processed_;
  std::vector<std::unique_ptr<vec_Vec2f>> occ_pos_vec_stages_;
  std::vector<double> occ_test_;

  nav_msgs::OccupancyGrid::ConstPtr occ_grid_map_ptr_;
  double radius_around_bbox_;

  // Ellipsoidal decomposition
  vec_Vec2f path_;
  size_t n_path_;
  EllipsoidDecomp2D decomp_util_;
  std::vector<LinearConstraint<2>>
      poly_constraints_;  // Static 2D halfspace constraints set in DecompUtil

  // Constraints variables
  // NOTE: Eigen stores in column-major order, so stack
  // 3*n_constraints_per_stage_ values per column, 1+N columns in total to first
  // iterate through values of constraints per stage, then through stages
  bool first_constraints_constructed_ = false;
  int n_stages_to_construct_, n_constraints_per_stage_;
  Eigen::MatrixXd
      constraints_;  // 3*n_constraints_per_stage_ x 1+N (stage 0 comes from the
                     // previous run and is not used in this run)
  Eigen::VectorXi
      n_relevant_constraints_;  // Number of relevant constraints per stage

  // HMPC variables
  bool first_constraints_received_ = false;
  int received_n_stages_ = 0;
  Eigen::MatrixXd received_constraints_;
  Eigen::VectorXi received_n_relevant_constraints_;
  int n_it_valid_ = 0, n_it_valid_max_ = 0;

  // ROS recording and visualization variables
  // NOTE: the size of these constraints depends on the type of MPC and what the
  // system interface indicates, so need to be able to resize during runtime
  bool use_ext_con_to_rec_and_vis_ = false;
  int n_stages_to_rec_and_vis_;
  Eigen::MatrixXd constraints_to_rec_and_vis_;
  Eigen::VectorXi n_relevant_constraints_to_rec_and_vis_;
  std::vector<Eigen::Vector2d> vertices_to_vis_;
  std::unique_ptr<ROSMarkerPublisher> ros_markers_;
};

#endif
