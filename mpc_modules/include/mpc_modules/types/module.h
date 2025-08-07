#ifndef MPC_MODULES_TYPES_MODULE_H
#define MPC_MODULES_TYPES_MODULE_H

// Forward-declarations (see https://cplusplus.com/articles/Gw6AC542/)
class MpcHeader;

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>
#include <mpc_modules/types/realtime_data.h>
#include <mpc_msgs/MpcHeader.h>
#include <mpc_tools/data_saver_json.h>
#include <ros/ros.h>

#include <memory>
#include <vector>

// To distinguish custom from regular optimization loops.
#define EXIT_CODE_NOT_OPTIMIZED_YET -999

enum class ModuleType { OBJECTIVE = 0, CONSTRAINT, UNDEFINED };

/**
 * @brief Purely virtual module of the controller to compute inequalities,
 * objective terms or anything else. The idea is that modules are defined in the
 * solver and seemingly integrate with the c++ code without having to adapt
 * parameters on either side. This should make the process of stacking different
 * MPC contributions more flexible.
 */
class ControllerModule {
 public:
  /**
   * @brief Construct a new Controller Module object. Note that controller
   * module initialization happens in the solver class itself based on the
   * python code.
   */
  ControllerModule(ModuleType type, std::string name, ros::NodeHandle &nh,
                   ConfigurationMPC *ptr_config, SolverBase *ptr_solver,
                   DataSaverJson *ptr_data_saver_json)
      : type_(type),
        name_(name),
        nh_(nh),
        ptr_config_(ptr_config),
        ptr_solver_(ptr_solver),
        ptr_data_saver_json_(ptr_data_saver_json) {}

  virtual ~ControllerModule() {};

 public:
  /** ====== MANDATORY FUNCTIONS (purely virtual) ==========*/

  /**
   * @brief Update the module (any computations that need to happen before
   * setting solver parameters)
   *
   * @param data For storing data
   */
  virtual void Update(RealTimeData &data) = 0;

  /**
   * @brief Insert the collision avoidance constraints
   *
   * @param data For storing data
   */
  virtual void SetParameters(const RealTimeData &data) = 0;

  /**
   * @brief Publish module data
   */
  virtual void PublishData(const mpc_msgs::MpcHeader &mpc_header) = 0;

  /**
   * @brief Create visualizations for the module
   */
  virtual void CreateVisualizations() = 0;

  /**
   * @brief Publish visualizations for the module
   */
  virtual void PublishVisualizations() = 0;
  /* ======================================================== */

  /** ====== OPTIONAL FUNCTIONS ==========*/

  /**
   * @brief Check if this module is ready for control
   *
   * @return true If this module can execute the necessary computations with the
   * current data
   * @return false Otherwise
   */
  virtual bool ReadyForControl(const RealTimeData &data) {
    return true;
  };  // Default: true

  /**
   * @brief Check if the objective of this module was reached
   *
   * @param data
   * @return true If the objective was reached
   */
  virtual bool ObjectiveReached(const RealTimeData &data) {
    return true;
  };  // Default: true

  /**
   * @brief Load the initial guess for the solver if desired
   *
   * @param custom_plan
   */
  // virtual void SetWarmStart(std::unique_ptr<std::vector<std::vector<double>>>
  // &custom_plan) {};

  /**
   * @brief Function used to update any class members when new data is received
   *
   * @param data All real-time data
   * @param data_name The name of the data that was updated (to decide if
   * anything needs to be updated)
   */
  virtual void OnDataReceived(RealTimeData &data, std::string data_name) {};

  /**
   * @brief Tighten the constraints using the predicted tightening over the
   * horizon
   *
   * @note This function is only used if the module is a constraint module
   */
  virtual void TightenConstraints(Eigen::VectorXd tightening_pred) {};

  /**
   * @brief Set custom constraints via system interface in the constraints
   * module
   *
   * @note This function is only used if the module is a constraint module
   *
   * @param constraints The constraints to visualize
   * @param n_relevant_constraints The number of relevant constraints
   */
  virtual void SetConstraintsToRecAndVis(
      const Eigen::MatrixXd &constraints,
      const Eigen::VectorXi &n_relevant_constraints) {};

  /**
   * @brief Reset any members if necessary
   */

  virtual void OnReset() {};

  /**
   * @brief Override to define a custom optimization loop. Note that there can
   * only be ONE customized optimization.
   *
   * @return int exit_code of the solver, return any exit_code other than
   * "EXIT_CODE_NOT_OPTIMIZED_YET" to define this as a custom optimization
   */
  // virtual int Optimize(SolverInterface *solver_interface) { return
  // EXIT_CODE_NOT_OPTIMIZED_YET; }; // Default: no custom optimization

  /**
   * @brief Export runtime data to store in JSON file
   *
   * @param data_saver_json The JSON data saver instance to save the data
   * @param count_since_start Control loop iteration count since start of the
   * current control run
   * @param count_total Control loop iteration count since start of node
   */
  virtual void ExportDataJson(const int count_since_start,
                              const int count_total) {};

  /** ================================== */

  ModuleType type_ = ModuleType::UNDEFINED; /* Constraint or Objective type */
  std::string name_;                        /* Module name */

 protected:
  ros::NodeHandle nh_;
  ConfigurationMPC *ptr_config_;       /* Configuration parameters */
  SolverBase *ptr_solver_;             /* Solver information */
  DataSaverJson *ptr_data_saver_json_; /* Data saver */
};

#endif
