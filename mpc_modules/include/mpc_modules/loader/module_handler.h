#ifndef MPC_MODULE_LOADER_MODULE_LOADER_H
#define MPC_MODULE_LOADER_MODULE_LOADER_H

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/module_handler_base.h>
#include <mpc_base/solver_base.h>
#include <mpc_modules/types/module.h>
#include <mpc_modules/types/realtime_data.h>
#include <mpc_tools/data_saver_json.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

// Forward decleration of classes. This can be done if we don't exactly use the
// class itself in this file. Therefore we can exclude a header include
// completely or move it to the cpp file.
enum class ModuleType;

class ModuleHandler : public ModuleHandlerBase {
 public:
  // All empty pointers to module base
  std::vector<std::unique_ptr<ControllerModule>> modules_pointers_;
  SolverBase *ptr_solver_;
  DataSaverJson *ptr_data_saver_json_;

  // Names of the modules that are loaded in
  std::vector<std::string> modules_names_;

  // Data between modules
  RealTimeData data_;

  /**
   * @brief This is the main module loader. The input is received from the
   * solver package which contains the names of the modules that we are going to
   * use.
   *
   * @param nh
   * @param config
   * @param vehicle
   * @param module_names module names received from solver package
   */
  ModuleHandler(ros::NodeHandle &nh, ConfigurationMPC *ptr_config,
                SolverBase *ptr_solver, DataSaverJson *ptr_data_saver_json);

  /**
   * @brief Update the objective modules
   *
   * @param module_type specify if we want to update constrain or objective
   * @param name name for the data
   */
  void onDataReceivedObjective(std::string name) override;

  /**
   * @brief Update the constraint modules
   *
   * @param module_type specify if we want to update constrain or objective
   * @param name name for the data
   */
  void onDataReceivedConstraint(std::string name) override;

  /**
   * @brief Update all modules
   *
   * @param name name for the data
   */
  void onDataReceived(std::string name) override;

  /**
   * @brief update the inequalities parameter
   *
   */
  void updateModules() override;

  /**
   * @brief Set the parameters of the objective modules
   *
   */
  void setParametersObjective() override;

  /**
   * @brief Set the parameters of the constraint modules
   *
   */
  void setParametersConstraint() override;

  /**
   * @brief Publish data from the modules
   *
   */
  void publishData(const mpc_msgs::MpcHeader &mpc_header) override;

  /**
   * @brief Saving data from the modules
   *
   * @param data_saver_json data object for saving
   * @param count_since_start counter since start
   * @param count_total total count
   */
  void exportDataJson(int count_since_start, int count_total) override;

  /**
   * @brief reset all the modules
   *
   */
  void onReset() override;

  /**
   * @brief check in all the modules if they are ready for control
   *
   * @return if modules are ready
   */
  bool checkModulesReady() override;

  /**
   * @brief check in all the modules if the objective is reached
   *
   * @return if objective is reached
   */
  bool checkObjectiveReached() override;

  /**
   * @brief Tighten the constraints using predicted tightening over horizon in
   * constraints module with name module_name
   */
  void tightenConstraints(std::string module_name,
                          Eigen::VectorXd tightening_pred) override;

  /**
   * @brief Set custom constraints via system interface in the constraints
   * module with name module_name
   *
   * @param module_name name of the module
   * @param constraints constraints to set
   * @param n_relevant_constraints number of relevant constraints
   */
  void setConstraintsToRecAndVis(
      std::string module_name, const Eigen::MatrixXd &constraints,
      const Eigen::VectorXi &n_relevant_constraints) override;

  /**
   * @brief Create visualizations for the modules
   */
  void createVisualizationsModules() override;

  /**
   * @brief Publish visualizations for the modules
   */
  void publishVisualizationsModules() override;

  /**
   * @brief Return the modulenames as one string
   *
   */
  std::string getMethodName() override;

  /**
   * @brief
   *
   */
  void printRealTimeData() override;
};

#endif
