#ifndef MPC_BASE_INTERFACE_BASE
#define MPC_BASE_INTERFACE_BASE

#include <Eigen/Core>
#include <string>
#include <vector>

/**
 * @brief This class defines the functions that are called from the controller
 * class. These should be implemented for ALL interfaces.
 *
 */

class InterfaceBase {
 public:
  InterfaceBase() {};

  virtual ~InterfaceBase() {};

  /**
   * @brief Interface computations at start of the control loop
   *
   */
  virtual void AtControlLoopStart() = 0;

  /**
   * @brief Interface computations after module updates (e.g., for applying
   * interface-specific constraint tightening)
   */
  virtual void AfterUpdateModules() = 0;

  /**
   * @brief Interface computations after the module parameters are set
   */
  virtual void AfterSetParametersModules() = 0;

  /**
   * @brief Interface computations after the optimization
   */
  virtual void AfterOptimization() = 0;

  /**
   * @brief Compute the control command
   */
  virtual void ComputeActuation() = 0;

  /**
   * @brief Apply the computed control command
   */
  virtual void Actuate() = 0;

  /**
   * @brief Deploy interface-specific objective reached strategy
   */
  virtual void DeployObjectiveReachedStrategy() = 0;

  /**
   * @brief Deploy interface-specific emergency strategy
   */
  virtual void DeployEmergencyStrategy() = 0;

  /**
   * @brief Define actions to take when the controller class is resetting
   */
  virtual void Reset() = 0;

  /**
   * @brief Create visualizations for the module
   */
  virtual void CreateVisualizations() = 0;

  /**
   * @brief Publish visualizations for the module
   */
  virtual void PublishVisualizations() = 0;

  /**
   * @brief Add static data to the JSON file
   *
   */
  virtual void AddStaticData() = 0;

  virtual bool setInterval(const double dt) = 0;

  virtual bool setLastCommand(const Eigen::VectorXd& last_command) = 0;

  virtual bool setState(const double t, const Eigen::VectorXd& state) = 0;

  virtual bool run() = 0;

  virtual Eigen::VectorXd getCommand() = 0;
};

#endif
