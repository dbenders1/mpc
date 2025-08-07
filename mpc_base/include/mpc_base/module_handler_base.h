#ifndef MPC_BASE_MODULE_HANDLER_BASE
#define MPC_BASE_MODULE_HANDLER_BASE

// Forward-declarations (see https://cplusplus.com/articles/Gw6AC542/)
class string;
class vector;
class Vector3d;

#include <mpc_msgs/MpcHeader.h>

#include <Eigen/Dense>

class ModuleHandlerBase {
 public:
  ModuleHandlerBase() {};
  virtual ~ModuleHandlerBase() {};
  virtual void onDataReceivedObjective(std::string name) = 0;
  virtual void onDataReceivedConstraint(std::string name) = 0;
  virtual void onDataReceived(std::string name) = 0;
  virtual void updateModules() = 0;
  virtual void setParametersObjective() = 0;
  virtual void setParametersConstraint() = 0;
  virtual void setConstraintsToRecAndVis(
      std::string module_name, const Eigen::MatrixXd &constraints,
      const Eigen::VectorXi &n_relevant_constraints) = 0;
  virtual void publishData(const mpc_msgs::MpcHeader &mpc_header) = 0;
  virtual void exportDataJson(int count_since_start, int count_total) = 0;
  virtual void onReset() = 0;
  virtual bool checkModulesReady() = 0;
  virtual bool checkObjectiveReached() = 0;
  virtual void tightenConstraints(std::string module_name,
                                  Eigen::VectorXd tightening_pred) = 0;
  virtual void createVisualizationsModules() = 0;
  virtual void publishVisualizationsModules() = 0;
  virtual std::string getMethodName() = 0;
  virtual void printRealTimeData() = 0;
};

#endif
