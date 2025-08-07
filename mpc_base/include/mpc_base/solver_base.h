#ifndef MPC_BASE_SOLVER_BASE
#define MPC_BASE_SOLVER_BASE

#include <Eigen/Core>
#include <string>
#include <vector>

class SolverBase {
 public:
  // Integrator step size
  const double dt_;

  // Horizon length
  const int n_;
  const int nbar_;  // ForcesPro needs one more stage to implement the usual 0-N
                    // MPC structure

  // Stage variables
  const int ninit_;  // Number of variables that are equality-constrained in
                     // stage 0 (most of the times equal to nx)
  const int nu_;
  const int nx_;
  const int n_stage_vars_;
  std::vector<std::string> stage_vars_names_;
  // At the start of each loop, after actuating, last_updated_stage_vars_ is
  // loaded into the stage_vars_ structure, this to start each loop with the
  // closest measured state. So this structure is the output from the solver but
  // then again also the input for the next step
  Eigen::MatrixXd stage_vars_;
  Eigen::VectorXd last_updated_stage_vars_;

  // Constants
  std::vector<std::string> constants_names_;
  std::vector<int> constants_ndims_;
  std::vector<int> constants_sizes_;
  std::vector<int> constants_start_idc_;
  Eigen::VectorXd constants_;

  // Parameters
  const int npar_;
  std::vector<std::string> pars_names_;
  const int n_par_objectives_;
  std::vector<std::string> par_objectives_names_;
  Eigen::MatrixXd par_objectives_;
  const int n_par_weights_;
  std::vector<std::string> par_weights_names_;
  Eigen::VectorXd par_weights_;
  const int n_discs_;
  const int n_par_constraints_;
  std::vector<std::string> par_constraints_names_;
  Eigen::MatrixXd par_constraints_;

  // Modules
  std::vector<std::string> modules_names_;

  // Settings for debug printing
  bool print_solver_info_ = false;
  bool print_solver_param_ = true;
  bool print_solver_output_ = false;
  bool print_solver_data_ = false;

  SolverBase(double dt, int n, int n_init, int nu, int nx,
             std::vector<std::string> stage_vars_names,
             std::vector<std::string> constants_names,
             std::vector<int> constants_ndims, std::vector<int> constants_sizes,
             std::vector<int> constants_start_idc,
             std::vector<double> constants, int n_par,
             std::vector<std::string> pars_names, int n_par_objectives,
             std::vector<std::string> par_objectives_names, int n_par_weights,
             std::vector<std::string> par_weights_names, int n_discs,
             int n_par_constraints,
             std::vector<std::string> par_constraints_names,
             std::vector<std::string> modules_names)
      : dt_(dt),
        n_(n),
        nbar_(n + 1),
        ninit_(n_init),
        nu_(nu),
        nx_(nx),
        n_stage_vars_(nx + nu),
        stage_vars_names_(stage_vars_names),
        stage_vars_(Eigen::MatrixXd::Zero(nu + nx, n + 1)),
        last_updated_stage_vars_(Eigen::VectorXd::Zero(nu + nx)),
        constants_names_(constants_names),
        constants_ndims_(constants_ndims),
        constants_sizes_(constants_sizes),
        constants_start_idc_(constants_start_idc),
        constants_(
            Eigen::Map<Eigen::VectorXd>(constants.data(), constants.size())),
        npar_(n_par),
        pars_names_(pars_names),
        n_par_objectives_(n_par_objectives),
        par_objectives_names_(par_objectives_names),
        par_objectives_(Eigen::MatrixXd::Zero(n_par_objectives, n + 1)),
        n_par_weights_(n_par_weights),
        par_weights_names_(par_weights_names),
        par_weights_(Eigen::VectorXd::Zero(n_par_weights)),
        n_discs_(n_discs),
        n_par_constraints_(n_par_constraints),
        par_constraints_names_(par_constraints_names),
        par_constraints_(Eigen::MatrixXd::Zero(n_par_constraints_, n + 1)),
        modules_names_(modules_names) {};

  virtual ~SolverBase() {};

  // Templated functions cannot be virtual, so need to define the type of
  // constant beforehand and perform function overloading (compiler needs to
  // know what the type is during, but type in case of virtual is determined
  // only during runtime)
  virtual bool getConstantScalarBool(std::string constant_name) = 0;
  virtual int getConstantScalarInt(std::string constant_name) = 0;
  virtual double getConstantScalarDouble(std::string constant_name) = 0;
  virtual Eigen::VectorXd getConstantVectorDouble(
      std::string constant_name) = 0;
  virtual Eigen::MatrixXd getConstantMatrixDouble(
      std::string constant_name) = 0;

  virtual std::vector<int> returnDataPositionsState(
      std::vector<std::string> find_names) = 0;

  virtual std::vector<int> returnDataPositionsObjective(
      std::vector<std::string> find_names) = 0;

  virtual std::vector<int> returnDataPositionsConstrain(
      std::vector<std::string> find_names) = 0;

  virtual void loadStatesMatrixInSolver() = 0;

  virtual void updateMeasuredState() = 0;

  virtual void updateMeasuredStateInHorizon() = 0;

  virtual void shiftHorizonInMatrix(int time_shift) = 0;

  virtual void loadObjectiveParams() = 0;

  virtual void loadWeightsParams() = 0;

  virtual void loadConstrainParams() = 0;

  virtual void loadAllParameters() = 0;

  virtual void resetSolver() = 0;

  virtual void insertPredictedTrajectoryInMatrix() = 0;

  virtual int runSolverStep() = 0;

  virtual void printSolverData() = 0;
};

#endif