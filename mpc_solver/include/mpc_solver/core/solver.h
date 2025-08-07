#ifndef MPC_SOLVER_CORE_SOLVER_H
#define MPC_SOLVER_CORE_SOLVER_H

#include <mpc_base/solver_base.h>
#include <mpc_solver/core/helper_functions.h>

#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

namespace mpc_solver {

template <class TParam, class TOutput, class TInfo, class TMem, class TExtfunc,
          class TFile, class TInt>
class Solver : public SolverBase {
 public:
  // Solver structs
  TParam forces_params_;
  TOutput forces_output_;
  TInfo forces_info_;

  // Stored Solver structs
  TParam last_forces_params_;
  TOutput last_forces_output_;
  TInfo last_info_;

  // Initial Solver Params for reset
  TParam initial_forces_params_;

  // Solver memory allocation
  char *solver_memory_;
  TMem *solver_memory_handle_;
  TExtfunc solver_extfunc_;
  TMem *mem_;

  // Function pointer to forces solver
  TInt (*solver_solve_func_)(TParam *, TOutput *, TInfo *, TMem *, TFile *,
                             TExtfunc);

  Solver(double dt, int n, int n_init, int nu, int nx,
         std::vector<std::string> stage_vars_names,
         std::vector<std::string> constants_names,
         std::vector<int> constants_ndims, std::vector<int> constants_sizes,
         std::vector<int> constants_start_idc, std::vector<double> constants,
         int n_par, std::vector<std::string> pars_names, int n_par_objectives,
         std::vector<std::string> par_objectives_names, int n_par_weights,
         std::vector<std::string> par_weights_names, int n_discs,
         int n_par_constraints, std::vector<std::string> par_constraints_names,
         std::vector<std::string> modules_names, int solver_number,
         TMem *(*solver_memory_handle)(void *, unsigned int, size_t),
         size_t (*solver_get_memory_size)(), TExtfunc solver_extfunc,
         TInt (*solver_solve_func)(TParam *, TOutput *, TInfo *, TMem *,
                                   TFile *, TExtfunc))
      : SolverBase(dt, n, n_init, nu, nx, stage_vars_names, constants_names,
                   constants_ndims, constants_sizes, constants_start_idc,
                   constants, n_par, pars_names, n_par_objectives,
                   par_objectives_names, n_par_weights, par_weights_names,
                   n_discs, n_par_constraints, par_constraints_names,
                   modules_names),
        solver_memory_((char *)malloc(solver_get_memory_size())),
        solver_memory_handle_(
            solver_memory_handle((char *)malloc(solver_get_memory_size()),
                                 solver_number, solver_get_memory_size())),
        solver_extfunc_(solver_extfunc),
        solver_solve_func_(solver_solve_func) {};

  ~Solver() {
    // free(solver_memory_);
    // free(solver_memory_handle_);
  }

  /**
   * @brief Function for returning the boolean value of constant with name
   * constant_name
   *
   * @note Exit the code if the constant is not found
   *
   * @param constant_name
   * @return true
   * @return false
   */
  bool getConstantScalarBool(std::string constant_name) override {
    int constant_idx = returnDataPosition(constant_name, constants_names_);
    if (constant_idx == -1) {
      std::cerr << "\033[31m[MPC solver] Constant with name " << constant_name
                << " not found! Exiting. \033[0m" << std::endl;
      exit(1);
    }
    return constants_[constants_start_idc_[constant_idx]];
  }

  /**
   * @brief Function for returning the integer value of constant with name
   * constant_name
   *
   * @note Exit the code if the constant is not found
   *
   * @param constant_name
   * @return int
   */
  int getConstantScalarInt(std::string constant_name) override {
    int constant_idx = returnDataPosition(constant_name, constants_names_);
    if (constant_idx == -1) {
      std::cerr << "\033[31m[MPC solver] Constant with name " << constant_name
                << " not found! Exiting. \033[0m" << std::endl;
      exit(1);
    }
    return constants_[constants_start_idc_[constant_idx]];
  }

  /**
   * @brief Function for returning the double value of constant with name
   * constant_name
   *
   * @note Exit the code if the constant is not found
   *
   * @param constant_name
   * @return double
   */
  double getConstantScalarDouble(std::string constant_name) override {
    int constant_idx = returnDataPosition(constant_name, constants_names_);
    if (constant_idx == -1) {
      std::cerr << "\033[31m[MPC solver] Constant with name " << constant_name
                << " not found! Exiting. \033[0m" << std::endl;
      exit(1);
    }
    return constants_[constants_start_idc_[constant_idx]];
  }

  /**
   * @brief Function for returning an Eigen::VectorXd with values of constant
   * with name constant_name
   *
   * @param constant_name
   * @return Eigen::VectorXd
   */
  Eigen::VectorXd getConstantVectorDouble(std::string constant_name) override {
    // Find index of constant in constants_names_
    int constant_idx = returnDataPosition(constant_name, constants_names_);

    if (constant_idx == -1) {
      std::cerr << "\033[31m[MPC solver] Constant with name " << constant_name
                << " not found! Exiting. \033[0m" << std::endl;
      exit(1);
    }

    // Find the corresponding index in sizes
    int size_idx = 0;
    for (int dim_idx = 0; dim_idx < constant_idx; dim_idx++) {
      if (constants_ndims_[dim_idx] == 0 || constants_ndims_[dim_idx] == 1) {
        size_idx += 1;
      } else {
        size_idx += 2;
      }
    }

    // Return Eigen::VectorXd with specific length
    return constants_.segment(constants_start_idc_[constant_idx],
                              constants_sizes_[size_idx]);
  }

  /**
   * @brief Function for returning an Eigen::MatrixXd with values of constant
   * with name constant_name
   *
   * @param constant_name
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd getConstantMatrixDouble(std::string constant_name) override {
    // Find index of constant in constants_names_
    int constant_idx = returnDataPosition(constant_name, constants_names_);

    if (constant_idx == -1) {
      std::cerr << "\033[31m[MPC solver] Constant with name " << constant_name
                << " not found! Exiting. \033[0m" << std::endl;
      exit(1);
    }

    // Find number of rows and columns
    int size_idx = 0;
    for (int dim_idx = 0; dim_idx < constant_idx; dim_idx++) {
      if (constants_ndims_[dim_idx] == 0 || constants_ndims_[dim_idx] == 1) {
        size_idx += 1;
      } else {
        size_idx += 2;
      }
    }
    int n_rows = constants_sizes_[size_idx];
    int n_cols = constants_sizes_[size_idx + 1];

    // Return Eigen::MatrixXd with corresponding size
    return Eigen::Map<Eigen::MatrixXd>(
        &constants_.segment(constants_start_idc_[constant_idx], n_rows * n_cols)
             .data()[0],
        n_rows, n_cols);
  }

  /**
   * @brief Function for returning the indexes (row) of the names attached to
   * the solver state input data
   *
   * @param find_names these are the names that we have to find in the stored
   * names
   * @return indexes of the names in order of the given vector
   */
  std::vector<int> returnDataPositionsState(
      std::vector<std::string> find_names) override {
    return returnDataPositions(find_names, stage_vars_names_);
  }

  /**
   * @brief Function for returning the indexes (row) of the names attached to
   * the solver objective input data
   *
   * @param find_names these are the names that we have to find in the stored
   * names
   * @return indexes of the names in order of the given vector
   */
  std::vector<int> returnDataPositionsObjective(
      std::vector<std::string> find_names) override {
    return returnDataPositions(find_names, par_objectives_names_);
  }

  /**
   * @brief Function for returning the indexes (row) of the names attached to
   * the solver state input data
   *
   * @param find_names these are the names that we have to find in the stored
   * names
   * @return indexes of the names in order of the given vector
   */
  std::vector<int> returnDataPositionsConstrain(
      std::vector<std::string> find_names) override {
    return returnDataPositions(find_names, par_constraints_names_);
  }

  void loadStatesMatrixInSolver() override {
    double *ptr_x0 = (double *)&forces_params_.x0;
    int i = 0;
    int j = 0;
    for (double *ptr = ptr_x0; ptr < ptr_x0 + n_stage_vars_ * nbar_; ptr++) {
      *ptr = stage_vars_(i, j);
      i++;

      if (i >= nu_ + nx_) {
        i = 0;
        j++;
      }
    }

    double *ptr_xinit = (double *)&forces_params_.xinit;
    i = nu_ + nx_ - ninit_;
    for (double *ptr = ptr_xinit; ptr < ptr_xinit + ninit_; ptr++) {
      *ptr = stage_vars_(i++, 0);
    };
  }

  /**
   * @brief Function for loading the measured state into the Eigen states matrix
   */
  void updateMeasuredState() override {
    stage_vars_.col(0) = last_updated_stage_vars_;
  }

  /**
   * @brief Function for loading the measured state into the Eigen states matrix
   * into the full horizon
   */
  void updateMeasuredStateInHorizon() override {
    for (int i = 0; i < nbar_; i++) {
      stage_vars_.col(i) = last_updated_stage_vars_;
    }
  }

  void shiftHorizonInMatrix(int time_shift) override {
    int current_col = time_shift;

    for (int i = 0; i < n_; i++) {
      stage_vars_.col(i) = stage_vars_.col(current_col);

      if (!(current_col == nbar_ - 1)) {
        current_col++;
      }
    }
  }

  /**
   * @brief Function for loading in the objective parameters
   */
  void loadObjectiveParams() override {
    double *ptr_start = (double *)&forces_params_.all_parameters;
    int size = npar_ * nbar_;
    int i = 0;
    int j = 0;
    for (double *ptr = ptr_start; ptr < ptr_start + size; ptr++) {
      *ptr = par_objectives_(i, j);
      i++;

      if (i >= n_par_objectives_) {
        ptr = ptr + n_par_constraints_ + n_par_weights_;
        i = 0;
        j++;
      }
    }
  }

  /**
   * @brief Function for loading the weights into solver struct into all
   * timesteps. This only needs to be done initially or when the weights change
   * through reconfigure callback.
   */
  void loadWeightsParams() override {
    double *ptr_start = (double *)&forces_params_.all_parameters;
    int size = npar_ * nbar_;
    int i = 0;
    for (double *ptr = ptr_start + n_par_objectives_; ptr < ptr_start + size;
         ptr++) {
      *ptr = par_weights_(i);
      i++;

      if (i >= n_par_weights_) {
        ptr = ptr + n_par_constraints_ + n_par_objectives_;
        i = 0;
      }
    }
  }

  /**
   * @brief Function for loading in the constrain parameters
   */
  void loadConstrainParams() override {
    double *ptr_start = (double *)&forces_params_.all_parameters;
    int size = npar_ * nbar_;
    int i = 0;
    int j = 0;

    for (double *ptr = ptr_start + n_par_objectives_ + n_par_weights_;
         ptr < ptr_start + size; ptr++) {
      *ptr = par_constraints_(i, j);
      i++;

      if (i >= n_par_constraints_) {
        ptr = ptr + n_par_objectives_ + n_par_weights_;
        i = 0;
        j++;
      }
    }
  }

  /**
   * @brief Function for loading all parameters, this could be used if parameter
   * structure is stored for a warm start for the solver. This should not be
   * used as the main way to update the parameters
   */
  void loadAllParameters() override {
    loadObjectiveParams();
    loadWeightsParams();
    loadConstrainParams();
  }

  /**
   * @brief Reset all parameters in the forces parameter list to zero (input to
   * the solver)
   */
  void resetSolver() override {
    double *pointer = (double *)&forces_params_;
    int size = (npar_ + n_stage_vars_ + ninit_) * nbar_;
    for (double *ptr = pointer; ptr < pointer + size; ptr++) {
      *ptr = 0;
    }
  }

  /**
   * @brief Copy all the solver output to the parameter list (input to the
   * solver) and store the solver output into a matrix that can be used in the
   * main MPC
   *
   * @param timeshift The timestep in the output that is going to be loaded as
   * the first timestep for the new optimization.
   * @param solver_output storage for the solver output
   */
  void insertPredictedTrajectoryInMatrix() {
    // size of the in the forces_output_ struct
    int size = n_stage_vars_ * nbar_;
    // Pointer to the beginning of the forces_output_
    double *ptr_output = (double *)&forces_output_;

    int i = 0;
    int j = 0;
    for (double *ptr = ptr_output; ptr < ptr_output + size; ptr++) {
      stage_vars_(i, j) = *ptr;
      i++;

      if (i >= nu_ + nx_) {
        i = 0;
        j++;
      }
    }
  }

  /**
   * @brief run the solver for one timestep and if the solver was succesfull,
   * store the previous
   *
   * @param time_shift The timeshift we use to put the output states into the
   * next iteration
   * @return int, exit code of the solver
   */
  int runSolverStep() override {
    loadObjectiveParams();
    loadConstrainParams();
    loadStatesMatrixInSolver();

    TInfo temp_info_ = forces_info_;
    TOutput temp_output = forces_output_;

    if (print_solver_data_ && print_solver_param_) {
      printSolverParam();
    };

    int exit_code =
        solver_solve_func_(&forces_params_, &forces_output_, &forces_info_,
                           solver_memory_handle_, stdout, solver_extfunc_);

    if (print_solver_data_ && print_solver_output_) {
      printSolverOutput();
    };
    if (print_solver_data_ && print_solver_info_) {
      printSolverInfo();
    };

    // Return if we did not get the correct exit code
    if (exit_code != 1) {
      return exit_code;
    }

    last_forces_params_ = forces_params_;
    last_info_ = temp_info_;
    last_forces_output_ = temp_output;

    insertPredictedTrajectoryInMatrix();

    return exit_code;
  }

  /**
   * @brief print all the parameter details from the solver_params struct
   */
  void printSolverParam() {
    // Names of the struct sections and sizes
    std::vector<std::string> sections = {"x0", "xinit", "Parameters"};
    std::vector<int> sizes = {n_stage_vars_, nx_, npar_, nu_, ninit_};
    double *ptr_solver_x0 = (double *)&forces_params_.x0;
    double *ptr_solver_xinit = (double *)&forces_params_.xinit;
    double *ptr_solver_all_parameters =
        (double *)&forces_params_.all_parameters;
    printTableParams(12, ptr_solver_x0, ptr_solver_xinit,
                     ptr_solver_all_parameters, sizes, nbar_, 5, sections,
                     stage_vars_names_, pars_names_);
  }

  /**
   * @brief print all the output values of the solver_output struct
   */
  void printSolverOutput() {
    // Names of the struct sections and sizes
    std::vector<std::string> sections = {"Inputs", "States"};
    std::vector<int> sizes = {nu_, nx_};
    double *ptr_solver_struct = (double *)&forces_output_;
    printTableOutput(12, ptr_solver_struct, sizes, nbar_, 5, sections,
                     stage_vars_names_);
  }

  /**
   * @brief print all the solver information of the solver_info struct
   */
  void printSolverInfo() {
    std::vector<std::string> solver_debug_params;
    solver_debug_params = {"it",      "it2opt",    "res_eq",    "res_ineq",
                           "rsnorm",  "rcompnorm", "pobj",      "dobj",
                           "dgap",    "rdgap",     "mu",        "mu_aff",
                           "sigma",   "lsit_aff",  "lsit_cc",   "step_aff",
                           "step_cc", "solvetime", "fevalstime"};
    int *ptr_solver_struct = (int *)&forces_info_;

    printTableInfo(30, solver_debug_params, ptr_solver_struct);
  }

  /**
   * @brief print all solver information
   */
  void printSolverData() {
    if (print_solver_param_) {
      printSolverParam();
    };
    if (print_solver_output_) {
      printSolverOutput();
    };
    if (print_solver_info_) {
      printSolverInfo();
    };
  }
};

}  // namespace mpc_solver

#endif