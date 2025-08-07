import os
import shutil
import sys

# Now parent directory is added, we can find helpers.py script
from include import dynamics, helpers

# If your forces is in this directory add it
helpers.load_forces_path()

import forcespro.nlp
import numpy as np


# Lambda functions for objectives and constraints
# NOTE: Python cannot handle lambdas without an additional function that manually creates the lambda with the correct value
def objective_with_stage_index(k, settings):
    return lambda z, p: settings.use_objective(k, z, p, settings)


def inequality_constraints_with_stage_index(k, settings):
    return lambda z, p: settings.modules.inequality_constraints(k, z, p, settings)


def create_solver(Name, system, dir_path, settings, use_floating, floating_platform):
    print("--- Starting Model creation ---")

    """
        Important: Problem formulation
        - In general, we consider an MPC cost function of the following form:
        J_terminal(x(N|t)) + sum_{k=0}^{N-1} J_stage(x(k|t),u(k|t))
        with N stage cost terms and 1 terminal stage cost term.
        
        - The ForcesPro 6.0.0 documentation gives the following cost function with 1-based MATLAB indices:
        sum_{k=1}^{N_bar} J_stage(x(k|t),u(k|t))
        where k=1 indicates the initial state and first optimized input
        and N_bar indicates the horizon length given to the solver: solver.N
        
        - This translates to the following cost function with 0-based Python indices:
        sum_{k=0}^{N_bar-1} J_stage(x(k|t),u(k|t))
        where k=0 indicates the initial state and first optimized input
        and N_bar indicates the horizon length given to the solver: solver.N

        - This means we have to add one additional (terminal) stage at the end to comply with our original formulation
        => N_bar = settings.N + 1 
    """
    settings.N_bar = (
        settings.N + 1
    )  # Note: this relation is assumed in the constraints modules and parameters => updating means checking if they are still correct!

    # Print the model and modules
    print(settings.model)
    print(settings.modules)

    # Load model parameters from the settings
    solver = forcespro.nlp.SymbolicModel(settings.N_bar)
    solver.N = settings.N_bar  # prediction/planning horizon
    solver.nvar = settings.model.nvar  # number of online variables
    solver.neq = settings.model.nx  # number of equality constraints
    solver.npar = settings.npar

    # Static system constraints
    # Note: in case of robust MPC, these have to be replaced by constraints module TightenedSystemConstraints
    solver.lb = settings.model.lower_bound()
    solver.ub = settings.model.upper_bound()

    # Stage-wise objectives and constraints
    # Note that we use solver.N = N_bar here!
    for k in range(solver.N):
        # OBJECTIVE
        # Although optimizing the state in the first stage does not contribute to anything, it is fine.
        # We do not really have to optimize the final input, but it only effects the runtimes
        solver.objective[k] = objective_with_stage_index(k, settings)

        # CONSTRAINTS
        # Currently two types of constraints:
        # - Tightened system constraints: apply to stages k \in [0,N] ([0,N_bar-1])
        # - Collision avoidance constraints: apply to stages k \in [1,N] ([1,N_bar-1])
        solver.ineq[k] = inequality_constraints_with_stage_index(k, settings)
        solver.hl[k] = settings.modules.constraint_manager.lower_bound[k]
        solver.hu[k] = settings.modules.constraint_manager.upper_bound[k]
        solver.nh[k] = settings.nh[k]

        # PARAMETERS (includes objective parameters, weights, disc information and constraint parameters)
        # solver.npar[k] = settings.npar[k]

    # Dynamical constraints
    solver.eq = lambda z, p: settings.model.discretize_dynamics(z, p, settings)
    solver.E = np.concatenate(
        [np.zeros((settings.model.nx, settings.model.nu)), np.eye(settings.model.nx)],
        axis=1,
    )

    # What needs to be initialized at runtime
    if settings.initialize_input_runtime == False:
        solver.xinitidx = range(settings.model.nu, settings.model.nvar)
    elif settings.initialize_input_runtime == True:
        solver.xinitidx = range(settings.model.nvar)
    else:
        print("Error in setting initialized_runtime")

    # ==== Solver options ==== #
    options = forcespro.CodeOptions(Name + "FORCESNLPsolver")
    options.printlevel = settings.print_level
    options.optlevel = settings.optimization_level
    options.timing = 1
    options.overwrite = 1
    options.cleanup = 1

    ## Floating licenses settings
    if use_floating:
        options.embedded_timing = 1
        options.license.use_floating_license = 1
        if floating_platform == 2:
            options.platform = "AARCH-Cortex-A72"

    # options.init = 0 # Warm start?

    # -- PRIMAL DUAL INTERIOR POINT (Default Solver!) -- #
    options.maxit = settings.maximum_iterations
    options.mu0 = settings.mu0
    options.init = 1
    options.linesearch.factor_aff = 0.8
    options.linesearch.factor_cc = 0.85
    options.linesearch.minstep = 1e-7
    options.linesearch.maxstep = 0.95
    options.nlp.integrator.nodes = 4
    options.nlp.integrator.type = "ERK4"
    # options.parallel = 8

    print("--- Generating solver ---")

    # Define Original path and new path
    solver_path = dir_path + "/" + Name + "FORCESNLPsolver"
    print("Path of the solver: {}".format(solver_path))
    new_solver_path = (
        dir_path + "/include/mpc_solver/" + system + "/" + Name + "FORCESNLPsolver"
    )

    print("Path of the new solver: {}".format(new_solver_path))
    if os.path.exists(new_solver_path) and os.path.isdir(new_solver_path):
        shutil.rmtree(new_solver_path)

    # Creates code for symbolic model formulation given above, then contacts server to generate new solver
    generated_solver = solver.generate_solver(options)

    # Move the solver
    if os.path.isdir(solver_path):
        shutil.move(solver_path, new_solver_path)

    # remove forces temp files
    for filename in os.listdir(dir_path):
        if filename.endswith(".forces"):
            os.remove(os.path.join(dir_path, filename))
