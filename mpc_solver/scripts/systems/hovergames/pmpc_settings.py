import numpy as np
from include import control_modules, dynamics, helpers, objective, systems
from include.offline_computations import offline_computations

# Define configuration
# ====================================================================
configuration = "HovergamesGO"
# ====================================================================

# Define system
# ====================================================================
# Define model options
model_options = dict()
model_options["nonlin"] = True
model_options["with_yaw"] = True
model_options["physical_only"] = False
model_options["use_input_rates"] = True
model_options["use_slack"] = False
model_options["use_ct_feedback_law"] = False
model_options["use_dt_feedback_law"] = False
model_options["use_tightened_system_constraints"] = True
model_options["use_tightened_obstacle_constraints"] = True
model_options["use_terminal_steady_state_constraint"] = True

# Model option checks
if model_options["nonlin"] and not model_options["with_yaw"]:
    exit(
        "Nonlinear model check failed: with_yaw should be set to True in order to use the nonlinear model!"
        " Check pmpc_settings.py"
    )
if model_options["use_ct_feedback_law"] and model_options["use_dt_feedback_law"]:
    exit(
        "Feedback law check failed: use_ct_feedback_law and use_dt_feedback_law cannot be set to True at the same time!"
        " Check pmpc_settings.py"
    )

# Define model, bounds and offline computations
robot = systems.Hovergames(model_options["use_input_rates"])
offline_comp = offline_computations.TmpcOffline()
model = dynamics.DroneModel(
    system=robot, options=model_options, offline_comp=offline_comp
)

# what we want to initialize during runtime:
# 0 -> only states
# 1 -> states and inputs
initialize_input_runtime = False
# ====================================================================


# --- Main MPC Parameters --- #
N = 4  # Simplesim/Gazebo/drone MPC Horizon (default: 4)
N_total = 5  # Total number of stages, difference with N is that it includes stages that are fixed and not optimized (default: 5, min: N+1)
n_discs = 1  # Number of discs modeling the robot region (default: 1)
integrator_options = dict()
integrator_options["stepsize"] = (
    0.05  # Timestep of the integrator - should equal TMPC stepsize (default: 0.05)
)
integrator_options["steps"] = (
    10  # beta: integration is done for a total amount of time equal to stepsize*steps - should equal TMPC ratio (default: 10)
)
integrator_options["use_ct_feedback_law"] = model_options[
    "use_ct_feedback_law"
]  # Use feedback in continuous model
integrator_options["use_dt_feedback_law"] = model_options[
    "use_dt_feedback_law"
]  # Use feedback in discrete model
initialize_input_runtime = False  # What we want to initialize during runtime (False: states, True: states and inputs)
print_level = 0  # 1 = timings, 2 = print progress
optimization_level = 2  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
maximum_iterations = 300  # Maximum number of iterations
mu0 = 20
hardcode_weights = False
# ====================================================================


# Define constants (that can be taken outside of solver)
# ====================================================================
constants = helpers.ConstantsStructure()
constants.add_constant(
    "use_input_rates", np.array(int(model_options["use_input_rates"]))
)
constants.add_constant("use_slack", np.array(int(model_options["use_slack"])))
constants.add_constant(
    "use_tightened_obstacle_constraints",
    np.array(int(model_options["use_tightened_obstacle_constraints"])),
)
if model_options["use_tightened_obstacle_constraints"]:
    constants.add_constant("c_o", np.array(offline_comp.get_c_o()))
    constants.add_constant("alpha", np.array(offline_comp.get_alpha()))
    constants.add_constant("P_delta", np.array(offline_comp.get_P_delta()))
constants.add_constant(
    "n_states_to_constrain", np.array(int(N_total - N + 1))
)  # Number of states to constrain includes the initial state constraint of the optimization problem with horizon N
# ====================================================================


# Define parameters
# ====================================================================
params = helpers.ParameterStructure(N)
# ====================================================================


# Define modules manager
# ====================================================================
modules = control_modules.ModuleManager(N, params)
# ====================================================================


# Define objective
# ====================================================================
modules.add_module(control_modules.GoalOrientedModule(params))


def use_objective(stage_idx, z, p, settings):
    params.load_objectives_params(stage_idx, p)
    return objective.hovergames_go_objective(stage_idx, z, p, settings)


# Define weights
weight_dict = dict()
if model_options["use_input_rates"]:
    weight_dict["input_delta_angles"] = 20
    weight_dict["input_delta_psi"] = 20
    weight_dict["input_thrust"] = 10
    weight_dict["input_angles"] = 10
    weight_dict["input_psi"] = 10
else:
    weight_dict["input_angles"] = 20
    weight_dict["input_psi"] = 10
    weight_dict["input_thrust"] = 10
weight_dict["goal_xy"] = 100
weight_dict["goal_z"] = 100
weight_dict["goal_yaw"] = 100
weight_dict["goal_xy_all"] = 0.1
weight_dict["goal_z_all"] = 0.1
weight_dict["goal_yaw_all"] = 0.1
weight_dict["thrust"] = 50
if model_options["use_slack"]:
    weight_dict["slack_linear"] = 0
    weight_dict["slack_quadratic"] = 1000
weights = helpers.Weights(params, weight_dict, hardcode_weights)
# ====================================================================


# Define constraints
# ====================================================================
# Parameters for the robot region
n_discs = 1

# Constraint modules to include
if model_options["use_tightened_system_constraints"]:
    tightened_system_stages = [i for i in range(N + 1)]
    nvar = model.nvar
    if model_options["use_input_rates"]:
        nvar = nvar - 3
    if model_options["use_slack"]:
        nvar = nvar - 1
    modules.add_module(
        control_modules.TightenedSystemConstraintsOld(params, nvar, offline_comp),
        tightened_system_stages,
    )

static_polyhedron_stages = [i for i in range(1, N + 1)]
modules.add_module(
    control_modules.StaticPolyhedronConstraintModule(
        params, n_discs, model_options["use_slack"]
    ),
    static_polyhedron_stages,
)

if model_options["use_terminal_steady_state_constraint"]:
    terminal_constraint_stages = N
    modules.add_module(
        control_modules.TerminalConstraintsSteadyState(model_options["use_slack"]),
        terminal_constraint_stages,
    )
# ====================================================================


# Print summary of parameters and constraints
# ====================================================================
print(constants)
# print(params)
npar = params.get_number_of_parameters()
print(f"Numer of parameters: {npar}")
nh = modules.get_number_of_constraints()
print(f"Number of constraints: {nh}")
# ====================================================================
