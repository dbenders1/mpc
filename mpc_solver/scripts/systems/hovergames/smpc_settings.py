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
model_options["use_slack"] = True  # False for simple sim, True for Gazebo and drone
model_options["use_ct_feedback_law"] = False
model_options["use_dt_feedback_law"] = False
model_options["is_robust"] = False
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
if model_options["is_robust"] and not (
    model_options["use_ct_feedback_law"] or model_options["use_dt_feedback_law"]
):
    exit(
        "Robust model check failed: use_ct_feedback_law or use_dt_feedback_law should be set to True in order to use the robust model!"
        " Check pmpc_settings.py"
    )

# Define model, bounds and offline computations
robot = systems.Hovergames(model_options["use_input_rates"])
if model_options["is_robust"]:
    offline_comp = offline_computations.RohmpcOffline()
    model = dynamics.DroneModel(
        system=robot, options=model_options, offline_comp=offline_comp
    )
else:
    offline_comp = offline_computations.TmpcOffline()
    model = dynamics.DroneModel(
        system=robot, options=model_options, offline_comp=offline_comp
    )
#
# what we want to initialize during runtime:
# 0 -> only states
# 1 -> states and inputs
initialize_input_runtime = False
# ====================================================================


# Define MPC settings
# ====================================================================
integrator_options = dict()

# N: MPC horizon
# stepsize: timestep of the integrator
# steps: steps before reaching the next timestep: integration is done based on intervals of stepsize/steps (default: 1)
N = 8  # Drone
integrator_options["stepsize"] = 0.05
integrator_options["steps"] = 1

n_discs = 1  # Number of discs modeling the robot region
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
constants.add_constant("integrator_stepsize", np.array(integrator_options["stepsize"]))
constants.add_constant("integrator_steps", np.array(integrator_options["steps"]))
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
    weight_dict["input_delta_angles"] = 1
    weight_dict["input_delta_psi"] = 1
    weight_dict["input_thrust"] = 1
    weight_dict["input_angles"] = 1
    weight_dict["input_psi"] = 1
else:
    weight_dict["input_angles"] = 1
    weight_dict["input_psi"] = 1
    weight_dict["input_thrust"] = 1
weight_dict["goal_xy"] = 100
weight_dict["goal_z"] = 100
weight_dict["goal_yaw"] = 1
weight_dict["goal_xy_all"] = 0.01
weight_dict["goal_z_all"] = 100
weight_dict["goal_yaw_all"] = 0.01
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
        control_modules.TerminalConstraintsSteadyState(False),
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
