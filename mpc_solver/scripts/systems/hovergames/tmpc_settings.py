import numpy as np
from include import control_modules, dynamics, helpers, objective, systems
from include.offline_computations import offline_computations

# Define configuration
# ====================================================================
configuration = "HovergamesRMPC"
# ====================================================================


# Define system
# ====================================================================
# Define model options
model_options = dict()
model_options["nonlin"] = True
model_options["with_yaw"] = True
model_options["physical_only"] = False
model_options["use_input_rates"] = False
model_options["use_slack"] = False
model_options["use_ct_feedback_law"] = False
model_options["use_dt_feedback_law"] = False
model_options["use_terminal_set_constraint"] = True


# Model option checks
if model_options["nonlin"] and not model_options["with_yaw"]:
    exit(
        "Nonlinear model check failed: with_yaw should be set to True in order to use the nonlinear model!"
        " Check tmpc_settings.py"
    )
if model_options["use_ct_feedback_law"] and model_options["use_dt_feedback_law"]:
    exit(
        "Feedback law check failed: use_ct_feedback_law and use_dt_feedback_law cannot be set to True at the same time!"
        " Check tmpc_settings.py"
    )

# Define model, bounds and offline computations
robot = systems.Hovergames(model_options["use_input_rates"])
offline_comp = offline_computations.TmpcOffline()
model = dynamics.DroneModel(
    system=robot, options=model_options, offline_comp=offline_comp
)

Q = offline_comp.get_Q()
R = offline_comp.get_R()
# ====================================================================


# Define solver settings
# ====================================================================
N = 10  # MPC Horizon (default: 10)
ratio = 10  # beta: ratio between the planner and tracker layers (default: N)
integrator_options = dict()
integrator_options["stepsize"] = (
    0.05  # Timestep of the integrator - should equal the PMPC stepsize (default: 0.05)
)
integrator_options["steps"] = (
    1  # Steps before reaching the next timestep: integration is done for a total amount of time equal to stepsize*steps (default: 1)
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
hardcode_weights = True
# ====================================================================


# Define constants (that can be taken outside of solver)
# ====================================================================
constants = helpers.ConstantsStructure()
constants.add_constant("ratio", np.array(ratio))
constants.add_constant(
    "use_input_rates", np.array(int(model_options["use_input_rates"]))
)
constants.add_constant("use_slack", np.array(int(model_options["use_slack"])))
constants.add_constant("integrator_stepsize", np.array(integrator_options["stepsize"]))
constants.add_constant("integrator_steps", np.array(integrator_options["steps"]))
constants.add_constant("Q", np.array(Q))
constants.add_constant("R", np.array(R))
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
modules.add_module(control_modules.ReferenceTrajectoryModule(params, model))


def use_objective(stage_idx, z, p, settings):
    params.load_objectives_params(stage_idx, p)
    return objective.hovergames_tmpc_objective(stage_idx, z, p, settings)


# Define weights
# NOTE: make sure these lists contain all inputs and states!
input_names = ["phi_c", "theta_c", "psi_c", "thrust_c"]
state_names = ["x", "y", "z", "vx", "vy", "vz", "phi", "theta", "psi", "thrust"]
weight_dict = dict()
for u_idx, u_name in enumerate(input_names):
    weight_dict["R_" + u_name] = R[u_idx, u_idx]
for x_idx, x_name in enumerate(state_names):
    weight_dict["Q_" + x_name] = Q[x_idx, x_idx]
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
if model_options["use_terminal_set_constraint"]:
    terminal_constraint_stages = N
    modules.add_module(
        control_modules.TerminalConstraintsSet(
            offline_comp, model_options["use_slack"]
        ),
        terminal_constraint_stages,
    )

static_polyhedron_stages = [i for i in range(1, N + 1)]
modules.add_module(
    control_modules.StaticPolyhedronConstraintModule(
        params, n_discs, model_options["use_slack"]
    ),
    static_polyhedron_stages,
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
