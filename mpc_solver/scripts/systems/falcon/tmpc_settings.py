import numpy as np
from include import control_modules, dynamics, helpers, objective, systems
from include.offline_computations import offline_computations

# Define configuration
# ====================================================================
configuration = "FalconTMPC"
# ====================================================================


# Define system
# ====================================================================
# Define model options
model_options = dict()
model_options["use_slack"] = False
model_options["use_tightened_system_constraints"] = True
model_options["use_tightened_obstacle_constraints"] = True
model_options["use_growing_tube"] = True
model_options["use_terminal_set_constraint"] = True

# Read offline-computed quantities and define system constraints and dynamical model
offline_comp = offline_computations.RohmpcOffline()
robot = systems.FalconT(offline_comp=offline_comp)
model = dynamics.FalconTModel(
    system=robot, options=model_options, offline_comp=offline_comp
)

# Get system variables
B_allocation = offline_comp.get_B_allocation()
C = offline_comp.get_C()
E = offline_comp.get_E()
g = offline_comp.get_g()
inertia = offline_comp.get_inertia()
kd = offline_comp.get_kd()
mass = offline_comp.get_mass()
thrust_map = offline_comp.get_thrust_map()
t_hover = offline_comp.get_t_hover()
w_bias = offline_comp.get_w_bias()

# Get tuning variables
L = offline_comp.get_L()
Q = offline_comp.get_Q()
R = offline_comp.get_R()
P = offline_comp.get_P()

# Get computed variables
K_delta = offline_comp.get_K_delta()
P_delta = offline_comp.get_P_delta()
# ====================================================================


# Define solver settings
# ====================================================================
integrator_options = dict()

# N: MPC horizon
# ratio: ratio between the planner and tracker layers (default: N)
# stepsize: timestep of the integrator - should equal the PMPC stepsize (default: 0.01)
# steps: steps before reaching the next timestep: integration is done based on intervals of stepsize/steps (default: 1)
N = 10
ratio = N
integrator_options["stepsize"] = 0.01
integrator_options["steps"] = 1
initialize_input_runtime = False  # What we want to initialize during runtime (False: states, True: states and inputs)
print_level = 0  # 1 = timings, 2 = print progress
optimization_level = 3  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
maximum_iterations = 300  # Maximum number of iterations
mu0 = 1e6
hardcode_weights = False
# ====================================================================


# Get and compute tightening variables
# ====================================================================
rho_c = offline_comp.get_rho_c()
w_bar_c = offline_comp.get_w_bar_c()
s_pred = offline_comp.get_s_array(
    integrator_options["stepsize"] * integrator_options["steps"],
    N + 1,
    model_options["use_growing_tube"],
)
epsilon = offline_comp.get_epsilon()
alpha = offline_comp.get_alpha()
c_o = offline_comp.get_c_o()
# ====================================================================


# Define constants (that can be taken outside of solver)
# ====================================================================
constants = helpers.ConstantsStructure()
constants.add_constant("ratio", np.array(ratio))
constants.add_constant("use_slack", np.array(int(model_options["use_slack"])))
constants.add_constant("integrator_stepsize", np.array(integrator_options["stepsize"]))
constants.add_constant("integrator_steps", np.array(integrator_options["steps"]))
constants.add_constant("B_allocation", np.array(B_allocation))
constants.add_constant("C", np.array(C))
constants.add_constant("E", np.array(E))
constants.add_constant("g", np.array(g))
constants.add_constant("inertia", np.array(inertia))
constants.add_constant("kd", np.array(kd))
constants.add_constant("mass", np.array(mass))
constants.add_constant("thrust_map", np.array(thrust_map))
constants.add_constant("t_hover", np.array(t_hover))
constants.add_constant("w_bias", np.array(w_bias))
constants.add_constant("L", np.array(L))
constants.add_constant("Q", np.array(Q))
constants.add_constant("R", np.array(R))
constants.add_constant("P", np.array(P))
constants.add_constant("K_delta", np.array(K_delta))
constants.add_constant("P_delta", np.array(P_delta))
constants.add_constant("rho_c", np.array(rho_c))
constants.add_constant("w_bar_c", np.array(w_bar_c))
constants.add_constant("s_pred", np.array(s_pred))
constants.add_constant("epsilon", np.array(epsilon))
constants.add_constant("alpha", np.array(alpha))
constants.add_constant("c_o", np.array(c_o))
constants.add_constant(
    "use_tightened_obstacle_constraints",
    np.array(int(model_options["use_tightened_obstacle_constraints"])),
)
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
    return objective.falcon_tmpc_objective(stage_idx, z, p, settings)


# Define weights
weight_dict = dict()
for u_idx, u_name in enumerate(model.get_input_names()):
    weight_dict["R_" + u_name] = R[u_idx, u_idx]
for x_idx, x_name in enumerate(model.get_state_names()):
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
if model_options["use_tightened_system_constraints"]:
    tightened_system_stages = [i for i in range(N + 1)]
    nvar = model.nvar
    if model_options["use_slack"]:
        nvar = nvar - 1
    modules.add_module(
        control_modules.TightenedSystemConstraints(
            nvar,
            offline_comp,
            s_pred,
            epsilon,
            model_options["use_slack"],
        ),
        tightened_system_stages,
    )

static_polyhedron_stages = [i for i in range(1, N + 1)]
modules.add_module(
    control_modules.StaticPolyhedronConstraintModule(
        params, n_discs, model_options["use_slack"]
    ),
    static_polyhedron_stages,
)

if model_options["use_terminal_set_constraint"]:
    terminal_constraint_stages = N
    modules.add_module(
        control_modules.TerminalConstraintsSet(
            P_delta, s_pred[-1], epsilon, alpha, model_options["use_slack"]
        ),
        terminal_constraint_stages,
    )
# ====================================================================


# Print summary of parameters and constraints
# ====================================================================
print(constants)
# print(params)
npar = params.get_number_of_parameters()
print(f"Number of parameters: {npar}")
nh = modules.get_number_of_constraints()
print(f"Number of constraints: {nh}")
# ====================================================================
