import casadi as cs
import helpers
import numpy as np

""" 
Defines inequality constraints of different types. 
See control_modules.py for their integration into the controller 
"""


# Class to keep track of:
# - Which constraints are applied to which stages
# - The corresponding number of constraints nh
# - The corresponding number of parameters npar
class ConstraintManager:
    def __init__(self, N, params):
        # Initialization of constraint types bookkeeping over the prediction horizon
        self.constraints = [[] for _ in range(N + 1)]

        # Initialization of lower- and upper-bounds bookkeeping
        self.lower_bound = [[] for _ in range(N + 1)]
        self.upper_bound = [[] for _ in range(N + 1)]

        # Initialization of number of constraints per stage bookkeeping
        self.nh = [0] * (N + 1)

        # Initialization of parameter bookkeeping
        # self.npar = 0
        self.params = params
        # self.param_idx = param_idx

    def add_constraint(self, constraint, stages):
        # Make sure stages is iterable
        if not hasattr(stages, "__iter__"):
            stages = [stages]

        # Add constraint in all stages in which it should be applied
        for stage_idx in stages:
            self.constraints[stage_idx].append(constraint)
            # constraint.add_parameters(stage_idx)
            constraint.append_upper_bound(self.upper_bound[stage_idx])
            constraint.append_lower_bound(self.lower_bound[stage_idx])
            self.nh[stage_idx] = self.nh[stage_idx] + constraint.nh
        # For now, use the same parameters in every stage
        constraint.add_parameters()

    def get_constraints(self, stage_idx, z, settings):
        constraints = []

        for constraint in self.constraints[stage_idx]:
            constraint.append_constraints(constraints, stage_idx, z, settings)

        return constraints


# Constraints used in robust MPC to tighten system constraints
class TightenedSystemConstraints:
    def __init__(self, nvar, offline_comp, s_pred, epsilon, use_slack=False):
        self.nh = 2 * nvar
        self.c_s = offline_comp.get_c_s()
        self.s_pred = s_pred
        self.epsilon = epsilon
        self.use_slack = use_slack

    def add_parameters(self):
        # Nothing to do here since the tightening of the system constraints can be defined offline
        pass

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.nh):
            lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.nh):
            upper_bound.append(0.0)

    def append_constraints(self, constraints, stage_idx, z, settings):
        # Get tube size
        s = self.s_pred[stage_idx]
        print(f"s[{stage_idx}]: {s}")

        # Define constraints lb <= your_constraint <= ub, with lb and ub lower and upper bounds
        # Tightened input constraints
        u = z[0 : settings.model.nu]
        nu = settings.model.nu
        if settings.model_options["use_slack"]:
            u = u[:-1]
            nu = nu - 1
        for u_idx, u_name in enumerate(settings.model.inputs):
            if settings.model_options["use_slack"] and u_name == "slack":
                continue

            lb = settings.robot.lower_bound[u_name]
            ub = settings.robot.upper_bound[u_name]
            constraints.append(-u[u_idx] + lb + self.c_s[u_idx] * s)
            constraints.append(u[u_idx] - ub + self.c_s[u_idx] * s)

        # Tightened state constraints
        x = z[settings.model.nu : settings.model.nu + settings.model.nx]
        for x_idx, x_name in enumerate(settings.model.states):
            lb = settings.robot.lower_bound[x_name]
            ub = settings.robot.upper_bound[x_name]
            constraints.append(
                -x[x_idx] + lb + self.c_s[nu + x_idx] * (s + self.epsilon)
            )
            constraints.append(
                x[x_idx] - ub + self.c_s[nu + x_idx] * (s + self.epsilon)
            )


# Old constraints used in robust MPC to tighten system constraints
class TightenedSystemConstraintsOld:
    def __init__(self, nvar, offline_comp):
        self.K_delta = offline_comp.get_K_delta()
        self.c_s = offline_comp.get_c_s()
        self.alpha = offline_comp.get_alpha()

        self.nh = 2 * nvar

    def add_parameters(self):
        # Nothing to do here, since it uses the parameter inserted by RobustTightening module
        pass

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.nh):
            lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.nh):
            upper_bound.append(0.0)

    def append_constraints(self, constraints, stage_idx, z, settings):
        # Retrieving tube size

        # Compute system input using linear feedback law K_delta
        u = z[0 : settings.model.nu]
        nu = settings.model.nu

        if settings.model_options["use_slack"]:
            u = u[:-1]
            nu = nu - 1
        x = z[settings.model.nu : settings.model.nu + settings.model.nx]
        if settings.model_options["use_input_rates"]:
            u[:-1] = u[:-1] + x[-3:]
            x = x[:-3]

        # Define constraints lb <= your_constraint <= ub, with lb and ub lower and upper bounds
        # Tightened input constraints
        for u_idx, u_name in enumerate(settings.model.inputs):
            if settings.model_options["use_slack"] and u_name == "slack":
                continue

            lb = settings.robot.lower_bound[u_name]
            ub = settings.robot.upper_bound[u_name]
            constraints.append(-u[u_idx] + lb + self.c_s[2 * u_idx] * self.alpha)
            constraints.append(u[u_idx] - ub + self.c_s[2 * u_idx + 1] * self.alpha)

        # Tightened state constraints
        for x_idx, x_name in enumerate(settings.model.states):
            if (
                settings.model_options["use_input_rates"]
                and x_idx >= settings.model.nx - 3
            ):
                continue
            lb = settings.robot.lower_bound[x_name]
            ub = settings.robot.upper_bound[x_name]
            constraints.append(-x[x_idx] + lb + self.c_s[2 * (nu + x_idx)] * self.alpha)
            constraints.append(
                x[x_idx] - ub + self.c_s[2 * (nu + x_idx) + 1] * self.alpha
            )


class TerminalConstraintsSteadyState:
    def __init__(self, use_slack=False):
        self.nh = 12
        self.use_slack = use_slack

    # Define the runtime parameters
    def add_parameters(self):
        # Nothing to do here
        pass

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.nh):
            lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.nh):
            upper_bound.append(0.0)

    def append_constraints(self, constraints, stage_idx, z, settings):
        # Obtain slack if desired
        if settings.model_options["use_slack"]:
            slack = z[settings.model.nu - 1]
        else:
            slack = 1e-7  # lower than the solver tolerances

        # Obtain terminal states
        vx = z[settings.model.get_idx("vx")]
        vy = z[settings.model.get_idx("vy")]
        vz = z[settings.model.get_idx("vz")]
        phi = z[settings.model.get_idx("phi")]
        theta = z[settings.model.get_idx("theta")]
        zero_states = [vx, vy, vz, phi, theta]
        thrust = z[settings.model.get_idx("thrust")]

        # Add terminal constraints with slack if desired
        for state in zero_states:
            constraints.append(state - slack)
            constraints.append(-state - slack)
        constraints.append(thrust - 9.81 - slack)
        constraints.append(-(thrust - 9.81) - slack)


class TerminalConstraintsSteadyStateFalcon:
    def __init__(self, use_slack=False):
        self.nh = 16
        # self.nh = 24
        self.use_slack = use_slack

    # Define the runtime parameters
    def add_parameters(self):
        # Nothing to do here
        pass

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.nh):
            lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.nh):
            upper_bound.append(0.0)

    def append_constraints(self, constraints, stage_idx, z, settings):
        # Obtain slack if desired
        if settings.model_options["use_slack"]:
            slack = z[settings.model.nu - 1]
        else:
            slack = 1e-7  # lower than the solver tolerances

        # Obtain terminal states
        phi = z[settings.model.get_idx("phi")]
        theta = z[settings.model.get_idx("theta")]
        vx = z[settings.model.get_idx("vx")]
        vy = z[settings.model.get_idx("vy")]
        vz = z[settings.model.get_idx("vz")]
        wbx = z[settings.model.get_idx("wbx")]
        wby = z[settings.model.get_idx("wby")]
        wbz = z[settings.model.get_idx("wbz")]
        zero_states = [phi, theta, vx, vy, vz, wbx, wby, wbz]

        # Add terminal constraints with slack if desired
        for state in zero_states:
            constraints.append(state - slack)
            constraints.append(-state - slack)


class TerminalConstraintsSet:
    def __init__(self, P_delta, s_T, epsilon, alpha, use_slack=False):
        self.nh = 1
        self.P_delta = P_delta
        self.s_T = s_T
        self.epsilon = epsilon
        self.alpha = alpha
        self.use_slack = use_slack

    # Define the runtime parameters
    def add_parameters(self):
        # Nothing to do here
        pass

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.nh):
            lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.nh):
            upper_bound.append(0.0)

    def append_constraints(self, constraints, stage_idx, z, settings):
        # Obtain runtime states
        # Only for 3D
        x_idx = settings.model.get_idx("x")
        x = z[x_idx : x_idx + settings.model.nx]
        u = z[0 : settings.model.nu]

        if settings.model_options["use_slack"]:
            slack = u[-1]

        # Compute error with respect to reference trajectory
        x_error = []
        for i in range(settings.model.nx):
            u_x_ref_idx = x_idx + i
            if settings.model_options["use_slack"]:
                u_x_ref_idx = u_x_ref_idx - 1
            x_error.append(
                (x[i] - getattr(settings.params, "u_x_ref_" + str(u_x_ref_idx)))
            )
        x_error = np.array(x_error).reshape(-1, 1)

        if self.use_slack:
            constraints.append(
                cs.sqrt(
                    cs.mtimes(
                        cs.mtimes(x_error.T, self.P_delta),
                        x_error,
                    )
                )
                + self.s_T
                + self.epsilon
                - self.alpha
                - slack
            )
        else:
            constraints.append(
                cs.sqrt(
                    cs.mtimes(
                        cs.mtimes(x_error.T, self.P_delta),
                        x_error,
                    )
                )
                + self.s_T
                + self.epsilon
                - self.alpha
            )


# Removed "Reuse constraints" because you can now define the parameters by name
# Constraints of the form Ax <= b (+ slack)
class LinearConstraints:

    def __init__(self, params, n_discs, num_constraints, use_slack=False):
        self.params = params
        self.num_constraints = num_constraints
        self.n_discs = n_discs
        self.nh = num_constraints * n_discs
        self.use_slack = use_slack

    def add_parameters(self):
        for disc in range(self.n_discs):
            for i in range(self.num_constraints):
                self.params.add_parameter(
                    self.constraint_name(disc, i) + "_a1", "constraints"
                )
                self.params.add_parameter(
                    self.constraint_name(disc, i) + "_a2", "constraints"
                )
                self.params.add_parameter(
                    self.constraint_name(disc, i) + "_b", "constraints"
                )

    def constraint_name(self, disc_idx, constraint_idx):
        return "disc" + str(disc_idx) + "_" + str(constraint_idx)

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.num_constraints):
            for disc in range(self.n_discs):
                lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.num_constraints):
            for disc in range(self.n_discs):
                upper_bound.append(0.0)

    def append_constraints(self, constraints, stage_idx, z, settings):
        # Retrieve variables
        x = z[settings.model.nu : settings.model.nu + settings.model.nx]
        u = z[0 : settings.model.nu]
        if self.use_slack:
            slack = u[-1]

        # States
        pos_x = settings.model.get_state(z, "x", True)
        pos_y = settings.model.get_state(z, "y", True)
        pos = np.array([pos_x, pos_y])

        for disc_it in range(self.n_discs):
            disc_pos = pos

            # A'x <= b
            for constraint_it in range(self.num_constraints):
                a1 = getattr(
                    settings.params,
                    self.constraint_name(disc_it, constraint_it) + "_a1",
                )
                a2 = getattr(
                    settings.params,
                    self.constraint_name(disc_it, constraint_it) + "_a2",
                )
                b = getattr(
                    settings.params, self.constraint_name(disc_it, constraint_it) + "_b"
                )

                if self.use_slack:
                    constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b - slack)
                else:
                    constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b)
