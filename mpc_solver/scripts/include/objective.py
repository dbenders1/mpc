import casadi as cs
import numpy as np


def falcon_tmpc_objective(stage_idx, z, p, settings):
    # Initialise cost at 0
    cost = 0.0

    # Retrieve variables
    x = z[settings.model.nu : settings.model.nu + settings.model.nx]
    u = z[0 : settings.model.nu]

    if settings.model_options["use_slack"]:
        slack = settings.model.get_z_index(z, "slack", True)

    # Add cost terms
    # ------------------------------- #
    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(x, u, p, settings, stage_idx)
    # ------------------------------- #

    if settings.model_options["use_slack"]:
        cost += (
            settings.weights.get_value(p, stage_idx, "slack_linear") * slack
            + settings.weights.get_value(p, stage_idx, "slack_quadratic") * slack**2
        )

    return cost


def falcon_go_objective(stage_idx, z, p, settings):
    # Initialise cost at 0
    cost = 0.0

    # Constants
    t_hover = settings.t_hover

    # Slack
    if settings.model_options["use_slack"]:
        slack = settings.model.get_z_index(z, "slack", True)

    # Inputs
    t0c = settings.model.get_z_index(z, "t0c", True)
    t1c = settings.model.get_z_index(z, "t1c", True)
    t2c = settings.model.get_z_index(z, "t2c", True)
    t3c = settings.model.get_z_index(z, "t3c", True)

    # Add cost terms
    # ------------------------------- #
    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(z, p, settings, stage_idx)

    # Inputs
    cost += settings.weights.get_value(p, stage_idx, "tc") * (
        (t0c - t_hover) ** 2
        + (t1c - t_hover) ** 2
        + (t2c - t_hover) ** 2
        + (t3c - t_hover) ** 2
    )

    if settings.model_options["use_slack"]:
        cost += (
            settings.weights.get_value(p, stage_idx, "slack_linear") * slack
            + settings.weights.get_value(p, stage_idx, "slack_quadratic") * slack**2
        )
    # ------------------------------- #

    return cost


def hovergames_tmpc_objective(stage_idx, z, p, settings):
    # Initialise cost at 0
    cost = 0.0

    # Retrieve variables
    x = z[settings.model.nu : settings.model.nu + settings.model.nx]
    u = z[0 : settings.model.nu]

    if settings.model_options["use_slack"]:
        slack = settings.model.get_z_index(z, "slack", True)

    # Add cost terms
    # ------------------------------- #
    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(x, u, p, settings, stage_idx)
    # ------------------------------- #

    if settings.model_options["use_slack"]:
        cost += (
            settings.weights.get_value(p, stage_idx, "slack_linear") * slack
            + settings.weights.get_value(p, stage_idx, "slack_quadratic") * slack**2
        )

    return cost


def hovergames_go_objective(stage_idx, z, p, settings):
    # Initialise cost at 0
    cost = 0.0

    if settings.model_options["use_slack"]:
        slack = settings.model.get_z_index(z, "slack", True)

    # Inputs
    if settings.model_options["use_input_rates"]:
        delta_phi_c = settings.model.get_z_index(z, "delta_phi_c", True)
        delta_theta_c = settings.model.get_z_index(z, "delta_theta_c", True)
        delta_psi_c = settings.model.get_z_index(z, "delta_psi_c", True)
        phi_c = settings.model.get_z_index(z, "phi_c", True)
        theta_c = settings.model.get_z_index(z, "theta_c", True)
        psi_c = settings.model.get_z_index(z, "psi_c", True)
    else:
        phi_c = settings.model.get_z_index(z, "phi_c", True)
        theta_c = settings.model.get_z_index(z, "theta_c", True)
        psi_c = settings.model.get_z_index(z, "psi_c", True)

    thrust_c = settings.model.get_z_index(z, "thrust_c", True)
    thrust = settings.model.get_z_index(z, "thrust", True)

    # Add cost terms
    # ------------------------------- #
    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(z, p, settings, stage_idx)

    # Inputs
    if settings.model_options["use_input_rates"]:
        cost += (
            settings.weights.get_value(p, stage_idx, "input_delta_angles")
            * (delta_phi_c**2 + delta_theta_c**2)
            + settings.weights.get_value(p, stage_idx, "input_delta_psi")
            * delta_psi_c**2
            + settings.weights.get_value(p, stage_idx, "input_angles")
            * (phi_c**2 + theta_c**2)
            + settings.weights.get_value(p, stage_idx, "input_psi") * psi_c**2
        )
    else:
        cost += (
            settings.weights.get_value(p, stage_idx, "input_angles")
            * (phi_c**2 + theta_c**2)
            + settings.weights.get_value(p, stage_idx, "input_psi") * psi_c**2
        )

    cost += (
        settings.weights.get_value(p, stage_idx, "input_thrust")
        * (thrust_c - 9.81) ** 2
    )
    cost += settings.weights.get_value(p, stage_idx, "thrust") * (thrust - 9.81) ** 2

    if settings.model_options["use_slack"]:
        cost += (
            settings.weights.get_value(p, stage_idx, "slack_linear") * slack
            + settings.weights.get_value(p, stage_idx, "slack_quadratic") * slack**2
        )
    # ------------------------------- #

    return cost


class CostTerm:

    def __init__(self, weight, variable):
        self.weight = weight
        self.variable = variable

    def cost(self):
        raise IOError("Costterm with undefined cost")


class QuadraticCost(CostTerm):

    def __init__(self, weight, variable):
        super().__init__(weight, variable)

    def cost(self):
        return self.weight * self.variable**2


class GoalOrientedObjective:
    def __init__(self, params):
        self.define_parameters(params)

    def define_parameters(self, params):
        params.add_parameter("goal_x", "objectives")
        params.add_parameter("goal_y", "objectives")
        params.add_parameter("goal_z", "objectives")
        params.add_parameter("goal_yaw", "objectives")

    def huber_loss(self, x, delta):
        return cs.if_else(x < delta**2, 0.5 * x, delta * (cs.sqrt(x) - 0.5 * delta))

    def pseudo_huber_loss(self, x, delta):
        return delta**2 * (cs.sqrt(1 + (cs.sqrt(x) / delta) ** 2) - 1)

    def get_value(self, z, p, settings, stage_idx):
        # Set initial cost
        cost = 0.0

        # Obtain current position and yaw
        pos_x = settings.model.get_z_index(z, "x", True)
        pos_y = settings.model.get_z_index(z, "y", True)
        pos_z = settings.model.get_z_index(z, "z", True)
        yaw = settings.model.get_z_index(z, "psi", True)

        # Obtain goal position and yaw
        goal_x = getattr(settings.params, "goal_x")
        goal_y = getattr(settings.params, "goal_y")
        goal_z = getattr(settings.params, "goal_z")
        goal_yaw = getattr(settings.params, "goal_yaw")

        # Obtain delta to use in Huber loss
        delta = 0.5

        # Compute distances
        dist_x = goal_x - pos_x
        dist_y = goal_y - pos_y
        dist_z = goal_z - pos_z
        dist_yaw = goal_yaw - yaw

        # Need to add a constant to make sure the cost is never zero for the square root
        dist_xy_squared = dist_x**2 + dist_y**2

        if settings.hardcode_weights:
            if stage_idx >= settings.N:
                cost += (
                    50 * dist_x**2 + 50 * dist_y**2 + 50 * dist_z**2 + 50 * dist_yaw**2
                )
            else:
                cost += 1 * dist_x**2
                cost += 1 * dist_y**2
                cost += 10 * dist_z**2
                cost += 1 * dist_yaw**2
        else:
            # self.huber_loss(dist_xy_squared)
            # (dist_x ** 2 + dist_y ** 2)
            if stage_idx == settings.N:
                cost += (
                    settings.weights.get_value(p, stage_idx, "goal_xy")
                    * self.huber_loss(dist_xy_squared, delta)
                    + settings.weights.get_value(p, stage_idx, "goal_z") * dist_z**2
                    + settings.weights.get_value(p, stage_idx, "goal_yaw") * dist_yaw**2
                )
            else:
                cost += (
                    settings.weights.get_value(p, stage_idx, "goal_xy_all")
                    * self.huber_loss(dist_xy_squared, delta)
                    + settings.weights.get_value(p, stage_idx, "goal_z_all") * dist_z**2
                    + settings.weights.get_value(p, stage_idx, "goal_yaw_all")
                    * dist_yaw**2
                )

        return cost


class ReferenceTrajectoryObjective:

    def __init__(self, params, model):
        # Load model
        self.model = model

        # Determine state and input dimensions
        if self.model.use_slack:
            self.nu = len(self.model.inputs) - 1
        else:
            self.nu = len(self.model.inputs)
        self.nx = len(self.model.states)

        # Insert parameter space in online parameter list
        self.define_parameters(params)

    def define_parameters(self, params):
        # Reserve parameter space for inputs and states references for 1 stage
        params.add_multiple_parameters("u_x_ref", self.nu + self.nx, "objectives")

    def get_value(self, x, u, p, settings, stage_idx):
        cost = 0

        # Determine discretization time
        dt = (
            settings.integrator_options["steps"]
            * settings.integrator_options["stepsize"]
        )

        # Stage costs
        if stage_idx < settings.N:
            for idx, input_name in enumerate(self.model.inputs):
                if input_name != "slack":
                    cost += QuadraticCost(
                        settings.weights.get_value(p, stage_idx, "R_" + input_name)
                        * dt,
                        u[idx] - getattr(settings.params, "u_x_ref_" + str(idx)),
                    ).cost()

            for idx, state_name in enumerate(self.model.states):
                cost += QuadraticCost(
                    settings.weights.get_value(p, stage_idx, "Q_" + state_name) * dt,
                    x[idx] - getattr(settings.params, "u_x_ref_" + str(self.nu + idx)),
                ).cost()

        # Terminal cost (involves only state error cost terms)
        elif stage_idx == settings.N:
            x_err = []
            for idx, state_name in enumerate(self.model.states):
                x_err.append(
                    x[idx] - getattr(settings.params, "u_x_ref_" + str(self.nu + idx))
                )
            x_err = np.array(x_err).reshape(-1, 1)
            cost += cs.mtimes(
                cs.mtimes(x_err.T, settings.offline_comp.get_P()),
                x_err,
            )
        return cost
