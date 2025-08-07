import inequality
import objective


class ModuleManager:
    """
    The idea of modules is that they can include multiple constraint sets if necessary
    In addition, they are directly linked to the c++ code module
    """

    def __init__(self, N, params):
        self.modules = []
        self.constraint_manager = inequality.ConstraintManager(N, params)
        self.params = params

    def add_constraint(self, constraint, stages=None):
        self.constraint_manager.add_constraint(constraint, stages)

    def add_module(self, module, stages=None):
        self.modules.append(module)

        if module.type == "constraint":
            for constraint in module.constraints:
                self.constraint_manager.add_constraint(constraint, stages)
        elif module.type == "constraint_params":
            for param_name in module.param_names:
                print(f"param_name: {param_name}")
                self.params.add_parameter(param_name, "constraints")

    def inequality_constraints(self, stage_idx, z, param, settings):
        # Update parameters before constructing constraints
        settings.params.load_constraints_params(stage_idx, param)
        return self.constraint_manager.get_constraints(stage_idx, z, settings)

    def get_number_of_constraints(self):
        return self.constraint_manager.nh

    def __str__(self):
        result = "--- MPC Modules ---\n"
        for module in self.modules:
            result += str(module) + "\n"

        return result


class Module:

    def __init__(self):
        self.module_name = "UNDEFINED"
        self.with_params = True
        self.description = ""

    def __str__(self):
        result = (
            self.type.capitalize() + ": " + self.module_name + " - " + self.description
        )
        return result


""" OBJECTIVE MODULES """


class GoalOrientedModule(Module):
    """
    Track position subgoals
    """

    def __init__(self, params):
        self.module_name = (
            "GoalOriented"  # Needs to correspond to the c++ name of the module
        )
        self.with_params = True
        self.import_name = "modules_objectives/goal_oriented.h"
        self.type = "objective"
        self.description = "Goal oriented formulation using way_points"

        self.objectives = []
        self.objectives.append(objective.GoalOrientedObjective(params))


class ReferenceTrajectoryModule(Module):
    """
    Track a reference defined by states and inputs
    """

    def __init__(self, params, model):
        self.module_name = (
            "ReferenceTrajectory"  # Needs to correspond to the c++ name of the module
        )
        self.with_params = True
        self.import_name = "modules_objectives/reference_trajectory.h"
        self.type = "objective"
        self.description = "Tracks a reference defined by states and inputs"

        self.objectives = []
        self.objectives.append(objective.ReferenceTrajectoryObjective(params, model))


""" CONSTRAINT MODULES """


class TightenedSystemConstraints(Module):
    """
    Linear constraints to tighten the system constraints
    """

    def __init__(self, nvar, offline_comp, s_pred, epsilon, use_slack):
        self.module_name = "TightenedSystemConstraints"  # Needs to correspond to the c++ name of the module
        self.with_params = False
        self.import_name = ""
        self.type = "constraint"
        self.description = "Tightened system constraints to ensure robustness"

        self.constraints = []
        self.constraints.append(
            inequality.TightenedSystemConstraints(
                nvar, offline_comp, s_pred, epsilon, use_slack
            )
        )


class TightenedSystemConstraintsOld(Module):
    """
    Linear constraints to tighten the system constraints
    """

    def __init__(self, params, nvar, offline_comp):
        self.module_name = "TightenedSystemConstraintsOld"  # Needs to correspond to the c++ name of the module
        self.with_params = False
        self.import_name = ""
        self.type = "constraint"
        self.description = "Tightened system constraints to ensure robustness. Uses RobustTightening module"

        self.constraints = []
        self.constraints.append(
            inequality.TightenedSystemConstraintsOld(nvar, offline_comp)
        )


class TerminalConstraintsSteadyState(Module):
    """
    Terminal constraints
    """

    def __init__(self, use_slack):
        self.module_name = "TerminalConstraintsSteadyState"  # Needs to correspond to the c++ name of the module
        self.with_params = False
        self.import_name = ""
        self.type = "constraint"
        self.description = "Terminal constraints to ensure steady state"

        self.constraints = []
        self.constraints.append(inequality.TerminalConstraintsSteadyState(use_slack))


class TerminalConstraintsSteadyStateFalcon(Module):
    """
    Terminal constraints
    """

    def __init__(self, use_slack):
        self.module_name = "TerminalConstraintsSteadyStateFalcon"  # Needs to correspond to the c++ name of the module
        self.with_params = False
        self.import_name = ""
        self.type = "constraint"
        self.description = (
            "Terminal constraints to ensure steady state for Falcon quadrotor"
        )

        self.constraints = []
        self.constraints.append(
            inequality.TerminalConstraintsSteadyStateFalcon(use_slack)
        )


class TerminalConstraintsSet(Module):
    """
    Terminal constraints set for position
    """

    def __init__(self, P_delta, s_T, epsilon, alpha, use_slack):
        self.module_name = "TerminalConstraintsSet"  # Needs to correspond to the c++ name of the module
        self.with_params = False
        self.import_name = ""
        self.type = "constraint"
        self.description = "Terminal constraints set for position"

        self.constraints = []
        self.constraints.append(
            inequality.TerminalConstraintsSet(P_delta, s_T, epsilon, alpha, use_slack)
        )


class StaticPolyhedronConstraintModule(Module):
    """
    Linear constraints defining a general polyhedron for static collision avoidance
    """

    def __init__(self, params, n_discs, use_slack):
        self.module_name = "StaticPolyhedronConstraints"  # Needs to correspond to the c++ name of the module
        self.with_params = True
        self.import_name = "modules_constraints/static_polyhedron_constraints.h"
        self.type = "constraint"
        self.description = "Avoid static obstacles using decomp_util"

        self.constraints = []
        self.constraints.append(
            inequality.LinearConstraints(params, n_discs, 24, use_slack)
        )  # max 24 constraints per stage
