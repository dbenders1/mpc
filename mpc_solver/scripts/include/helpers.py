import os
import sys

import casadi
import numpy as np


def check_forces_path(forces_path):
    # Otherwise is it in a folder forces_path?
    try:
        if os.path.exists(forces_path) and os.path.isdir(forces_path):
            sys.path.append(forces_path)
        else:
            raise IOError("Forces path not found")

        import forcespro.nlp

        print("Forces found in: {}".format(forces_path))

        return True
    except:
        return False


def rotation_matrix(angle):
    return np.array(
        [
            [casadi.cos(angle), -casadi.sin(angle)],
            [casadi.sin(angle), casadi.cos(angle)],
        ]
    )


def load_forces_path():
    print_paths = ["PYTHONPATH"]
    # Is forces in the python path?
    try:
        import forcespro.nlp

        print("Forces found in PYTHONPATH")

        return
    except:
        pass

    paths = [
        os.path.join(os.path.expanduser("~"), "forces_pro_client"),
        os.path.join(os.getcwd(), "forces"),
        os.path.join(os.getcwd(), "forces_pro_lib"),
        os.path.join(os.getcwd(), "../forces"),
        os.path.join(os.getcwd(), "scripts/forces"),
        os.path.join(os.getcwd(), "forces/forces_pro_client"),
        os.path.join(os.getcwd(), "../forces/forces_pro_client"),
    ]
    for path in paths:
        if check_forces_path(path):
            return
        print_paths.append(path)

    print("Forces could not be imported, paths tried:\n")
    for path in print_paths:
        print("{}".format(path))
    print("\n")


class ConstantsStructure:
    def __init__(self):
        # Initialization of constants
        # For each of the constants, we store: name, number of dimensions, sizes and 1d array of values
        # self.constants has the following structure:
        # [{'name': constant_0_name, 'ndim': number of dimensions, 'sizes': sizes, 'start_idx': start index in values array, 'values': values},
        #  ...
        #  {'name': constant_0_name, 'ndim': number of dimensions, 'sizes': sizes, 'start_idx': start index in values array, 'values': values}]
        self.constants = []
        self.n_values = 0

    def add_constant(self, name, constant):
        # Create empty constant dictionary
        constant_dict = dict()

        # Fill name
        constant_dict["name"] = name

        # Fill start_idx
        constant_dict["start_idx"] = self.n_values

        # Fill ndim
        ndim = constant.ndim
        if ndim > 2:
            exit(f"Constants with three dimensions are not supported.")
        constant_dict["ndim"] = ndim

        # Fill sizes
        constant_dict["sizes"] = []
        if ndim == 0:
            constant_dict["sizes"].append(1)
        else:
            for dim_idx in range(ndim):
                constant_dict["sizes"].append(constant.shape[dim_idx])

        # Fill values in column-major order (Eigen in C defaults to column-major, so more efficient to save and read)
        constant_flattened = constant.flatten("F")
        constant_dict["values"] = constant_flattened
        self.n_values += len(constant_flattened)

        # Add to constants list
        self.constants.append(constant_dict)

    def __str__(self):
        result = f"NOTE:\nThe constants list of dictionaries contains the name, number of dimensions, size per dimension and values per constant.\n"
        result += f"Constants:\n"
        for constant_dict in self.constants:
            result += f"Name: {constant_dict['name']}\n"
            result += f"Ndim: {constant_dict['ndim']}\n"
            result += f"Sizes: {constant_dict['sizes']}\n"
            result += f"Start idx: {constant_dict['start_idx']}\n"
            result += f"Values: {constant_dict['values']}\n\n"
        return result


# Right now, the following rules apply:
# - Objectives and weights parameters are always defined for all stages
# - Constraints are not applied to all stages, but the parameters are included in all stages to ensure constant npar over all stages
class ParameterStructure:

    def __init__(self, N):
        # Initialization of parameters
        # self.params has the following structure:
        # {0: {'n_par': no+nw+nc,
        #      'n_par_objectives': no
        #      'n_par_weights': nw
        #      'n_par_constraints': nc
        #      'par_objectives_names': [param_0_name, ..., param_no_name],
        #      'par_weights_names': [param_0_name, ..., param_nw_name],
        #      'par_constraints_names': [param_0_name, ..., param_nc_name],
        #  ...
        #  N: {'n_par': no+nw+nc,
        #      'n_par_objectives': no
        #      'n_par_weights': nw
        #      'n_par_constraints': nc
        #      'par_objectives_names': [param_0_name, ..., param_no_name],
        #      'par_weights_names': [param_0_name, ..., param_nw_name],
        #      'par_constraints_names': [param_0_name, ..., param_nc_name],
        self.N = N
        self.params = [
            {
                "n_par": 0,
                "n_par_objectives": 0,
                "n_par_weights": 0,
                "n_par_constraints": 0,
                "par_objectives_names": [],
                "par_weights_names": [],
                "par_constraints_names": [],
            }
            for _ in range(N + 1)
        ]

    def add_parameter(self, name, type):
        stages = [stage_idx for stage_idx in range(self.N + 1)]

        for stage_idx in stages:
            self.params[stage_idx]["n_par"] += 1
            if type == "objectives":
                self.params[stage_idx]["n_par_objectives"] += 1
                self.params[stage_idx]["par_objectives_names"].append(name)
            elif type == "weights":
                self.params[stage_idx]["n_par_weights"] += 1
                self.params[stage_idx]["par_weights_names"].append(name)
            elif type == "constraints":
                self.params[stage_idx]["n_par_constraints"] += 1
                self.params[stage_idx]["par_constraints_names"].append(name)

    def add_multiple_parameters(self, name, amount, type):
        for i in range(amount):
            self.add_parameter(name + "_" + str(i), type)

    def get_weight_parameter(self, runtime_params, stage_idx, name):
        return runtime_params[
            self.params[stage_idx]["n_par_objectives"]
            + self.params[stage_idx]["par_weights_names"].index(name)
        ]

    def get_number_of_parameters(self):
        return [self.params[stage_idx]["n_par"] for stage_idx in range(self.N + 1)]

    def get_number_of_objectives_parameters(self):
        return [
            self.params[stage_idx]["n_par_objectives"]
            for stage_idx in range(self.N + 1)
        ]

    def get_number_of_weights_parameters(self):
        return [
            self.params[stage_idx]["n_par_weights"] for stage_idx in range(self.N + 1)
        ]

    def get_number_of_constraints_parameters(self):
        return [
            self.params[stage_idx]["n_par_constraints"]
            for stage_idx in range(self.N + 1)
        ]

    def load_objectives_params(self, stage_idx, runtime_params):
        for name in self.params[stage_idx]["par_objectives_names"]:
            setattr(
                self,
                name,
                runtime_params[
                    self.params[stage_idx]["par_objectives_names"].index(name)
                ],
            )

    def load_constraints_params(self, stage_idx, runtime_params):
        for name in self.params[stage_idx]["par_constraints_names"]:
            setattr(
                self,
                name,
                runtime_params[
                    self.params[stage_idx]["n_par_objectives"]
                    + self.params[stage_idx]["n_par_weights"]
                    + self.params[stage_idx]["par_constraints_names"].index(name)
                ],
            )

    def __str__(self):
        result = f"NOTE:\nThe parameters dictionary below contains equal amount of parameters for all stages to be able to use the same indices for each of the parameters.\nHowever, the constraints are applied to specific stages, so some constraints parameters might not be used in all stages.\nCheck settings.py for the specification of stages to which the (different) constraints apply!\n"
        result += f"Parameters:\n"
        for stage_idx, param_stage_dict in enumerate(self.params):
            result += f"Stage {stage_idx}:\n"
            for key, value in param_stage_dict.items():
                result += f"{key}: {value}\n"
            result += f"\n"
        return result


class Weights:
    def __init__(self, params, weight_dict, hardcode_weights=False):
        self.params = params
        self.weight_dict = weight_dict
        self.hardcode_weights = hardcode_weights

        for weight_name in weight_dict:
            self.params.add_parameter(weight_name, "weights")

    def get_value(self, runtime_params, stage_idx, name):
        if self.hardcode_weights:
            return self.weight_dict[name]
        else:
            return self.params.get_weight_parameter(runtime_params, stage_idx, name)
