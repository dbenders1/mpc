import datetime
import importlib
import os
import sys

file_loaction = os.path.dirname(os.path.abspath(__file__))
scripts_location = os.path.abspath(os.path.join(__file__, "../../"))

# Now parent directory is added, we can find helpers.py script
from include import helpers

# If your forces is in this directory add it
helpers.load_forces_path()

# def write_model_header(settings, model):


def generate_cpp_files(Names, module_settings, system, pkg_path):

    # CPP files path and name
    cpp_path = pkg_path + "/src/" + system
    if not os.path.exists(cpp_path):
        os.makedirs(cpp_path)

    # CPP files path and name
    h_path = pkg_path + "/include/mpc_solver/" + system
    if not os.path.exists(h_path):
        os.makedirs(h_path)

    cpp_path_interface = cpp_path + "/" + system + "_solver_interface.cpp"
    h_path_interface = h_path + "/" + system + "_solver_interface.h"

    # Show the file paths and check if other solvers need to be generated
    print("Placing header files in: {}\n".format(h_path))
    print("Placing cpp files in: {}\n".format(cpp_path))

    # Date and Time
    now = datetime.datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

    # Open the files
    cpp_file_interface = open(cpp_path_interface, "w")
    h_file_interface = open(h_path_interface, "w")

    # create variables
    solver_classes = ""
    header_files = ""
    solver_if_statement = ""
    solvers = ""
    solvers_emplace = ""
    weight_load = ""

    ## Loop through names
    index = 0
    for name in Names:
        ## Define amount of solver stages
        # NOTE: this is hard-coded and should comply to the settings in generate_solver.py!
        module_settings[index].N_bar = module_settings[index].N + 1

        ## Global variables
        DT = (
            module_settings[index].integrator_options["steps"]
            * module_settings[index].integrator_options["stepsize"]
        )
        N = module_settings[index].N
        NU = module_settings[index].model.nu
        NX = module_settings[index].model.nx
        if module_settings[index].initialize_input_runtime == False:
            NINIT = NX
        else:
            NINIT = NU + NX
        n_par = module_settings[index].params.params[0]["n_par"]
        NDISCS = module_settings[index].n_discs

        # Inputs
        inputs_names = ""
        for i in range(len(module_settings[index].model.inputs)):
            inputs_names = (
                inputs_names + ',"' + module_settings[index].model.inputs[i] + '"'
            )
        inputs_names = inputs_names[1:]

        # States
        states_names = ""
        for i in range(len(module_settings[index].model.states)):
            states_names = (
                states_names + ',"' + module_settings[index].model.states[i] + '"'
            )
        states_names = states_names[1:]

        # Constants
        constants_names = ""
        constants_ndims_str = ""
        constants_sizes_str = ""
        constants_start_idc_str = ""
        constants_values_str = ""
        for constant_dict in module_settings[index].constants.constants:
            constants_names = constants_names + ',"' + constant_dict["name"] + '"'
            constants_ndims_str = constants_ndims_str + "," + str(constant_dict["ndim"])
            for size in constant_dict["sizes"]:
                constants_sizes_str = constants_sizes_str + "," + str(size)
            constants_start_idc_str = (
                constants_start_idc_str + "," + str(constant_dict["start_idx"])
            )
            for value in constant_dict["values"]:
                constants_values_str = constants_values_str + "," + str(value)
        constants_names = constants_names[1:]
        constants_ndims_str = constants_ndims_str[1:]
        constants_sizes_str = constants_sizes_str[1:]
        constants_start_idc_str = constants_start_idc_str[1:]
        constants_values_str = constants_values_str[1:]

        # Parameters
        # Objectives parameters
        n_par_objectives = module_settings[
            index
        ].params.get_number_of_objectives_parameters()[0]
        par_objectives_names = ""
        for objective_name in module_settings[index].params.params[0][
            "par_objectives_names"
        ]:
            par_objectives_names = par_objectives_names + ',"' + objective_name + '"'
        par_objectives_names = par_objectives_names[1:]

        # Weights parameters
        n_par_weights = module_settings[
            index
        ].params.get_number_of_weights_parameters()[0]
        par_weights_names = ""
        for idx, weight_name in enumerate(
            module_settings[index].params.params[0]["par_weights_names"]
        ):
            par_weights_names = par_weights_names + ',"' + weight_name + '"'
        par_weights_names = par_weights_names[1:]

        # Constraints
        n_par_constraints = module_settings[
            index
        ].params.get_number_of_constraints_parameters()[0]
        par_constraints_names = ""
        for constraint_name in module_settings[index].params.params[0][
            "par_constraints_names"
        ]:
            par_constraints_names = par_constraints_names + ',"' + constraint_name + '"'
        par_constraints_names = par_constraints_names[1:]

        # All parameters
        pars_names = (
            par_objectives_names + "," + par_weights_names + "," + par_constraints_names
        )

        # Modules
        module_names = ""
        for module in module_settings[index].modules.modules:
            if module.with_params:
                module_names = module_names + ',"' + module.module_name + '"'
        module_names = module_names[1:]

        # Open the files
        cpp_path_solver = cpp_path + "/" + name + "solver.cpp"
        cpp_file_solver = open(cpp_path_solver, "w")
        h_path_solver = h_path + "/" + name + "solver.h"
        h_file_solver = open(h_path_solver, "w")

        # Create the solver specific header file constructor
        h_file_solver.write(
            "// This file is automatically generated \n"
            + "// Time of creation:"
            + dt_string
            + "\n"
            + "#ifndef MPC_SOLVER_"
            + name.upper()
            + "SOLVER_INTERFACE_H \n"
            + "#define MPC_SOLVER_"
            + name.upper()
            + "SOLVER_INTERFACE_H \n\n"
            + "#include <memory> \n\n"
            + "#include <mpc_base/solver_base.h> \n\n"
            + "namespace mpc_solver{ \n\n"
            + "class Solver"
            + str(index)
            + " \n"
            + "{\n"
            + "public: \n"
            + "\t"
            + "SolverBase* solver_"
            + name
            + "; \n\n"
            + "\t"
            + "Solver"
            + str(index)
            + "();\n\n"
            + "}; \n"
            + "}/* namespace mpc_solver*/ \n\n"
            + "#endif"
        )

        # Create the solver specific header file constructor
        cpp_file_solver.write(
            "// This file is automatically generated \n"
            + "// Time of creation:"
            + dt_string
            + "\n"
            + "#include <mpc_solver/"
            + system
            + "/"
            + name
            + "solver.h> \n\n"
            + "#include <mpc_solver/core/solver.h> \n\n"
            + "#include <mpc_solver/"
            + system
            + "/"
            + name
            + "FORCESNLPsolver/include/"
            + name
            + "FORCESNLPsolver.h> \n"
            + "#include <mpc_solver/"
            + system
            + "/"
            + name
            + "FORCESNLPsolver/include/"
            + name
            + "FORCESNLPsolver_memory.h> \n\n"
            + 'extern "C" \n'
            + "{	\n"
            + "\t#include <mpc_solver/"
            + system
            + "/"
            + name
            + "FORCESNLPsolver/"
            + name
            + "FORCESNLPsolver_interface.c> \n"
            + "\t#include <mpc_solver/"
            + system
            + "/"
            + name
            + "FORCESNLPsolver/"
            + name
            + "FORCESNLPsolver_model.c> \n"
            + "} \n\n"
            "namespace mpc_solver{ \n\n"
            + "template class Solver<"
            + name
            + "FORCESNLPsolver_params, "
            + name
            + "FORCESNLPsolver_output, \n"
            + "\t"
            + name
            + "FORCESNLPsolver_info, "
            + name
            + "FORCESNLPsolver_mem, "
            + name
            + "FORCESNLPsolver_extfunc, FILE, solver_int32_default>; \n\n"
            + "typedef Solver<"
            + name
            + "FORCESNLPsolver_params, "
            + name
            + "FORCESNLPsolver_output, "
            + name
            + "FORCESNLPsolver_info, \n"
            + "\t"
            + name
            + "FORCESNLPsolver_mem, "
            + name
            + "FORCESNLPsolver_extfunc, FILE, solver_int32_default> TSolver"
            + str(index)
            + ";\n\n"
            + "Solver"
            + str(index)
            + "::Solver"
            + str(index)
            + "(): "
            + "solver_"
            + name
            + "(new TSolver"
            + str(index)
            + "("
            + str(DT)
            + ", "
            + str(N)
            + ", "
            + str(NINIT)
            + ",\n"
            + "\t"
            + str(NU)
            + ", "
            + str(NX)
            + ",\n"
            + "\t{"
            + inputs_names
            + ","
            + states_names
            + "},\n"
            + "\t{"
            + constants_names
            + "},\n"
            + "\t{"
            + constants_ndims_str
            + "},\n"
            + "\t{"
            + constants_sizes_str
            + "},\n"
            + "\t{"
            + constants_start_idc_str
            + "},\n"
            + "\t{"
            + constants_values_str
            + "},\n"
            + "\t"
            + str(n_par)
            + ",\n"
            + "\t{"
            + pars_names
            + "},\n"
            + "\t"
            + str(n_par_objectives)
            + ",\n"
            + "\t{"
            + par_objectives_names
            + "},\n"
            + "\t"
            + str(n_par_weights)
            + ",\n"
            + "\t{"
            + par_weights_names
            + "},\n"
            + "\t"
            + str(NDISCS)
            + ",\n"
            "\t"
            + str(n_par_constraints)
            + ",\n"
            + "\t{"
            + par_constraints_names
            + "},\n"
            + "\t{"
            + module_names
            + "},\n"
            + "\t"
            + str(index)
            + ",\n"
            + "\t&"
            + name
            + "FORCESNLPsolver_external_mem,\n"
            + "\t&"
            + name
            + "FORCESNLPsolver_get_mem_size,\n"
            + "\t&"
            + name
            + "FORCESNLPsolver_adtool2forces,\n"
            + "\t&"
            + name
            + "FORCESNLPsolver_solve))\n"
            + "{ };\n"
            + "}/*namespace mpc_solver*/\n\n"
        )

        cpp_file_solver.close()
        h_file_solver.close()

        solver_classes = (
            solver_classes + "\tSolver" + str(index) + " solver_" + name + "; \n"
        )
        header_files = (
            header_files
            + "#include <mpc_solver/"
            + system
            + "/"
            + name
            + "solver.h> \n"
        )

        if index == 0:
            solver_if_statement = (
                solver_if_statement
                + "\tif (solver_number == "
                + str(index)
                + ") \n"
                + "\t{\n"
                + "\t\tSolver"
                + str(index)
                + " solver; \n"
                + "\t\tptr_solver = std::unique_ptr<SolverBase>(std::move(solver.solver_"
                + name
                + ")); \n"
                + "\t} \n"
            )
        else:
            solver_if_statement = (
                solver_if_statement
                + "\telse if (solver_number == "
                + str(index)
                + ") \n"
                + "\t{\n"
                + "\t\tSolver"
                + str(index)
                + " solver; \n"
                + "\t\tptr_solver = std::unique_ptr<SolverBase>(std::move(solver.solver_"
                + name
                + ")); \n"
                + "\t} \n"
            )

        solvers = solvers + "\tSolver" + str(index) + " solver" + str(index) + ";\n"

        solvers_emplace = (
            solvers_emplace
            + "\tvector_ptr_solvers.emplace_back();\n"
            + "\tvector_ptr_solvers.back().reset(std::move(solver"
            + str(index)
            + ".solver_"
            + name
            + ")); \n"
        )

        index = index + 1

    # Write to the main interface header file
    h_file_interface.write(
        "// This file is automatically generated \n"
        + "// Time of creation:"
        + dt_string
        + "\n"
        + "#ifndef MPC_SOLVER_"
        + system.upper()
        + "_SOLVER_INTERFACE_H \n"
        + "#define MPC_SOLVER_"
        + system.upper()
        + "_SOLVER_INTERFACE_H \n\n"
        + "#include <memory> \n\n"
        + "#include <string> \n\n"
        + "#include <vector> \n\n"
        + "#include <mpc_base/solver_base.h> \n\n"
        + "std::unique_ptr<SolverBase> returnSolverPointer(int solver_number); \n\n"
        + "std::vector<std::unique_ptr<SolverBase>> returnSolverPointersVector(); \n\n"
        + "#endif"
    )

    # write to main cpp interface file
    cpp_file_interface.write(
        "// This file is automatically generated \n"
        + "// Time of creation:"
        + dt_string
        + "\n"
        + "#include <mpc_solver/"
        + system
        + "/"
        + system
        + "_solver_interface.h> \n\n"
        + header_files
        + "using namespace mpc_solver; \n\n"
        + "std::unique_ptr<SolverBase> returnSolverPointer(int solver_number) \n"
        + "{ \n"
        + "\tstd::unique_ptr<SolverBase> ptr_solver; \n"
        + solver_if_statement
        + "\treturn ptr_solver; \n"
        + "}\n\n"
        + "std::vector<std::unique_ptr<SolverBase>> returnSolverPointersVector() \n"
        + "{ \n"
        + "\tstd::vector<std::unique_ptr<SolverBase>> vector_ptr_solvers; \n\n"
        + solvers
        + "\n"
        + solvers_emplace
        + "\n"
        + "\treturn vector_ptr_solvers; \n"
        + "} \n"
    )

    cpp_file_interface.close()
    h_file_interface.close()
