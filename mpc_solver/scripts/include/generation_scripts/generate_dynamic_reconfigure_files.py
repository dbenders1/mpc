
import datetime
import os


def generate_dynamic_reconfigure_files(solver_names, module_settings, system, pkg_path):
    
    # Path for files folder
    dynamic_reconfigure_files_path = pkg_path + "/include/mpc_solver/" + system + "/dynamic_reconfigure_files"
    if not os.path.exists(dynamic_reconfigure_files_path):
        os.makedirs(dynamic_reconfigure_files_path)

    # Show the file path
    print("Placing Dynamic recofigure files in: {}\n".format(dynamic_reconfigure_files_path))

    # Path for the files
    cmake_path = dynamic_reconfigure_files_path + "/generate_dynamic_reconfigure_options.cmake"
    header_path = dynamic_reconfigure_files_path + "/reconfigure_callback_options.h"


    # Date and Time
    now = datetime.datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")


    ## Store the parameters for the files
    cfgs_cmake = ""
    weight_load = ""
    type_config = ""
    cfgs = "#!/usr/bin/env python \n" +\
            "PACKAGE = \"mpc_" + system + "\" \n\n" +\
            "from dynamic_reconfigure.parameter_generator_catkin import * \n\n" +\
            "gen = ParameterGenerator() \n\n" +\
            "weight_param = gen.add_group(\"weights\", \"solver_constraints\") \n"

    index = 0
    cfgs_file = []
    yaml_file = []
    for name in solver_names:
        name = name[:-1]
        cfgs_temp = cfgs
        yaml_temp = "enable_debug_solver: false \n" +\
                    "print_solver_info: false \n" +\
                    "print_solver_output: false \n" +\
                    "print_solver_parameters: false\n"
        
        cfgs_cmake = cfgs_cmake + "\t${CMAKE_CURRENT_SOURCE_DIR}/../../mpc_solver/include/mpc_solver/"+ system +"/dynamic_reconfigure_files/"+ name +".cfg \n"

        config_weights = ""
        for idx, weight_name in enumerate(module_settings[index].params.params[0]['par_weights_names']):
            cfgs_temp = cfgs_temp + "weight_param.add(\""+ weight_name +"\",  double_t,   1,  \""+ weight_name +" weight\",  1.0, 0.0, 1000.0) \n"
            yaml_temp = yaml_temp + weight_name + ": 1.0 \n"
            config_weights = config_weights + "\tptr_solver->par_weights_["+str(idx)+"] = config."+weight_name+";\n"

        # Finish the cfg file
        cfgs_temp = cfgs_temp + "solver_debug = gen.add_group(\"Solver Debug Options\", \"solver_debug_options\") \n" +\
                    "solver_debug.add(\"enable_debug_solver\",     bool_t,     0,  \"Enable solver debug output\",           False) \n" +\
                    "solver_debug.add(\"print_solver_parameters\", bool_t,     0,  \"Enable printing of all parameters\",    True) \n" +\
                    "solver_debug.add(\"print_solver_info\",       bool_t,     0,  \"Enable printing of solver info\",       True) \n" +\
                    "solver_debug.add(\"print_solver_output\",     bool_t,     0,  \"Enable printing of inputs\",            True) \n" +\
                    "solver_debug.add(\"print_states\",            bool_t,     0,  \"Enable printing of states\",            True) \n\n" +\
                    "exit(gen.generate(PACKAGE, \"mpc_" + system + "\", \"" + name + "\")) \n\n"

        cfgs_file.append(open(dynamic_reconfigure_files_path + "/" + name + ".cfg", "w"))
        cfgs_file[index].write(cfgs_temp)
        cfgs_file[index].close()

        yaml_file.append(open(dynamic_reconfigure_files_path + "/" + name + "_template.yaml", "w"))
        yaml_file[index].write(yaml_temp)
        yaml_file[index].close()

        weight_load = weight_load + \
                "template <class TConfig> \n" + \
                "void loadMPCWeights"+ str(index) +"(TConfig &config, SolverBase* ptr_solver) \n" + \
                "{\n" + \
                config_weights + "\n" + \
                "\tptr_solver->loadWeightsParams();\n" + \
                "}\n \n"
        
        type_config = type_config + \
                "#include <mpc_" + system + "/" + name + "Config.h> \n\n" + \
                "typedef mpc_" + system + "::" + name + "Config solver_config" + str(index) + ";\n"

        index = index + 1

    
    # Finish cmake file
    cmake_file = open(cmake_path, "w")

    cmake_file.write("# This file is automatically generated \n" +\
                    "# Time of creation:" + dt_string + "\n" +\
                    "# Set the dynamic reconfigure callback cfg files \n" +\
                    "set(DYNAMIC_RECONFIGURE_CFG_FILES ${DYNAMIC_RECONFIGURE_CFG_FILES} \n" +\
                    cfgs_cmake +\
                    ") \n "
                    )
    cmake_file.close()

    reconfigure_callback_solver = open(header_path, "w")
    reconfigure_callback_solver.write("// This file is automatically generated \n" +
        "// Time of creation:" + dt_string + "\n" +
        "#ifndef MPC_SOLVER_" + system.upper() +"_DYNAMIC_RECONFIGURE_RECONFIGURE_CALLBACK_OPTIONS_H \n" +
        "#define MPC_SOLVER_" + system.upper() +"_DYNAMIC_RECONFIGURE_RECONFIGURE_CALLBACK_OPTIONS_H \n\n" +
        "#include <mpc_base/solver_base.h> \n\n" +
        " // The include(s) below is made by the dynamic reconfigure pkg in the main system folder \n" +
        " // But here is declared what the name of the include header(s) is/are \n" +
        type_config + "\n\n" +
        weight_load +
        "\n" +
        "#endif"
    )


    