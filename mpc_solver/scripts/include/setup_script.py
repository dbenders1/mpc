#!/usr/bin/env python3

import argparse
import importlib
import os
import sys

import yaml

file_loaction = os.path.dirname(os.path.abspath(__file__))
scripts_location = os.path.abspath(os.path.join(__file__, "../../"))

sys.path.append(scripts_location)
sys.path.append(scripts_location + "/systems")
sys.path.append(scripts_location + "/include")


from generation_scripts import (
    generate_cmake,
    generate_dynamic_reconfigure_files,
    generate_files,
    generate_solver,
)


def create_solver(
    solver_names, system, dir_path, build_solver, use_floating, floating_platform
):
    module_setting = []

    k = 0
    for name in solver_names:

        print(
            "----------------------------------------------------\n"
            + " Setting up solver for "
            + name
            + "\n"
        )

        module_setting.append(importlib.import_module("." + name + "settings", system))

        # Create the choosen system solver
        if build_solver:
            generate_solver.create_solver(
                name,
                system,
                dir_path,
                module_setting[k],
                use_floating,
                floating_platform,
            )

        k = k + 1
        print(" Done generating solver!")

    print("----------------------------------------------------\n")
    print("All Done with solver(s) generation!")
    print("----------------------------------------------------\n")
    return module_setting


def create_cpp(settings, Names, platform):

    i = 0
    for name in Names:
        print("Creating solver interface for solver name: " + str(name) + "\n")
        generate_files.generate_cpp_files(name, settings[i], platform)

        i = i + 1

        print("----------------------------------------------------\n")


def check_config(
    system, build_solver, build_solver_interface, number_solvers, setting_files
):
    # Check if the config file was correct format
    if not isinstance(build_solver, bool) or not isinstance(
        build_solver_interface, bool
    ):
        print(
            "[ERROR]: build_solver and build_solver_interface should be true or false statement! \n"
        )
        exit()
    if not isinstance(number_solvers, int):
        print("[ERROR]: number_of_solvers should be an integer! \n")
        exit()

    try:
        dir_system = dir_package + "/scripts/systems/" + system
    except:
        print(
            "[ERROR]: system should be a string containing one of the available systems! \n"
        )
        exit()

    if not os.path.isdir(dir_system):
        print("[ERROR]: set system is not one of the available systems! \n")
        exit()

    try:
        dir_setting_files = []
        solver_names = []
        number_of_setting_files = 0

        for file in setting_files:
            dir_setting_files.append(dir_system + "/" + file)
            solver_names.append(file[:-11])
            if not os.path.isfile(dir_setting_files[number_of_setting_files]):
                print("[ERROR]: " + file + " is not available! \n")
                exit()
            number_of_setting_files = number_of_setting_files + 1
    except:
        print(
            "[ERROR]: something wrong in the configuration.yaml file or in the system directory!\n"
        )
        exit()

    if number_solvers is not len(setting_files):
        print("[ERROR]: length of array is not consistent with number_of_solvers! \n")
        exit()

    return solver_names


def check_floating(use_floating, floating_platform):
    if use_floating and floating_platform == 1:
        print("\n Making solver for other device with X86 structure. \n")
    elif use_floating and floating_platform == 2:
        print("\n Making solver for other device with Jetson structure (ARM based). \n")
    elif use_floating:
        print(
            "\n Chosen the option for a floating license, but not a correct platform is specified. \
              \n Aborting script! \n"
        )

    print("\n Making solver for this device. \n")


if __name__ == "__main__":
    # Define the command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", "--config", nargs="+", help="One or more configuration files"
    )
    parser.add_argument("-s", "--system", required=True, help="System argument")
    parser.add_argument(
        "-f",
        "--floating_license_platform",
        required=False,
        help="Floating license platform argument",
    )
    parser.add_argument(
        "-b", "--build_solver", required=True, help="Build solver argument"
    )

    # Parse the command line arguments
    args = parser.parse_args()

    # Print the arguments
    print("Config files: ", args.config)
    print("System argument: ", args.system)
    if args.floating_license_platform:
        print("Floating license platform argument: ", args.floating_license_platform)

    print(
        "----------------------------------------------------\n \n"
        + "Importing the settings file from the package config directory \n \n"
        + "----------------------------------------------------\n \n"
    )

    # Find current directory path and package path
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print("Current directory path: " + dir_path + "\n")
    temp = dir_path.rsplit("/scripts")
    dir_package = temp[0]
    print("Package directory path: " + dir_package + "\n")

    system = args.system
    if args.build_solver == "true":
        build_solver = True
    else:
        build_solver = False
    build_solver_interface = True
    number_solvers = len(args.config)
    setting_files = args.config
    if args.floating_license_platform:
        use_floating = True
        if args.floating_license_platform == "X86":
            floating_platform = 1
        elif args.floating_license_platform == "ARM":
            floating_platform = 2
    else:
        use_floating = False
        floating_platform = ""

    print(
        build_solver,
        build_solver_interface,
        number_solvers,
        setting_files,
        use_floating,
        floating_platform,
    )

    solver_names = check_config(
        system, build_solver, build_solver_interface, number_solvers, setting_files
    )
    check_floating(use_floating, floating_platform)

    print(
        "----------------------------------------------------\n\n"
        + "Configuration file is checked and has the correct format! \n\n"
        + "----------------------------------------------------\n\n"
    )

    # Creating solver if defined
    print("Starting creation of the Forces Pro solver: \n \n")
    module_setting = create_solver(
        solver_names, system, dir_package, build_solver, use_floating, floating_platform
    )

    print("----------------------------------------------------\n")

    # Creating solver interface files if defined
    if build_solver_interface:
        print("Starting creation of the solver interface cpp files: \n \n")
        generate_files.generate_cpp_files(
            solver_names, module_setting, system, dir_package
        )
        generate_cmake.generate_cmake_files(
            solver_names, module_setting, system, dir_package
        )
        generate_dynamic_reconfigure_files.generate_dynamic_reconfigure_files(
            solver_names, module_setting, system, dir_package
        )

    else:
        print("Not creating the cpp files for the solver interface")

    print("----------------------------------------------------\n \n")

    print(" All Done!!")
    print(" Happy Solving...")
