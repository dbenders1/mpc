include(${PROJECT_SOURCE_DIR}/../../mpc_solver/src/cmake_globalvars.cmake)

if(NOT DEFINED SYSTEM)
  message(
    FATAL_ERROR
      "Could not find SYSTEM variable in the cmake_globalvars.cmake file! \n Please setup the solver first before building the packages!"
  )
endif(NOT DEFINED SYSTEM)

# Needs a check to see if the system is supported correctly
