cmake_minimum_required(VERSION 3.0.2)
project(mpc_falcon)

# ##############################################################################
# Check which system library to use This includes the cmake file from the
# platform we want
include(${PROJECT_SOURCE_DIR}/../system.cmake)

# Now check if we want to continue if this is the correct platform
if(SYSTEM STREQUAL "falcon")
  message(... "Falcon platform is chosen!" ...)
else()
  return()
endif()
# ##############################################################################

set(DEBUGGING FALSE)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../mpc_solver/src/cmake_globalvars.cmake)

if(NOT DEFINED LAYERS)
  message(
    FATAL_ERROR
      "Could not find LAYERS variable in the cmake_vars.cmake file! \n Please setup the solver first before building the packages!"
  )
endif(NOT DEFINED LAYERS)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             mpc_base
             mpc_core
             mpc_modules
             mpc_solver
             mpc_tools
             roscpp)

find_package(Eigen3 REQUIRED)

# This includes the cmake file from the platform we want This file will be
# automatically generated when creating the solver and cpp files
include(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../mpc_solver/include/mpc_solver/${SYSTEM}/dynamic_reconfigure_files/generate_dynamic_reconfigure_options.cmake
)

# Generate the reconfigure callback api
generate_dynamic_reconfigure_options(${DYNAMIC_RECONFIGURE_CFG_FILES})

# Define the current package
catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  mpc_base
  mpc_core
  mpc_modules
  mpc_solver
  mpc_tools
  roscpp
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME})

# Directories to include
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
                    ${PROJECT_NAME}_gencfg)

# Always compile Falcon interface
set(LIBRARY_SOURCES ${LIBRARY_SOURCES} src/mpc.cpp src/falcon_interface.cpp)

# Add the sources and headers to the library
add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
# Add the dependency of created dynamic reconfigure configs
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
# Link the library to other libraries needed
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# Specify include directories to use when compiling given target
target_include_directories(${PROJECT_NAME} PUBLIC)

# Installation rules for the created library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Executable nodes
if(LAYERS STREQUAL "2")
  message("  ->  Using a double layer mpc")
  add_executable(mpc_control_node src/mpc_control_node.cpp)
  add_dependencies(mpc_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(mpc_control_node ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
