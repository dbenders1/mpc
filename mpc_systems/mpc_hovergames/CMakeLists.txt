cmake_minimum_required(VERSION 3.0.2)
project(mpc_hovergames)

# ##############################################################################
# Check which system library to use This includes the cmake file from the
# platform we want
include(${PROJECT_SOURCE_DIR}/../system.cmake)

# Now check if we want to continue if this is the correct platform
if(SYSTEM STREQUAL "hovergames")
  message(... "Hovergames platform is chosen!" ...)
else()
  return()
endif()
# ##############################################################################

# System platform to use
set(PLATFORM "hovergames_simplesim")
# set(PLATFORM "hovergames_px4")
set(DEBUGGING FALSE)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../mpc_solver/src/cmake_globalvars.cmake)

if(NOT DEFINED LAYERS)
  message(
    FATAL_ERROR
      "Could not find LAYERS variable in the cmake_vars.cmake file! \n Please setup the solver first before building the packages!"
  )
endif(NOT DEFINED LAYERS)

# For reconfigure callback
if(LAYERS STREQUAL "1")
  add_definitions(-DSINGLE_LAYER)
else()
  add_definitions(-DDOUBLE_LAYER)
endif()

if(DEBUGGING)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -Wno-deprecated-declarations")
else()
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -DNDEBUG -Wno-deprecated-declarations"
  )
endif()

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             dynamic_reconfigure
             mpc_solver
             mpc_modules
             mpc_tools
             mpc_base
             mpc_core
             mavros_msgs
             sensor_msgs)

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
  roscpp
  mpc_solver
  mpc_modules
  mpc_tools
  mpc_base
  mpc_core
  dynamic_reconfigure
  mavros_msgs
  sensor_msgs
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME})

# Directories to include
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
                    ${PROJECT_NAME}_gencfg)

# Always compile hovergames drone interface
set(LIBRARY_SOURCES ${LIBRARY_SOURCES} src/hovergames_drone_interface.cpp)

if(PLATFORM STREQUAL "hovergames_px4")
  message("  ->  Using PX4 interface")
  add_definitions(-DPX4)
  set(LIBRARY_SOURCES ${LIBRARY_SOURCES} src/hovergames_px4_interface.cpp)

elseif(PLATFORM STREQUAL "hovergames_simplesim")
  message("  ->  Using simplesim interface")
  add_definitions(-DSIMPLESIM)
  set(LIBRARY_SOURCES ${LIBRARY_SOURCES} src/hovergames_simplesim_interface.cpp)
endif()

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
if(LAYERS STREQUAL "1")
  message("  ->  Using a single layer mpc")
  add_executable(mpc_control_node src/mpc_control_node.cpp)
  add_dependencies(mpc_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(mpc_control_node ${catkin_LIBRARIES} ${PROJECT_NAME})
else()
  message("  ->  Using a double layer mpc")
  add_executable(mpc_control_node_2_layer src/mpc_control_node_2_layer.cpp)
  add_dependencies(mpc_control_node_2_layer ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(mpc_control_node_2_layer ${catkin_LIBRARIES}
                        ${PROJECT_NAME})
endif()
