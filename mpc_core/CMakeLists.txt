cmake_minimum_required(VERSION 3.0.2)
project(mpc_core)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs mpc_base mpc_msgs
                                        mpc_tools)

# Define the current package
catkin_package(
  CATKIN_DEPENDS
  mpc_base
  mpc_msgs
  mpc_tools
  roscpp
  sensor_msgs
  INCLUDE_DIRS
  include
  ${DECOMP_UTIL_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME})

# Directories to include
include_directories(include ${catkin_INCLUDE_DIRS})

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries ${catkin_LIBRARIES})

# Library sources
list(APPEND LIBRARY_SOURCES src/functions/update_mpc_configuration.cpp
     src/functions/error_function.cpp src/interface.cpp src/controller.cpp)

# Library headers
list(
  APPEND LIBRARY_HEADERS include/mpc_core/functions/update_mpc_configuration.h
  include/mpc_core/functions/error_function.h include/mpc_core/interface.h
  include/mpc_core/controller.h)

# Add the sources and headers to the library
add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
# Link the library to other libraries needed
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                      ${thirdparty_libraries})
# Specify include directories to use when compiling given target
target_include_directories(${PROJECT_NAME} PUBLIC include/)

# Installation rules for the created library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
