cmake_minimum_required(VERSION 3.0.2)
project(mpc_modules)

# Include the global variables created by the solver generation
include(${PROJECT_SOURCE_DIR}/../mpc_solver/src/cmake_globalvars.cmake)

# Check modules that are needed. These are defined in the cmake file generate
# upon creating the solver
if(NOT DEFINED MODULES)
  message(
    FATAL_ERROR
      "Could not find MODULES variable in the cmake_globalvars.cmake file! \n Please setup the solver first before building the packages!"
  )
endif(NOT DEFINED MODULES)

# Compile options
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall
# -Wno-deprecated-declarations") # without optimization set(CMAKE_CXX_FLAGS
# "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -DNDEBUG -fno-math-errno
# -fno-trapping-math -Wno-deprecated-declarations") # previously used Notes: -
# -O3 has the biggest influence on code speedup, the other arguments are not as
# important - -DNDEBUG is used to remove all assert statements in the code, if
# you are testing code don't use this flag!! - -fno-math-errno
# -fno-trapping-math is used to remove the math error handling, this can speed
# up the code. However, take care when using in combination with -O3 - Another
# compile option for even faster decomposition: -ffast-math

# To perform map pre-processing (DECOMP_OLD=FALSE) or not (DECOMP_OLD=TRUE)
set(DECOMP_OLD FALSE)
if(DECOMP_OLD)
  add_definitions(-DDECOMP_OLD)
endif()

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS decomp_util
             geometry_msgs
             mpc_msgs
             mpc_tools
             mpc_base
             nav_msgs
             roscpp)

# Finding external packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Define the current package
catkin_package(
  CATKIN_DEPENDS
  decomp_util
  geometry_msgs
  mpc_msgs
  mpc_tools
  mpc_base
  nav_msgs
  roscpp
  INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME})

# Directories to include
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES}
     ${OpenCV_LIBS})

# use #define option in module loader to define which header we need to include
# These can be used in the code with for example definition -Dfoo: #ifdef foo
# -code- #endif You can manage code that needs to be compiled. to define these
# macros in the cmakelist for example: #define foo add_definitions(-Dfoo)

message("---- MODULES SELECTED: ----")
foreach(MODULE IN LISTS MODULES)
  if(${MODULE} STREQUAL "GoalOriented")
    message("  -> ${MODULE}")
    add_definitions(-Dgoal_oriented)
    list(APPEND LIBRARY_SOURCES src/objectives/goal_oriented.cpp)
    list(APPEND LIBRARY_HEADERS include/mpc_modules/objectives/goal_oriented.h)
  elseif(${MODULE} STREQUAL "ReferenceTrajectory")
    message("  -> ${MODULE}")
    add_definitions(-Dreference_trajectory)
    list(APPEND LIBRARY_SOURCES src/objectives/reference_trajectory.cpp)
    list(APPEND LIBRARY_HEADERS
         include/mpc_modules/objectives/reference_trajectory.h)
  elseif(${MODULE} STREQUAL "StaticPolyhedronConstraints")
    message("  -> ${MODULE}")
    add_definitions(-Dstatic_polyhedron_constraints)
    list(APPEND LIBRARY_SOURCES
         src/constraints/static_polyhedron_constraints.cpp)
    list(APPEND LIBRARY_HEADERS
         include/mpc_modules/constraints/static_polyhedron_constraints.h)
  endif()
endforeach()

# Library sources used by all modules
list(APPEND LIBRARY_SOURCES src/loader/module_handler.cpp)

# Library headers used by all modules
list(APPEND LIBRARY_HEADERS include/mpc_modules/loader/module_handler.h)

# Add the sources and headers to the library
add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
# Link the library to other libraries needed
target_link_libraries(${PROJECT_NAME} ${thirdparty_libraries})
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
