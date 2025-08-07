cmake_minimum_required(VERSION 3.0.2)
project(mpc_solver)

# System platform to use
set(PLATFORM "falcon")
# set(PLATFORM "hovergames")

set(DEBUGGING FALSE)

if(DEBUGGING)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -Wno-deprecated-declarations")
else()
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -DNDEBUG -Wno-deprecated-declarations"
  )
endif()

# Compiler definitions, these can be used in the code as #ifdef. You can manage
# code that needs to be compiled.
add_definitions(-DUSETEST -Dtest2)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mpc_base)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

# Define the current package
catkin_package(CATKIN_DEPENDS mpc_base INCLUDE_DIRS include LIBRARIES
               ${PROJECT_NAME})

# Directories to include
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# This includes the cmake file from the platform we want This file will be
# automatically generated when creating the solver and cpp files
include(${PROJECT_SOURCE_DIR}/src/${PLATFORM}/sources_list.cmake)

# Set the core library
set(LIBRARY_CORE_HEADERS
    ${LIBRARY_CORE_HEADERS} include/mpc_solver/core/helper_functions.h
    include/mpc_solver/core/solver.h)

set(LIBRARY_CORE_SOURCES ${LIBRARY_CORE_SOURCES} src/core/helper_functions.cpp)

# Add the sources and headers to the library
add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_CORE_HEADERS}
                            ${LIBRARY_HEADERS} ${LIBRARY_CORE_SOURCES})
# Link the library to other libraries needed
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SOLVER_LIBS})
# Specify include directories to use when compiling given target
# target_include_directories(${PROJECT_NAME} PUBLIC include/ )

# Installation rules for the created library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
