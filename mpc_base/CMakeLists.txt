cmake_minimum_required(VERSION 2.8.3)
project(mpc_base)

add_compile_options(-std=c++11)
add_compile_options(-fstack-protector)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS mpc_msgs)

find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(INCLUDE_DIRS include/ ${EIGEN3_INCLUDE_DIRS})

# Include header files
include_directories(include/mpc_base ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
