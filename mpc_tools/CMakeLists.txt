cmake_minimum_required(VERSION 2.8.3)
project(mpc_tools)

add_compile_options(-std=c++11)
add_compile_options(-fstack-protector)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(DEBUGGING FALSE)

if(DEBUGGING)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -Wno-deprecated-declarations")
  add_definitions(-DOLD_DECOMP)
else()
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -DNDEBUG -Wno-deprecated-declarations"
  )
endif()

find_package(
  catkin REQUIRED
  COMPONENTS eigen_conversions
             geometry_msgs
             roscpp
             roslib
             std_msgs
             tf
             tf_conversions
             visualization_msgs)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

catkin_package(
  CATKIN_DEPENDS
  eigen_conversions
  geometry_msgs
  roscpp
  roslib
  std_msgs
  tf
  tf_conversions
  visualization_msgs
  DEPENDS
  Boost
  INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME})

# Include header files
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

# Library sources
list(
  APPEND
  LIBRARY_SOURCES
  src/eigen_std.cpp
  src/ros_visuals.cpp
  src/data_saver_json.cpp
  src/benchmarker.cpp
  src/robot_region.cpp
  src/instrumentation_timer.cpp
  src/ROS_timer_handler.cpp
  src/ThreadPool.cpp
  src/traj_gen.cpp)

list(
  APPEND
  LIBRARY_HEADERS
  include/mpc_tools/eigen_ros.h
  include/mpc_tools/eigen_std.h
  include/mpc_tools/math.h
  include/mpc_tools/ros_visuals.h
  include/mpc_tools/data_saver_json.h
  include/mpc_tools/benchmarker.h
  include/mpc_tools/robot_region.h
  include/mpc_tools/instrumentation_timer.h
  include/mpc_tools/RK4.h
  include/mpc_tools/ROS_timer_handler.h
  include/mpc_tools/ThreadPool.h
  include/mpc_tools/traj_gen.h)

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
