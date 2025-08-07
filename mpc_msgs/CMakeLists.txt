cmake_minimum_required(VERSION 2.8.3)
project(mpc_msgs)

# Find message packages from catkin to build the new messages
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation
                                        std_msgs)

# Add message files and location
add_message_files(
  DIRECTORY
  msg
  FILES
  ConstraintsOverHorizon.msg
  Float64Array.msg
  GoalPose.msg
  InterpData.msg
  MpcHeader.msg
  ReferenceTrajectoryConstraints.msg
  ScalarOverHorizon.msg
  StartMpcLayer.msg
  TrajectoryOverHorizon.msg)

# Generate the new messages
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

# Create a new catkin package from this project
catkin_package(DEPENDS geometry_msgs message_runtime std_msgs)
