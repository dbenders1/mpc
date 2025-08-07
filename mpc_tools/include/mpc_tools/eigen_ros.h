#ifndef MPC_TOOLS_ROS_EIGEN_H
#define MPC_TOOLS_ROS_EIGEN_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Eigen>
#include <boost/array.hpp>

// Template function to convert Eigen block or vector to boost::array<double, N>
template <typename EigenType, std::size_t N>
inline boost::array<double, N> eigenToBoostArray(
    const Eigen::MatrixBase<EigenType>& block) {
  boost::array<double, N> boostArray;
  for (std::size_t i = 0; i < N; ++i) {
    boostArray[i] = block(i);
  }
  return boostArray;
}

inline Eigen::Quaterniond fromRosQuaternion(
    const geometry_msgs::Quaternion quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

template <typename T>
inline Eigen::Vector3d fromRosVec3(const T& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

template <typename T>
inline geometry_msgs::Point toRosPoint(const T& vector) {
  geometry_msgs::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

template <typename T>
inline geometry_msgs::Quaternion toRosQuaternion(const T& quaternion) {
  geometry_msgs::Quaternion ros_quaternion;
  ros_quaternion.w = quaternion.w();
  ros_quaternion.x = quaternion.x();
  ros_quaternion.y = quaternion.y();
  ros_quaternion.z = quaternion.z();
  return ros_quaternion;
}

template <typename T>
inline geometry_msgs::Vector3 toRosVector(const T& vector) {
  geometry_msgs::Vector3 vec;
  vec.x = vector.x();
  vec.y = vector.y();
  vec.z = vector.z();
  return vec;
}

#endif  // MPC_TOOLS_ROS_EIGEN_H
