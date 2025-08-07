#ifndef ROBOT_REGION_H
#define ROBOT_REGION_H

#include <Eigen/Dense>
#include <vector>

class RobotRegion;

/**
 * @brief Models a robot region shaped as a disc. Positions are derived based on
 * offsets and the vehicle position.
 */
struct Disc {
 public:
  int id_;  // Disc ID for identification

  double radius_;  // Radius of a disc
  double offset_;

  double x_;
  double y_;

  /**
   * @brief Construct a disc from known parameters
   *
   * @param id ID
   * @param offset Offset
   * @param radius Radius
   */
  Disc(int id, const Eigen::Vector2d &pos, const double orientation,
       const double offset, const double radius)
      : id_(id), radius_(radius), offset_(offset) {
    SetPositionWithVehiclePosition(pos, orientation);
  }

  /** @brief Get the position of this disc as Eigen::Vector2d */
  Eigen::Vector2d AsVector2d() const { return Eigen::Vector2d(x_, y_); }

  /** @brief Compute the position of this disc based on the given position and
   * orientation of the vehicle */
  void SetPositionWithVehiclePosition(const Eigen::Vector2d &pos,
                                      double orientation);

  /** @brief Translate the position of this disc back to the given vehicle
   * center position */
  Eigen::Vector2d TranslateToVehicleCenter(const double orientation) const;

  /** @brief Set the position of this disc */
  void SetPosition(const Eigen::Vector2d &new_pos);

  double DiscX(const double &pose_x, const double &orientation) {
    return pose_x + offset_ * std::cos(orientation);
  }

  double DiscY(const double &pose_y, const double &orientation) {
    return pose_y + offset_ * std::cos(orientation);
  }

  operator std::string() const { return "disc_" + std::to_string(id_); }
};

class RobotRegionTemplate {
 public:
  /**
   * @brief Construct a new Vehicle Region Template object based on precomputed
   * offsets and radius
   *
   * @param n_discs The number of discs
   * @param offsets A vector of the offsets of each disc
   * @param radius The radius of each disc
   *
   * The output should not be used for position or orientation data, but as a
   * template for instanciating other vehicle regions.
   */
  RobotRegionTemplate(int n_discs, const std::vector<double> &offsets,
                      double radius);
  RobotRegionTemplate() {};

  double DiscRadius(int id = 0) const { return discs_[id].radius_; };

 public:
  std::vector<Disc> discs_;

  std::vector<double> offsets_;
};

/** @brief Models a vehicle region. Currently only supports a set of discs @see
 * Disc */
class RobotRegion : public RobotRegionTemplate {
 public:
  /** @brief Deprecated: Make a vehicle region using the width and length of the
   * vehicle */
  RobotRegion(
      const Eigen::Vector2d &pos, const double orientation, int n_discs,
      double width, double length,
      double center_offset);  // int n_discs, predictive_configuration *config);
                              // // SolverInterface *solver_interface,
                              // predictive_configuration *config);
  /** @brief Use a template to create a vehicle region at the given position /
   * orientation */
  RobotRegion(
      const Eigen::Vector2d &pos, const double orientation,
      const RobotRegionTemplate &
          vehicle_template);  // int n_discs, predictive_configuration *config);
                              // // SolverInterface *solver_interface,
                              // predictive_configuration *config);

  /** @brief Set the new position of this vehicle. Also updates the disc
   * positions */
  void SetPosition(const Eigen::Vector2d &new_pos);
  void SetPosition(const Eigen::Vector2d &new_pos, const double orientation);

 public:
  Eigen::Vector2d pos_;
  double orientation_;
};

#endif  // ROBOT_REGION_H
