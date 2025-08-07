#include "mpc_tools/robot_region.h"

Eigen::Vector2d Disc::TranslateToVehicleCenter(const double orientation) const {
  return Eigen::Vector2d(x_ - offset_ * std::cos(orientation),
                         y_ - offset_ * std::sin(orientation));
}

void Disc::SetPosition(const Eigen::Vector2d &new_pos) {
  x_ = new_pos(0);
  y_ = new_pos(1);
}

void Disc::SetPositionWithVehiclePosition(const Eigen::Vector2d &pos,
                                          double orientation) {
  x_ = pos(0) + offset_ * std::cos(orientation);
  y_ = pos(1) + offset_ * std::sin(orientation);
}

/* Vehicle */

RobotRegionTemplate::RobotRegionTemplate(int n_discs,
                                         const std::vector<double> &offsets,
                                         double radius) {
  offsets_ = offsets;

  for (int i = 0; i < n_discs; i++)
    discs_.emplace_back(i, Eigen::Vector2d(0., 0.), 0., offsets_[i], radius);
}

RobotRegion::RobotRegion(const Eigen::Vector2d &pos, const double orientation,
                         int n_discs, double width, double length,
                         double center_offset) {
  orientation_ = orientation;
  pos_ = pos;
  // Compute the offsets of the discs
  std::vector<double> offsets;
  offsets.resize(n_discs);

  // Compute radius of the discs
  double radius = width / 2;

  // Loop over discs and assign positions, with respect to center of mass
  for (int discs_it = 0; discs_it < n_discs; discs_it++) {
    if (n_discs == 1) {  // if only 1 disc, position in center;
      offsets[discs_it] = -center_offset + length / 2;
    } else if (discs_it ==
               0) {  // position first disc so it touches the back of the car
      offsets[discs_it] = -center_offset + radius;
    } else if (discs_it == n_discs - 1) {
      offsets[discs_it] = -center_offset + length - radius;
    } else {
      offsets[discs_it] = -center_offset + radius +
                          discs_it * (length - 2 * radius) / (n_discs - 1);
    }
  }
  // Create discs for the obstacle
  for (int i = 0; i < n_discs; i++)
    discs_.emplace_back(i, pos, orientation, offsets[i],
                        radius);  // i, offsets[i], radius);
  // discs_.emplace_back(i, offsets[i], radius, solver_interface->FORCES_N);
}

RobotRegion::RobotRegion(const Eigen::Vector2d &pos, const double orientation,
                         const RobotRegionTemplate &vehicle_template) {
  // Copy the template
  offsets_ = vehicle_template.offsets_;
  discs_ = vehicle_template.discs_;

  // Update the disc and vehicle positions
  SetPosition(pos, orientation);
}

void RobotRegion::SetPosition(const Eigen::Vector2d &new_pos) {
  for (auto &disc : discs_)
    disc.SetPositionWithVehiclePosition(new_pos, orientation_);

  pos_ = new_pos;
}

void RobotRegion::SetPosition(const Eigen::Vector2d &new_pos,
                              const double orientation) {
  for (auto &disc : discs_)
    disc.SetPositionWithVehiclePosition(new_pos, orientation);

  pos_ = new_pos;
  orientation_ = orientation;
}