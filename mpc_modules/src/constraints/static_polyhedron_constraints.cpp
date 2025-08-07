// Include the header
#include "mpc_modules/constraints/static_polyhedron_constraints.h"

// Include other headers that are needed for this file
#include <decomp_util/decomp_geometry/geometric_utils.h>
#include <geometry_msgs/Point.h>
#include <mpc_tools/printing.h>

#include <algorithm>  // std::find, std::sort
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>  //for debugging only!
#include <opencv2/imgproc.hpp>

StaticPolyhedronConstraints::StaticPolyhedronConstraints(
    std::string name, ros::NodeHandle &nh, ConfigurationMPC *ptr_config,
    SolverBase *ptr_solver, DataSaverJson *ptr_data_saver_json,
    RealTimeData &data)
    : ControllerModule(ModuleType::CONSTRAINT, name, nh, ptr_config, ptr_solver,
                       ptr_data_saver_json) {
  MPC_WARN_ALWAYS("Initializing " << name_ << " module");

  // Set random seed
  std::srand(ptr_config_->random_seed_);

  // Constraints variables
  // NOTE: this assumes the initial state is not constrained (in total N
  // circular/ellipsoidal paths are constructed), but stored from the previous
  // run for visualization
  n_stages_to_construct_ = ptr_solver_->n_;
  n_constraints_per_stage_ = ptr_config_->n_constraints_per_stage_;
  constraints_ = Eigen::MatrixXd::Zero(3 * n_constraints_per_stage_,
                                       n_stages_to_construct_ + 1);
  n_relevant_constraints_ = Eigen::VectorXi::Zero(n_stages_to_construct_ + 1);

  // ROS recording and visualization variables
  n_stages_to_rec_and_vis_ = ptr_solver_->n_ + 1;
  constraints_to_rec_and_vis_ = Eigen::MatrixXd::Zero(
      3 * n_constraints_per_stage_, n_stages_to_rec_and_vis_);
  n_relevant_constraints_to_rec_and_vis_ =
      Eigen::VectorXi::Zero(n_stages_to_rec_and_vis_);
  if (ptr_config_->record_constraints_) {
    constraints_rec_pub_ = nh.advertise<mpc_msgs::ConstraintsOverHorizon>(
        ptr_config_->constraints_rec_topic_, 1);
    constraints_msg_.header.frame_id = ptr_config_->robot_target_frame_;
  }
  vertices_to_vis_.reserve(n_constraints_per_stage_ + 1);

  // Obtain data positions
  std::vector<int> idc =
      ptr_solver_->returnDataPositionsState({"x", "y", "psi"});
  x_position_ = idc[0];
  y_position_ = idc[1];
  psi_position_ = idc[2];

  idc = ptr_solver_->returnDataPositionsConstrain({"disc0_0_a1"});
  disc0_0_a1_position_ = idc[0];

  // Only initialize the polyhedron constraints generator in single-layer or
  // highest-layer MPC
  if (ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1) {
    // Receive occupancy grid map via ROS topic
    // Receive state via solver_data in OnDataReceived

    status_ = StaticPolyhedronConstraintsStatus::RESET;

    pred_traj_.poses.resize(n_stages_to_construct_ + 1);
    line_segments_updated_ = false;

    occupied_threshold_ = ptr_config_->occupied_threshold_;
    map_processed_ = false;

    // Determine how to construct paths for circular/ellipsoidal decomposition
    // Note: the initial state constraints is not applied to the solver
    switch (ptr_config_->line_segment_to_stage_) {
      // Constraints based on line segment after stage
      // => ellipsoidal decomposition for stages 1-N-1
      // => circular    decomposition for stage  N
      case -1:
        n_path_ =
            n_stages_to_construct_ +
            1;  // one extra stage needed, because line segments lie in between
        break;

      // Constraints based on circle around stage
      // => circular       decomposition for stage 0-N
      // => no ellipsoidal decomposition
      case 0:
        n_path_ =
            2 * n_stages_to_construct_;  // each stage needs two points in space
        break;

      // Constraints based on line segment before stage
      // => ellipsoidal decomposition for stages 1-N
      // => no circular decomposition
      case 1:
        n_path_ =
            n_stages_to_construct_ +
            1;  // one extra stage needed, because line segments lie in between
        break;

      default:
        MPC_WARN(
            "StaticPolyhedronConstraints: Unknown value for "
            "'ptr_config_->line_segment_to_stage_' (should be in {-1,0,1}). "
            "Setting 1 as value");
        ptr_config_->line_segment_to_stage_ = 1;
        n_path_ =
            n_stages_to_construct_ +
            1;  // one extra stage needed, because line segments lie in between
        break;
    }
    // Do not reserve memory for path_ and poly_constraints_ -> this gives
    // runtime errors or malfunctioning behaviour, probably caused by the
    // dynamically-sized Eigen matrices and vectors

    // Only look around for obstacles using a box with sides of width
    // 2*bounding_box_width_
    decomp_util_.set_local_bbox(Vec2f(ptr_config_->bounding_box_width_,
                                      ptr_config_->bounding_box_width_));

    ros_markers_.reset(new ROSMarkerPublisher(
        nh, ptr_config_->constraints_vis_topic_.c_str(),
        ptr_config_->robot_target_frame_,
        (ptr_solver_->n_ + 2) *
            (2 * ptr_config_->n_constraints_per_stage_ + 1)));

    local_map_sub_ =
        nh.subscribe(ptr_config_->occ_grid_topic_, 1,
                     &StaticPolyhedronConstraints::OccGridMapCallBack, this);

    // This is the radius around the bounding box that is used to determine the
    // local map distance
    radius_around_bbox_ =
        0.5 * std::sqrt(2 * 2 * ptr_config_->bounding_box_width_ * 2 *
                        ptr_config_->bounding_box_width_);

    occ_pos_vec_stages_.resize(n_path_ - 1);

    // Create publisher local gridmap
    local_occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(
        ptr_config_->local_occ_grid_topic_, 1);
  } else {
    // Memory for received_constraints_ and received_n_relevant_constraints_ can
    // only be reserved when first receiving the constraints, since size is
    // unknown here

    // Set validity of real-time data to zero until valid data is received for
    // all lower-layer MPCs
    n_it_valid_ = 0;
    n_it_valid_max_ = 0;

    ros_markers_.reset(new ROSMarkerPublisher(
        nh, ptr_config_->constraints_vis_topic_.c_str(),
        ptr_config_->robot_target_frame_,
        (ptr_solver_->n_ + 1) *
            (2 * ptr_config_->n_constraints_per_stage_ + 1)));
  }

  MPC_WARN_ALWAYS(name_ << " module initialized");
}

void StaticPolyhedronConstraints::OnDataReceived(RealTimeData &data,
                                                 std::string data_name) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::OnDataReceived");

  if (ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1) {
    return;
  }

  // Save correct range of received_constraints to use + reduce real-time data
  // validity for all lower-layer MPCs
  if (data_name == "Constraints") {
    // When receiving constraints for the first time, make sure to allocate the
    // proper amount of memory for the received constraints
    if (!first_constraints_received_) {
      received_n_stages_ = data.constraints_.cols();
      received_constraints_ = Eigen::MatrixXd::Zero(
          3 * n_constraints_per_stage_, received_n_stages_);
      received_n_relevant_constraints_ =
          Eigen::VectorXi::Zero(received_n_stages_);
      first_constraints_received_ = true;
    }

    // Save received constraints in received_constraints_
    for (int k = 0; k < received_n_stages_; k++) {
      received_constraints_.col(k) = data.constraints_.col(k);
      received_n_relevant_constraints_(k) = data.n_relevant_constraints_(k);
    }

    // Set data validity
    // NOTE: initially, this is the same as the MPC layer ratio
    n_it_valid_max_ = data.constraints_n_it_valid_;
    n_it_valid_ = n_it_valid_max_;
  }
}

bool StaticPolyhedronConstraints::ReadyForControl(const RealTimeData &data) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::ReadyForControl");

  // Check for real-time data validity for all lower-layer MPCs in multi-layer
  if (!(ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1)) {
    if (n_it_valid_ > 0) {
      return true;
    };

    MPC_WARN("StaticPolyhedronConstraints: Constraints data is not valid!");
    return false;
  }

  // Now further with single-layer or highest-layer MPC
  if (map_processed_) {
    return true;
  };

  MPC_WARN("StaticPolyhedronConstraints: Map has not been updated yet!");
  return false;
}

void StaticPolyhedronConstraints::Update(RealTimeData &data) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::Update");

  // Compute constraints and save in real-time data in highest-layer MPC
  if (ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1) {
    // First shift the constraints and number of relevant constraints, such that
    // the constraints for stage 0 (not used in the solver) are set equal to the
    // constraints of stage 1
    constraints_.col(0) = constraints_.col(1);
    n_relevant_constraints_(0) = n_relevant_constraints_(1);

    // Compute line segments and constraints
    // NOTE: updated state should already be ready to this function call
    status_ = StaticPolyhedronConstraintsStatus::BUSY;
    if (!ComputeLineSegments()) {
      MPC_ERROR(
          "StaticPolyhedronConstraints: Line segments computation failed!");
    }
    if (ComputeConstraints()) {
      status_ = StaticPolyhedronConstraintsStatus::SUCCESS;
    } else {
      MPC_ERROR("StaticPolyhedronConstraints: Constraints computation failed!");
      status_ = StaticPolyhedronConstraintsStatus::FAILURE;
    }

    // Initially, set the constraints for stage 0 equal to the constraints of
    // stage 1
    if (!first_constraints_constructed_) {
      n_relevant_constraints_(0) = n_relevant_constraints_(1);
      for (int l = 0; l < n_relevant_constraints_(0); l++) {
        constraints_(3 * l, 0) = poly_constraints_[0].A_.row(l)[0];
        constraints_(3 * l + 1, 0) = poly_constraints_[0].A_.row(l)[1];
        constraints_(3 * l + 2, 0) = poly_constraints_[0].b_(l);
      }
      first_constraints_constructed_ = true;
    }

    // Save constraints (excluding stage 0)
    for (int k = 1; k < n_stages_to_construct_ + 1; k++) {
      for (int l = 0; l < n_relevant_constraints_(k); l++) {
        constraints_(3 * l, k) = poly_constraints_[k - 1].A_.row(l)[0];
        constraints_(3 * l + 1, k) = poly_constraints_[k - 1].A_.row(l)[1];
        constraints_(3 * l + 2, k) = poly_constraints_[k - 1].b_(l);
      }

      // If more constraints fields are given in the solver: insert dummies (x
      // constraints far away, see above)
      for (int l = n_relevant_constraints_(k); l < n_constraints_per_stage_;
           l++) {
        constraints_(3 * l, k) = 1;
        constraints_(3 * l + 1, k) = 0;
        // Note: using stage_vars_ of previous run, so need account for stage
        // index N here
        if (k == n_stages_to_construct_)
          constraints_(3 * l + 2, k) =
              ptr_solver_->stage_vars_(x_position_, k) + 100;
        else
          constraints_(3 * l + 2, k) =
              ptr_solver_->stage_vars_(x_position_, k + 1) + 100;
      }

      // Save in real-time data
      if (ptr_config_->layer_idx_ == 1) {
        data.constraints_.col(k - 1) = constraints_.col(k);
        data.n_relevant_constraints_(k - 1) = n_relevant_constraints_(k);
      }
    }
  }
  // Process received constraints and reduce received data validity for all
  // lower-layer MPCs
  else if (ptr_config_->layer_idx_ < ptr_config_->n_layers_ - 1) {
    int constraint_idx;
    for (int k = 0; k < n_stages_to_construct_ + 1; k++) {
      constraint_idx = n_it_valid_max_ - n_it_valid_ + k;
      constraints_.col(k) = received_constraints_.col(constraint_idx);
      n_relevant_constraints_(k) =
          received_n_relevant_constraints_(constraint_idx);
    }

    if (n_it_valid_ > 0) n_it_valid_ -= 1;
  }
}

void StaticPolyhedronConstraints::TightenConstraints(
    Eigen::VectorXd tightening_pred) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::TightenConstraints");

  // Tighten relevant constraints
  // NOTE: only tighten the constructed constraints, since these stages will not
  // be communicated to the interface
  for (int k = 1; k < n_stages_to_construct_ + 1; k++) {
    for (int l = 0; l < n_relevant_constraints_(k); l++) {
      constraints_(3 * l + 2, k) =
          constraints_(3 * l + 2, k) - tightening_pred(k);
    }
  }
}

void StaticPolyhedronConstraints::SetParameters(const RealTimeData &data) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints: Inserting constraints");

  // Insert constraints: amount of constraints given by n_constraints_per_stage_
  // For stage 0, insert constraints that the solver does not like as check for
  // correct usage of constraints over the horizon
  ptr_solver_->par_constraints_.block(disc0_0_a1_position_, 0,
                                      3 * n_constraints_per_stage_, 1) =
      Eigen::VectorXd::Zero(3 * n_constraints_per_stage_);
  for (int k = 1; k < ptr_solver_->n_ + 1; k++) {
    ptr_solver_->par_constraints_.block(disc0_0_a1_position_, k,
                                        3 * n_constraints_per_stage_, 1) =
        constraints_.col(k);
  }
}

void StaticPolyhedronConstraints::SetConstraintsToRecAndVis(
    const Eigen::MatrixXd &constraints,
    const Eigen::VectorXi &n_relevant_constraints) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::SetConstraints");

  use_ext_con_to_rec_and_vis_ = true;

  // Reserve memory for given the known amount of stages for which to visualize
  // the constraints
  n_stages_to_rec_and_vis_ = constraints.cols();
  constraints_to_rec_and_vis_ = Eigen::MatrixXd::Zero(
      3 * n_constraints_per_stage_, n_stages_to_rec_and_vis_);
  n_relevant_constraints_to_rec_and_vis_ =
      Eigen::VectorXi::Zero(n_stages_to_rec_and_vis_);

  // Store constraints to visualize
  for (int k = 0; k < n_stages_to_rec_and_vis_; k++) {
    constraints_to_rec_and_vis_.col(k) = constraints.col(k);
    n_relevant_constraints_to_rec_and_vis_(k) = n_relevant_constraints(k);
  }
}

void StaticPolyhedronConstraints::PublishData(
    const mpc_msgs::MpcHeader &mpc_header) {
  if (ptr_config_->record_constraints_) {
    PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::PublishData");

    constraints_msg_.header.stamp = ros::Time::now();
    constraints_msg_.mpc_header = mpc_header;
    constraints_msg_.con.resize(3 * n_constraints_per_stage_ *
                                n_stages_to_rec_and_vis_);
    constraints_msg_.n_relevant_con.resize(n_stages_to_rec_and_vis_);
    for (int k = 0; k < n_stages_to_rec_and_vis_; k++) {
      if (use_ext_con_to_rec_and_vis_) {
        for (int l = 0; l < n_constraints_per_stage_; l++) {
          for (int m = 0; m < 3; m++)
            constraints_msg_.con[k * n_constraints_per_stage_ * 3 + l * 3 + m] =
                constraints_to_rec_and_vis_(3 * l + m, k);
        }
        constraints_msg_.n_relevant_con[k] =
            n_relevant_constraints_to_rec_and_vis_(k);
      } else {
        for (int l = 0; l < n_constraints_per_stage_; l++) {
          for (int m = 0; m < 3; m++)
            constraints_msg_.con[k * n_constraints_per_stage_ * 3 + l * 3 + m] =
                constraints_(3 * l + m, k);
        }
        constraints_msg_.n_relevant_con[k] = n_relevant_constraints_(k);
      }
    }

    constraints_rec_pub_.publish(constraints_msg_);
  }
}

void StaticPolyhedronConstraints::CreateVisualizations() {
  if (!ptr_config_->draw_constraints_) {
    return;
  }

  if (use_ext_con_to_rec_and_vis_)
    CreateVisualizations(constraints_to_rec_and_vis_,
                         n_relevant_constraints_to_rec_and_vis_,
                         n_stages_to_rec_and_vis_);
  else
    CreateVisualizations(constraints_, n_relevant_constraints_,
                         n_stages_to_construct_ + 1);
}

void StaticPolyhedronConstraints::CreateVisualizations(
    const Eigen::MatrixXd &constraints,
    const Eigen::VectorXi &n_relevant_constraints, int max_n_stages) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::CreateVisualizations");

  ROSLine &line = ros_markers_->getNewLine();
  line.setScale(ptr_config_->constraints_marker_scale_[0]);

  geometry_msgs::Point p1, p2;

  ROSPointMarker &vts_markers = ros_markers_->getNewPointMarker("CYLINDER");
  vts_markers.setScale(ptr_config_->constraints_marker_scale_[0],
                       ptr_config_->constraints_marker_scale_[1],
                       ptr_config_->constraints_marker_scale_[2]);

  int n_idc = (int)ptr_config_->indices_to_draw_.size();
  size_t n_verts;

  double p_value = ptr_config_->constraints_marker_z_;

  for (int k = 0; k < n_idc; k++) {
    int &index = ptr_config_->indices_to_draw_[k];
    if ((int)index >= max_n_stages) {
      MPC_WARN_STREAM("Index to draw ("
                      << index << ") exceeds polyhedra array size ("
                      << max_n_stages << "). Ignoring this index");
      continue;
    }
    if ((ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1) &&
        (ptr_config_->n_layers_ > 1)) {
      line.setColorInt(k, n_idc,
                       ptr_config_->constraints_marker_alpha_hierarchical_);
      vts_markers.setColorInt(
          k, n_idc, ptr_config_->constraints_marker_alpha_hierarchical_);
    } else {
      line.setColorInt(k, n_idc);
      vts_markers.setColorInt(k, n_idc);
    }

    // Compute vertices
    vertices_to_vis_.clear();
    ComputeVertices(
        constraints.block(0, index, 3 * n_relevant_constraints(index), 1));

    // Create lines between consecutive vertices
    p1.z = (n_idc - 1 - k) * p_value;
    p2.z = (n_idc - 1 - k) * p_value;

    n_verts = vertices_to_vis_.size();
    for (size_t i = 0; i < n_verts; i++) {
      if (i == 0) {
        p1.x = vertices_to_vis_[n_verts - 1](0);
        p1.y = vertices_to_vis_[n_verts - 1](1);
      } else {
        p1.x = vertices_to_vis_[i - 1](0);
        p1.y = vertices_to_vis_[i - 1](1);
      }

      p2.x = vertices_to_vis_[i](0);
      p2.y = vertices_to_vis_[i](1);

      line.addLine(p1, p2);

      if (ptr_config_->draw_vertices_) {
        vts_markers.addPointMarker(p1);
      }
    }
  }
}

void StaticPolyhedronConstraints::PublishVisualizations() {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::PublishVisualizations");

  ros_markers_->publish();
}

void StaticPolyhedronConstraints::OnReset() {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::OnReset");

  // Only execute this reset code in single-layer or highest-layer MPC
  if (ptr_config_->layer_idx_ == ptr_config_->n_layers_ - 1) {
    status_ = StaticPolyhedronConstraintsStatus::RESET;
    first_constraints_constructed_ = false;
  }
  // Execute this reset code in all lower-layer MPCs
  else {
    first_constraints_received_ = false;
    n_it_valid_ = 0;
  }
}

void StaticPolyhedronConstraints::ExportDataJson(const int count_since_start,
                                                 const int count_total) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::ExportDataJson");
}

// Callback methods
void StaticPolyhedronConstraints::OccGridMapCallBack(
    nav_msgs::OccupancyGrid::ConstPtr occ_grid_map_ptr) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::OccGridMapCallBack");

  occ_grid_map_ptr_ = occ_grid_map_ptr;
#ifndef DECOMP_OLD
  map_processed_ = true;
#endif
#ifdef DECOMP_OLD
  nav_msgs::OccupancyGrid occ_grid_map = *occ_grid_map_ptr_;

  if (!GetOccupiedGridCells(occ_grid_map)) {
    MPC_ERROR("StaticPolyhedronConstraints: Map processing failed!");
  }
#endif
}

void StaticPolyhedronConstraints::getLocalOccupiedCells(
    nav_msgs::OccupancyGrid::ConstPtr occ_grid_map_ptr, vec_Vec2f path) {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::getLocalOccupiedCells");

  // Convert occupancy grid manually to OpenCV image directly
  cv::Mat image =
      cv::Mat(occ_grid_map_ptr->info.height, occ_grid_map_ptr->info.width,
              CV_8UC1, const_cast<schar *>(&occ_grid_map_ptr->data[0]),
              (int)occ_grid_map_ptr->info.width);

  // determine scalar value for realworld to image position conversion
  int world_to_image_scalar = 1 / occ_grid_map_ptr->info.resolution;
  double image_to_world_scalar = 1.0 / world_to_image_scalar;

  int img_radius_around_bbox = radius_around_bbox_ * world_to_image_scalar;

  for (size_t k = 0; k < path.size() - 1; k++) {
    PROFILE_FUNCTION();
    // Get min and max values for path section
    double min_x = std::min(path[k](0), path[k + 1](0));
    double max_x = std::max(path[k](0), path[k + 1](0));
    double min_y = std::min(path[k](1), path[k + 1](1));
    double max_y = std::max(path[k](1), path[k + 1](1));

    // convert to gridmap data info. Starting point in data and size for row and
    // column
    int cell_start_x = (min_x - occ_grid_map_ptr->info.origin.position.x) *
                           world_to_image_scalar -
                       img_radius_around_bbox;
    int cell_start_y = (min_y - occ_grid_map_ptr->info.origin.position.y) *
                           world_to_image_scalar -
                       img_radius_around_bbox;
    int n_cols =
        (max_x - min_x) * world_to_image_scalar + 2 * img_radius_around_bbox;
    int n_rows =
        (max_y - min_y) * world_to_image_scalar + 2 * img_radius_around_bbox;

    // Check if the ROI is within the image
    if (cell_start_x < 0) {
      cell_start_x = 0;
    }
    if (cell_start_y < 0) {
      cell_start_y = 0;
    }
    if (cell_start_x + n_cols > image.cols) {
      n_cols = image.cols - cell_start_x;
    }
    if (cell_start_y + n_rows > image.rows) {
      n_rows = image.rows - cell_start_y;
    }

    // double roiImage_origin_x = min_x - radius_around_bbox_;
    // double roiImage_origin_y = min_y - radius_around_bbox_;
    double roiImage_origin_x = cell_start_x * image_to_world_scalar +
                               occ_grid_map_ptr->info.origin.position.x;
    double roiImage_origin_y = cell_start_y * image_to_world_scalar +
                               occ_grid_map_ptr->info.origin.position.y;

    // Extract the ROI using the min and max values
    cv::Rect roi(cell_start_x, cell_start_y, n_cols, n_rows);
    cv::Mat roiImage = image(roi);

    // Get the contours in the ROI image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiImage, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);

    int size = 0;
    for (size_t i = 0; i < contours.size(); i++) {
      size += contours[i].size();
    }

    occ_pos_vec_stages_[k] = std::make_unique<vec_Vec2f>();
    // Eigen::MatrixXd occ_pos_matrix = Eigen::MatrixXd::Zero(2, size);

    // Reserve initial size of memory for the vector of points
    occ_pos_vec_stages_[k]->reserve(size);
    {
      PROFILE_SCOPE("loop through contours");
      // Loop through each contour
      for (size_t i = 0; i < contours.size(); i++) {
        // Loop through each point in the contour
        for (size_t j = 0; j < contours[i].size(); j++) {
          // Add the contour point to the vector of points close to the path
          // point
          double x =
              roiImage_origin_x + contours[i][j].x * image_to_world_scalar;
          double y =
              roiImage_origin_y + contours[i][j].y * image_to_world_scalar;

          occ_pos_vec_stages_[k]->emplace_back(
              Eigen::Matrix<double, 2, 1>(x, y));
        }
      }
    }

    if (k == 0) {
      // Publish the local gridmap
      nav_msgs::OccupancyGrid gridmap;
      gridmap.header.stamp = ros::Time::now();
      gridmap.header.frame_id = ptr_config_->robot_target_frame_;
      gridmap.info.resolution = occ_grid_map_ptr->info.resolution;
      gridmap.info.width = roiImage.cols;
      gridmap.info.height = roiImage.rows;
      gridmap.data.resize(gridmap.info.width * gridmap.info.height);

      gridmap.info.origin.position.x = roiImage_origin_x;
      gridmap.info.origin.position.y = roiImage_origin_y;
      for (int y = 0; y < roiImage.rows; ++y) {
        for (int x = 0; x < roiImage.cols; ++x) {
          // Assuming the image is a CV_8UC1 grayscale image
          int i = x + y * roiImage.cols;
          uint8_t pixel_value = roiImage.at<uint8_t>(y, x);
          // Scale the pixel values to [0, 100] for OccupancyGrid
          gridmap.data[i] = pixel_value;
        }
      }
      local_occ_grid_pub_.publish(gridmap);
    }
  }
}

/* Static polyhedron constraints generation */
// Main
bool StaticPolyhedronConstraints::ComputeConstraints() {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::ComputeConstraints");

  bool success = true;

#ifndef DECOMP_OLD

  // Get the occupied cells in the local region of the path points
  getLocalOccupiedCells(occ_grid_map_ptr_, path_);

  {
    PROFILE_SCOPE("decomp dilate new");

    // Depending on the constraints configuration, dilate the path
    // (circular/ellipsoidal decomposition)
    switch (ptr_config_->line_segment_to_stage_) {
      case -1:
        decomp_util_.dilate(path_, occ_pos_vec_stages_, 0, false);
        break;

      case 0:
        decomp_util_.dilate(path_, occ_pos_vec_stages_, 0, true);
        break;

      case 1:
        decomp_util_.dilate(path_, occ_pos_vec_stages_, 0, false);
        break;

      default:
        break;
    }
  }

#endif
#ifdef DECOMP_OLD
  {
    PROFILE_SCOPE("decomp set_obs old");
    // Always set the currently known occupied positions
    decomp_util_.set_obs(occ_pos_);
  }

  {
    PROFILE_SCOPE("decomp dilate old");
    // Depending on the constraints configuration, dilate the path
    // (circular/ellipsoidal decomposition)
    switch (ptr_config_->line_segment_to_stage_) {
      case -1:
        decomp_util_.dilate(path_, 0, false);
        break;

      case 0:
        decomp_util_.dilate(path_, 0, true);
        break;

      case 1:
        decomp_util_.dilate(path_, 0, false);
        break;

      default:
        break;
    }
  }
#endif

  {
    PROFILE_SCOPE("decomp set_constraints");
    // Obtain constraints
    decomp_util_.set_constraints(poly_constraints_,
                                 ptr_config_->robot_center_of_mass_to_back_ +
                                     ptr_config_->safety_margin_);

    for (int k = 1; k < n_stages_to_construct_ + 1; k++) {
      n_relevant_constraints_(k) = (int)poly_constraints_[k - 1].A_.rows();
      if (n_relevant_constraints_(k) > n_constraints_per_stage_) {
        MPC_ERROR(
            "StaticPolyhedronConstraints: Amount of relevant constraints ("
            << n_relevant_constraints_(k) << ") exceeds maximum limit of "
            << n_constraints_per_stage_ << ". Using first "
            << n_constraints_per_stage_ << "constraints");
        n_relevant_constraints_(k) = n_constraints_per_stage_;
      }
    }
  }

  // Determine whether polyhedron creations were successful
  if (poly_constraints_.size() > 0) {
    for (int i = 0; i < (int)poly_constraints_.size(); i++) {
      success = success &&
                (int)poly_constraints_[i].A_.rows() >=
                    4;  // constraints should always consist of the bounding box
    }
  } else {
    success = false;
  }

  return success;
}

// State and map processing
bool StaticPolyhedronConstraints::ComputeLineSegments() {
  PROFILE_AND_LOG_INFO("StaticPolyhedronConstraints::ComputeLineSegments");

  double x, y, yaw, x_old, y_old, error;
  for (int k = 0; k < ptr_solver_->n_ + 1; k++) {
    x = ptr_solver_->stage_vars_(x_position_, k);
    y = ptr_solver_->stage_vars_(y_position_, k);
    yaw = ptr_solver_->stage_vars_(psi_position_, k);

    error = 0;

    if (k > 0) {
      x_old = ptr_solver_->stage_vars_(x_position_, k - 1);
      y_old = ptr_solver_->stage_vars_(y_position_, k - 1);
      if ((x - x_old == 0) && (y - y_old == 0)) {
        error = double(rand()) / double((RAND_MAX)) * 10e-4;
      };
    }

    pred_traj_.poses[k].pose.position.x = x + error * cos(yaw);
    pred_traj_.poses[k].pose.position.y = y + error * sin(yaw);
    pred_traj_.poses[k].pose.orientation.z =
        yaw;  // abusing quaternion z element for yaw rotation about z-axis
  }

  // Construct path elements based on predicted positions of previous run
  // Note: see constructor StaticPolyhedronConstraints() for more details
  // about the convention
  path_.clear();
  Eigen::Matrix<double, 2, 1> point;
  switch (ptr_config_->line_segment_to_stage_) {
    case -1:
      for (int k = 2; k < ptr_solver_->n_ + 1; k++) {
        point << pred_traj_.poses[k].pose.position.x,
            pred_traj_.poses[k].pose.position.y;
        path_.push_back(point);
      }
      point
          << pred_traj_.poses[ptr_solver_->n_].pose.position.x +
                 ptr_config_->delta_segment_ *
                     cos(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z),
          pred_traj_.poses[ptr_solver_->n_].pose.position.y +
              ptr_config_->delta_segment_ *
                  sin(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z);
      path_.push_back(point);
      point
          << pred_traj_.poses[ptr_solver_->n_].pose.position.x +
                 2 * ptr_config_->delta_segment_ *
                     cos(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z),
          pred_traj_.poses[ptr_solver_->n_].pose.position.y +
              2 * ptr_config_->delta_segment_ *
                  sin(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z);
      path_.push_back(point);
      break;

    case 0:
      for (int k = 2; k < ptr_solver_->n_ + 1; k++) {
        point << pred_traj_.poses[k].pose.position.x,
            pred_traj_.poses[k].pose.position.y;
        path_.push_back(point);
        point << pred_traj_.poses[k].pose.position.x +
                     ptr_config_->delta_segment_ *
                         cos(pred_traj_.poses[k].pose.orientation.z),
            pred_traj_.poses[k].pose.position.y +
                ptr_config_->delta_segment_ *
                    sin(pred_traj_.poses[k].pose.orientation.z);
        path_.push_back(point);
      }
      point
          << pred_traj_.poses[ptr_solver_->n_].pose.position.x +
                 ptr_config_->delta_segment_ *
                     cos(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z),
          pred_traj_.poses[ptr_solver_->n_].pose.position.y +
              ptr_config_->delta_segment_ *
                  sin(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z);
      path_.push_back(point);
      break;

    case 1:
      for (int k = 1; k < ptr_solver_->n_ + 1; k++) {
        point << pred_traj_.poses[k].pose.position.x,
            pred_traj_.poses[k].pose.position.y;
        path_.push_back(point);
      }
      point
          << pred_traj_.poses[ptr_solver_->n_].pose.position.x +
                 ptr_config_->delta_segment_ *
                     cos(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z),
          pred_traj_.poses[ptr_solver_->n_].pose.position.y +
              ptr_config_->delta_segment_ *
                  sin(pred_traj_.poses[ptr_solver_->n_].pose.orientation.z);
      path_.push_back(point);
      break;

    default:
      MPC_ERROR(
          "StaticPolyhedronConstraints: Unknown value for "
          "'ptr_config_->line_segment_to_stage_'!");
      return false;
      break;
  }

  line_segments_updated_ = true;

  return true;
}

// Helper methods
void StaticPolyhedronConstraints::ComputeVertices(
    const Eigen::VectorXd &constraints) {
  PROFILE_AND_LOG_INFO("ComputeVertices");

  size_t n_constraints = constraints.size() / 3;

  // For all constraint pairs
  double det;
  bool satisfies_all_constraints;
  for (size_t i = 0; i < n_constraints; i++) {
    for (size_t j = i + 1; j < n_constraints; j++) {
      // Compute the intersection point
      det = constraints(3 * i) * constraints(3 * j + 1) -
            constraints(3 * i + 1) * constraints(3 * j);
      if (std::abs(det) < 1e-9) continue;  // Ignore parallel lines
      Eigen::Vector2d x;
      x << (constraints(3 * j + 1) * constraints(3 * i + 2) -
            constraints(3 * i + 1) * constraints(3 * j + 2)) /
               det,
          (constraints(3 * i) * constraints(3 * j + 2) -
           constraints(3 * j) * constraints(3 * i + 2)) /
              det;

      // Check if the intersection point satisfies all constraints
      satisfies_all_constraints = true;
      for (size_t k = 0; k < n_constraints; k++) {
        if (k == i || k == j)
          continue;  // Skip the constraints used to compute the intersection
                     // point
        if (constraints(3 * k) * x(0) + constraints(3 * k + 1) * x(1) >
            constraints(3 * k + 2) + FLOAT_TOL) {
          satisfies_all_constraints = false;
          break;
        }
      }

      // If the intersection point satisfies all constraints, add it to the
      // list of vertices
      if (satisfies_all_constraints) vertices_to_vis_.push_back(x);
    }
  }

  // Compute the centroid of the vertices
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto &vertex : vertices_to_vis_) {
    centroid += vertex;
  }
  centroid /= static_cast<double>(vertices_to_vis_.size());

  // Sort the vertices based on their angle with respect to the centroid
  std::sort(vertices_to_vis_.begin(), vertices_to_vis_.end(),
            [&centroid](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
              return std::atan2(a(1) - centroid(1), a(0) - centroid(0)) <
                     std::atan2(b(1) - centroid(1), b(0) - centroid(0));
            });
}

/**
 * @brief OLD CODE USED FOR DECOMP_OLD defnitions
 *
 */

bool StaticPolyhedronConstraints::GetOccupiedGridCells(
    const nav_msgs::OccupancyGrid &occ_grid_map) {
  PROFILE_FUNCTION();
  MPC_INFO("StaticPolyhedronConstraints::GetOccupiedGridCells");

  /** Two options: calculate halfspace constraints based on:
   * 1. Each occupied cell in the occupancy grid
   * 2. The obstacle contours (determined using OpenCV)
   */
  int n_points = 0;
  /************************ Option 1 ************************/
  if (ptr_config_->occ_cell_selection_method_ == 0) {
    /* Create members to construct polyhedron constraints */
    // Determine amount of occupied cells in the occupancy grid map
    // Create vector to store indices of occupied grid cells
    std::vector<int> ids;
    for (int i = 0; i < (int)occ_grid_map.data.size(); i++) {
      if (occ_grid_map.data[i] >= occupied_threshold_) {
        ids.push_back(i);
      }
    }
    n_points = ids.size();
    MPC_INFO_STREAM(
        "StaticPolyhedronConstraints: Amount of occupied cells in grid map: "
        << n_points);

    // Store all occupied cells in the grid map
    occ_pos_.resize(n_points);
    float x = 0, y = 0;
    int idx = 0;
    for (int i = 0; i < (int)contours_.size(); i++) {
      for (int j = 0; j < (int)contours_[i].size(); j++) {
        x = occ_grid_map.info.origin.position.x +
            contours_[i][j].x * occ_grid_map.info.resolution;
        y = occ_grid_map.info.origin.position.y +
            contours_[i][j].y * occ_grid_map.info.resolution;
        occ_pos_[idx] = Vec2f(x, y);
        idx++;
      }
    }

    // Free memory of indices vector
    std::vector<int>().swap(ids);
  }
  /************************ Option 2 ************************/
  else if (ptr_config_->occ_cell_selection_method_ == 1) {
    // Obtain obstacle contours
    if (!GetObstacleContours(occ_grid_map, contours_)) {
      MPC_ERROR(
          "StaticPolyhedronConstraints: Determining obstacle contours via "
          "grid "
          "map and OpenCV failed!");
      exit(1);
    }

    // Determine amount of obstacle contour points
    for (int i = 0; i < (int)contours_.size(); i++) {
      n_points += contours_[i].size();
    }
    MPC_INFO_STREAM(
        "StaticPolyhedronConstraints: Amount of occupied obstacle contour "
        "cells in grid map: "
        << n_points);

    // Store all occupied cells in the grid map
    occ_pos_.resize(n_points);
    float x = 0, y = 0;
    int idx = 0;
    for (int i = 0; i < (int)contours_.size(); i++) {
      for (int j = 0; j < (int)contours_[i].size(); j++) {
        x = occ_grid_map.info.origin.position.x +
            contours_[i][j].x * occ_grid_map.info.resolution;
        y = occ_grid_map.info.origin.position.y +
            contours_[i][j].y * occ_grid_map.info.resolution;
        occ_pos_[idx] = Vec2f(x, y);
        idx++;
      }
    }
  } else {
    MPC_ERROR(
        "StaticPolyhedronConstraints: Unknown value for "
        "'ptr_config_->occ_cell_selection_method_'!");
    return false;
  }

  map_processed_ = true;

  return true;
}

// Helpers
bool StaticPolyhedronConstraints::GetObstacleContours(
    const nav_msgs::OccupancyGrid &occ_grid_map,
    std::vector<std::vector<cv::Point>> &contours) {
  PROFILE_FUNCTION();
  MPC_INFO("StaticPolyhedronConstraints::GetObstacleContours");

  cv::Mat image;

  // Convert occupancy grid manually to OpenCV image directly
  {
    PROFILE_SCOPE("Conversion from occupancy grid to OpenCV image");
    image = cv::Mat(occ_grid_map.info.height, occ_grid_map.info.width, CV_8UC1,
                    const_cast<schar *>(&occ_grid_map.data[0]),
                    (int)occ_grid_map.info.width);
  }

  // Determine contour points in OpenCV image
  {
    PROFILE_SCOPE("Determine contours in OpenCV image");
    cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  }

  // /** For debugging only!
  // cv::Mat contourImage(image.size(), CV_8UC1, cv::Scalar(0,0,0));
  // cv::Scalar white, colors[3];
  // white = cv::Scalar(255, 0, 0);
  // colors[0] = cv::Scalar(255, 0, 0);
  // colors[1] = cv::Scalar(0, 255, 0);
  // colors[2] = cv::Scalar(0, 0, 255);
  // for (int idx = 0; idx < contours.size(); idx++) {
  //     cv::drawContours(contourImage, contours, idx, white);
  // }

  // cv::imshow("Input Image", image);
  // cv::moveWindow("Input Image", 0, 0);
  // cv::imshow("Contours", contourImage);
  // cv::moveWindow("Contours", 200, 0);
  // cv::waitKey(0);
  // */

  return true;
}
