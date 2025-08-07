#include <mpc_core/functions/update_mpc_configuration.h>

#include <cmath>

bool mpc_configuration_initialize(const ros::NodeHandle& nh,
                                  ConfigurationMPC& config) {
  ROS_WARN_STREAM("[MPC config] Initializing parameter configuration");

  /************************** PARAMS **************************/
  /* Hierarchical settings */
  if (!nh.param("/sim", config.sim_, false)) {
    defaultFunc("/sim");
  };

  /* Hierarchical settings (necessary to define here) */
  if (!nh.param("/n_layers", config.n_layers_, 1)) {
    defaultFunc("/n_layers");
  };
  if (!nh.param("layer_idx", config.layer_idx_, 0)) {
    defaultFunc("layer_idx");
  };
  if (config.n_layers_ > 1) {
    ROS_WARN_STREAM(
        "[MPC config] Total amount of MPC layers: " << config.n_layers_);
    ROS_WARN_STREAM("[MPC config] MPC layer: " << config.layer_idx_);
  }
  std::string layer_idx_str;
  if (config.n_layers_ > 1) {
    layer_idx_str = "/" + std::to_string(config.layer_idx_);
  } else {
    layer_idx_str = "";
  }

  /* Control loop and actuation */
  if (!nh.getParam("/params/control_loop_and_actuation/enable_output",
                   config.enable_output_)) {
    return errorFunc("/params/control_loop_and_actuation/enable_output");
  };
  if (!nh.getParam("/params/control_loop_and_actuation/auto_enable_plan",
                   config.auto_enable_plan_)) {
    return errorFunc("/params/control_loop_and_actuation/auto_enable_plan");
  };
  if (config.auto_enable_plan_) {
    config.plan_ = true;
  } else {
    config.plan_ = false;
  };
  if (!nh.getParam("/params/control_loop_and_actuation/synchronized_actuation",
                   config.synchronized_actuation_)) {
    return errorFunc(
        "/params/control_loop_and_actuation/synchronized_actuation");
  };
  if (!nh.param("/params/control_loop_and_actuation/feedback_law_frequency",
                config.feedback_law_frequency_, 100.0)) {
    defaultFunc("/params/control_loop_and_actuation/sync_mode");
  };
  if (!nh.param("/params/control_loop_and_actuation/sync_mode",
                config.sync_mode_, true)) {
    defaultFunc("/params/control_loop_and_actuation/sync_mode");
  };
  if (!nh.param("/params/control_loop_and_actuation/sync_simple_sim",
                config.sync_simple_sim_, false)) {
    defaultFunc("/params/control_loop_and_actuation/sync_simple_sim");
  };

  /* Solver */
  if (!nh.param("/params/solver/use_custom_initial_guess",
                config.use_custom_initial_guess_, false)) {
    defaultFunc("/params/solver/use_custom_initial_guess");
  };
  if (!nh.param("/params/solver/time_shift", config.time_shift_, 1)) {
    defaultFunc("/params/solver/time_shift");
  };

  /* Robot description */
  if (!nh.param("/params/robot/n_dim", config.n_dim_, 2)) {
    defaultFunc("/params/robot/n_dim");
  };
  if (!nh.getParam("/params/robot/length", config.robot_length_)) {
    return errorFunc("/params/robot/length");
  };
  if (!nh.getParam("/params/robot/width", config.robot_width_)) {
    return errorFunc("/params/robot/width");
  };
  if (!nh.getParam("/params/robot/com_to_back",
                   config.robot_center_of_mass_to_back_)) {
    return errorFunc("/params/robot/com_to_back");
  };
  if (!nh.getParam("/params/robot/base_link", config.robot_base_link_)) {
    return errorFunc("/params/robot/base_link");
  };
  if (!nh.getParam("/params/robot/target_frame", config.robot_target_frame_)) {
    return errorFunc("/params/robot/target_frame");
  };

  /* Modules */
  // Goal oriented
  if (!nh.param("/params/modules/goal_oriented/goal_reached_threshold",
                config.goal_reached_threshold_, 0.02)) {
    defaultFunc("/params/modules/goal_oriented/goal_reached_threshold");
  };
  if (!nh.param("/params/modules/goal_oriented/auto_send_goals",
                config.auto_send_goals_, false)) {
    defaultFunc("/params/modules/goal_oriented/auto_send_goals");
  };
  if (!nh.param("/params/modules/goal_oriented/goals_trajectory",
                config.goals_trajectory_, std::string("none"))) {
    defaultFunc("/params/modules/goal_oriented/goals_trajectory");
  };
  if (!nh.param("/params/modules/goal_oriented/goals_x", config.goals_x_,
                {0})) {
    defaultFunc("/params/modules/goal_oriented/goals_x");
  };
  if (!nh.param("/params/modules/goal_oriented/goals_y", config.goals_y_,
                {0})) {
    defaultFunc("/params/modules/goal_oriented/goals_y");
  };
  if (!nh.param("/params/modules/goal_oriented/goals_z", config.goals_z_,
                {0})) {
    defaultFunc("/params/modules/goal_oriented/goals_z");
  };
  if (!nh.param("/params/modules/goal_oriented/goals_yaw", config.goals_yaw_,
                {0})) {
    defaultFunc("/params/modules/goal_oriented/goals_yaw");
  };
  // Static polyhedron constraints
  if (!nh.param("/params/modules/static_polyhedron_constraints/"
                "n_constraints_per_stage",
                config.n_constraints_per_stage_, 24)) {
    defaultFunc(
        "/params/modules/static_polyhedron_constraints/"
        "n_constraints_per_stage");
  };
  if (!nh.param(
          "/params/modules/static_polyhedron_constraints/occupied_threshold",
          config.occupied_threshold_, 70)) {
    defaultFunc(
        "/params/modules/static_polyhedron_constraints/occupied_threshold");
  };
  if (!nh.param("/params/modules/static_polyhedron_constraints/"
                "occ_cell_selection_method",
                config.occ_cell_selection_method_, 1)) {
    defaultFunc(
        "/params/modules/static_polyhedron_constraints/"
        "occ_cell_selection_method");
  };
  if (!nh.param(
          "/params/modules/static_polyhedron_constraints/bounding_box_width",
          config.bounding_box_width_, 2.0)) {
    defaultFunc(
        "/params/modules/static_polyhedron_constraints/bounding_box_width");
  };
  if (!nh.param(
          "/params/modules/static_polyhedron_constraints/line_segment_to_stage",
          config.line_segment_to_stage_, -1)) {
    defaultFunc(
        "/params/modules/static_polyhedron_constraints/line_segment_to_stage");
  };
  if (!nh.param("/params/modules/static_polyhedron_constraints/delta_segment",
                config.delta_segment_, 0.00001)) {
    defaultFunc("/params/modules/static_polyhedron_constraints/delta_segment");
  };
  if (!nh.param("/params/modules/static_polyhedron_constraints/safety_margin",
                config.safety_margin_, -1.0)) {
  };
  if (!nh.param("/params/modules/static_polyhedron_constraints/safety_margins",
                config.safety_margins_, {-1.0})) {
  };
  if ((int)config.safety_margins_.size() != config.n_layers_) {
    ROS_ERROR_STREAM(
        "[MPC config] The provided vector of safety margins has length "
        << config.safety_margins_.size()
        << ", whereas the amount of hierarchical MPC layers equals "
        << config.n_layers_ << "! Exiting.");
    return false;
  }
  if (config.safety_margin_ == -1) {
    config.safety_margin_ = config.safety_margins_[config.layer_idx_];
  };
  if (config.safety_margin_ == -1) {
    defaultFunc(
        "/params/modules/static_polyhedron_constraints/safety_margin_ or "
        "safety_margins_");
  }

  /* Random */
  if (!nh.param("/params/random/seed", config.random_seed_, 0)) {
    defaultFunc("/params/random/seed");
  };

  /* Printing */
  if (!nh.param("/params/printing/debug_output", config.debug_output_, false)) {
    defaultFunc("/params/printing/debug_output");
  };
  if (!nh.param("/params/printing/debug_data", config.debug_data_, false)) {
    defaultFunc("/params/printing/debug_data");
  };

  /* System interface */
  if (!nh.param("/params/system_interface/apply_feedback",
                config.apply_feedback_, false)) {
    defaultFunc("/params/system_interface/apply_feedback");
  };
  if (!nh.param("/params/system_interface/use_observer", config.use_observer_,
                false)) {
    defaultFunc("/params/system_interface/use_observer");
  };
  if (!nh.param("/params/system_interface/use_nominal_reference",
                config.use_nominal_reference_, false)) {
    defaultFunc("/params/system_interface/use_nominal_reference");
  };

  /* Visualization */
  // General
  if (!nh.param("/params/visualization/general/ground_projection_marker_alpha",
                config.ground_projection_marker_alpha_, 0.1)) {
    defaultFunc("/params/visualization/general/ground_projection_marker_alpha");
  };
  if (!nh.param("/params/visualization/general/ground_projection_marker_z",
                config.ground_projection_marker_z_, 0.1)) {
    defaultFunc("/params/visualization/general/ground_projection_marker_z");
  };
  // System interface
  if (!nh.param("/params/visualization/system_interface/"
                "draw_current_position_ground_air",
                config.draw_current_position_ground_air_, 3)) {
    defaultFunc(
        "/params/visualization/system_interface/"
        "draw_current_position_ground_air");
  };
  if (!nh.param("/params/visualization/system_interface/"
                "current_position_marker_color",
                config.current_position_marker_color_, {0.5, 0, 0.5, 1})) {
    defaultFunc(
        "/params/visualization/system_interface/current_position_marker_color");
  };
  if (!nh.param("/params/visualization/system_interface/"
                "current_position_marker_scale",
                config.current_position_marker_scale_, {0.05, 0.05, 0.2})) {
    defaultFunc(
        "/params/visualization/system_interface/current_position_marker_scale");
  };
  if (!nh.param("/params/visualization/system_interface/region_min_n_points",
                config.region_min_n_points_, 30)) {
    defaultFunc("/params/visualization/system_interface/region_min_n_points");
  };
  if (!nh.param(
          "/params/visualization/system_interface/current_region_n_points",
          config.current_region_n_points_, 1)) {
    defaultFunc(
        "/params/visualization/system_interface/current_region_n_points");
  };
  if (config.draw_current_region_ && config.current_region_n_points_ > 1 &&
      config.current_region_n_points_ < config.region_min_n_points_) {
    ROS_WARN_STREAM(
        "[MPC config] The allowed values for current_region_n_points_ are {1,>"
        << config.region_min_n_points_
        << "}. Current region will not be created!");
  };
  if (!nh.param(
          "/params/visualization/system_interface/predicted_regions_n_points",
          config.predicted_regions_n_points_, 1)) {
    defaultFunc(
        "/params/visualization/system_interface/predicted_regions_n_points");
  };
  if (config.draw_predicted_regions_ &&
      config.predicted_regions_n_points_ > 1 &&
      config.predicted_regions_n_points_ < config.region_min_n_points_) {
    ROS_WARN_STREAM(
        "[MPC config] The allowed values for predicted_regions_n_points_ are "
        "{1,>"
        << config.region_min_n_points_
        << "}. Current region will not be created!");
  };
  if (!nh.param("/params/visualization/system_interface/draw_current_region",
                config.draw_current_region_, true)) {
    defaultFunc("/params/visualization/system_interface/draw_current_region");
  };
  if (!nh.param(
          "/params/visualization/system_interface/current_region_marker_scale",
          config.current_region_marker_scale_, {0.05, 0.05, 0.2})) {
    defaultFunc(
        "/params/visualization/system_interface/current_region_marker_scale");
  };
  if (!nh.param(
          "/params/visualization/system_interface/current_region_marker_z",
          config.current_region_marker_z_, 0.001)) {
    defaultFunc(
        "/params/visualization/system_interface/current_region_marker_z");
  };
  if (!nh.param("/params/visualization/system_interface/"
                "draw_predicted_positions_ground_air" +
                    layer_idx_str,
                config.draw_predicted_positions_ground_air_, 3)) {
    defaultFunc(
        "/params/visualization/system_interface/"
        "draw_predicted_positions_ground_air" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/system_interface/"
                "predicted_positions_marker_color" +
                    layer_idx_str,
                config.predicted_positions_marker_color_,
                {1, 0.6484375, 0, 0.8})) {
    defaultFunc(
        "/params/visualization/system_interface/"
        "predicted_positions_marker_color" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/system_interface/"
                "predicted_positions_marker_scale" +
                    layer_idx_str,
                config.predicted_positions_marker_scale_, {0.1, 0.1, 0.16})) {
    defaultFunc(
        "/params/visualization/system_interface/"
        "predicted_positions_marker_scale" +
        layer_idx_str);
  };
  if (!nh.param(
          "/params/visualization/system_interface/draw_predicted_regions" +
              layer_idx_str,
          config.draw_predicted_regions_, false)) {
    defaultFunc(
        "/params/visualization/system_interface/draw_predicted_regions" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/system_interface/"
                "predicted_regions_marker_scale" +
                    layer_idx_str,
                config.predicted_regions_marker_scale_, {0.1, 0.1, 0.16})) {
    defaultFunc(
        "/params/visualization/system_interface/"
        "predicted_regions_marker_scale" +
        layer_idx_str);
  };
  if (!nh.param(
          "/params/visualization/system_interface/predicted_regions_marker_z" +
              layer_idx_str,
          config.predicted_regions_marker_z_, 0.0005)) {
    defaultFunc(
        "/params/visualization/system_interface/predicted_regions_marker_z" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/system_interface/region_indices_to_draw",
                config.region_indices_to_draw_, {1})) {
    defaultFunc(
        "/params/visualization/system_interface/region_indices_to_draw");
  };
  // Modules
  if (!nh.param("/params/visualization/modules/goal_oriented/"
                "draw_goal_position_ground_air",
                config.draw_goal_position_ground_air_, 3)) {
    defaultFunc(
        "/params/visualization/modules/goal_oriented/"
        "draw_goal_position_ground_air");
  };
  if (!nh.param("/params/visualization/modules/goal_oriented/marker_color",
                config.goal_position_marker_color_, {0, 1, 0, 0.2})) {
    defaultFunc("/params/visualization/modules/goal_oriented/marker_color");
  };
  if (!nh.param("/params/visualization/modules/goal_oriented/marker_scale",
                config.goal_position_marker_scale_, {0.2, 0.2, 0.2})) {
    defaultFunc("/params/visualization/modules/goal_oriented/marker_scale");
  };
  if (!nh.param("/params/visualization/modules/reference_trajectory/"
                "draw_reference_trajectory_ground_air",
                config.draw_reference_trajectory_ground_air_, 3)) {
    defaultFunc(
        "/params/visualization/modules/reference_trajectory/"
        "draw_reference_trajectory_ground_air");
  };
  if (!nh.param(
          "/params/visualization/modules/reference_trajectory/marker_color",
          config.reference_trajectory_marker_color_, {1, 0, 0, 0.6})) {
    defaultFunc(
        "/params/visualization/modules/reference_trajectory/marker_color");
  };
  if (!nh.param(
          "/params/visualization/modules/reference_trajectory/marker_scale",
          config.reference_trajectory_marker_scale_, {0.12, 0.12, 0.12})) {
    defaultFunc(
        "/params/visualization/modules/reference_trajectory/marker_scale");
  };
  if (!nh.param("/params/visualization/modules/static_polyhedron_constraints/"
                "draw_constraints" +
                    layer_idx_str,
                config.draw_constraints_, true)) {
    defaultFunc(
        "/params/visualization/modules/static_polyhedron_constraints/"
        "draw_constraints" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/modules/static_polyhedron_constraints/"
                "marker_alpha_hierarchical",
                config.constraints_marker_alpha_hierarchical_, 0.1)) {
    defaultFunc(
        "/params/visualization/modules/static_polyhedron_constraints/"
        "marker_alpha_hierarchical");
  };
  if (!nh.param("/params/visualization/modules/static_polyhedron_constraints/"
                "marker_scale",
                config.constraints_marker_scale_, {0.1, 0.1, 0.001})) {
    defaultFunc(
        "/params/visualization/modules/static_polyhedron_constraints/"
        "marker_scale");
  };
  if (!nh.param("/params/visualization/modules/static_polyhedron_constraints/"
                "marker_z" +
                    layer_idx_str,
                config.constraints_marker_z_, 0.004)) {
    defaultFunc(
        "/params/visualization/modules/static_polyhedron_constraints/marker_z" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/modules/static_polyhedron_constraints/"
                "indices_to_draw" +
                    layer_idx_str,
                config.indices_to_draw_, {0, 1, 2, 3})) {
    defaultFunc(
        "/params/visualization/modules/static_polyhedron_constraints/"
        "indices_to_draw" +
        layer_idx_str);
  };
  if (!nh.param("/params/visualization/modules/static_polyhedron_constraints/"
                "draw_vertices",
                config.draw_vertices_, true)) {
    defaultFunc(
        "/params/visualization/modules/static_polyhedron_constraints/"
        "draw_vertices");
  };

  /* Recording */
  // General
  if (!nh.param("/params/recording/enable_ros_recordings",
                config.enable_ros_recordings_, false)) {
    defaultFunc("/params/recording/enable_ros_recordings");
  };
  if (!nh.param("/params/recording/enable_json_recordings",
                config.record_experiment_json_, false)) {
    defaultFunc("/params/recording/enable_json_recordings");
  };
  if (!nh.param("/params/recording/experiment_name_json",
                config.recording_name_json_, std::string("no_name"))) {
    defaultFunc("/params/recording/experiment_name_json");
  };
  // System interface
  if (!nh.param("/params/recording/system_interface/record_current_state",
                config.record_cur_state_, false)) {
    defaultFunc("/params/recording/system_interface/record_current_state");
  };
  if (!nh.param("/params/recording/system_interface/record_interp_data",
                config.record_interp_data_, false)) {
    defaultFunc("/params/recording/system_interface/record_interp_data");
  };
  if (!nh.param("/params/recording/system_interface/record_nominal_reference",
                config.record_nominal_reference_, false)) {
    defaultFunc("/params/recording/system_interface/record_nominal_reference");
  };
  if (!config.use_nominal_reference_) {
    config.record_nominal_reference_ = false;
    ROS_WARN_STREAM(
        "[MPC config] The nominal reference is not used, so it will not be "
        "recorded!");
  }
  if (!nh.param("/params/recording/system_interface/"
                "record_predicted_positions" +
                    layer_idx_str,
                config.record_pred_traj_, false)) {
    defaultFunc(
        "/params/recording/system_interface/record_predicted_positions" +
        layer_idx_str);
  };
  if (!nh.param(
          "/params/recording/system_interface/record_slack" + layer_idx_str,
          config.record_slack_, false)) {
    defaultFunc("/params/recording/system_interface/record_slack" +
                layer_idx_str);
  };
  if (!nh.param("/params/recording/system_interface/record_time_reference" +
                    layer_idx_str,
                config.record_time_reference_, false)) {
    defaultFunc("/params/recording/system_interface/record_time_reference" +
                layer_idx_str);
  };
  // Modules
  if (!nh.param("/params/recording/modules/goal_oriented/record_goal_position" +
                    layer_idx_str,
                config.record_goal_position_, false)) {
    defaultFunc("/params/recording/modules/goal_oriented/record_goal_position" +
                layer_idx_str);
  };
  if (!nh.param("/params/recording/modules/reference_trajectory/"
                "record_reference_trajectory" +
                    layer_idx_str,
                config.record_reference_trajectory_, false)) {
    defaultFunc(
        "/params/recording/modules/reference_trajectory/"
        "record_reference_trajectory" +
        layer_idx_str);
  };
  if (!nh.param("/params/recording/modules/static_polyhedron_constraints/"
                "record_constraints" +
                    layer_idx_str,
                config.record_constraints_, false)) {
    defaultFunc(
        "/params/recording/modules/static_polyhedron_constraints/"
        "record_constraints" +
        layer_idx_str);
  };
  // Give warning if ROS data recording is disabled, but one of the recording
  // settings is enabled
  if (!config.enable_ros_recordings_) {
    if (config.record_cur_state_) rosRecordingWarning("current state");
    if (config.record_interp_data_) rosRecordingWarning("interpolation data");
    if (config.record_pred_traj_) rosRecordingWarning("predicted positions");
    if (config.record_slack_) rosRecordingWarning("slack");
    if (config.record_goal_position_) rosRecordingWarning("goal position");
    if (config.record_reference_trajectory_)
      rosRecordingWarning("reference trajectory");
    if (config.record_constraints_) rosRecordingWarning("constraints");
  }
  /************************************************************/

  /************************** TOPICS **************************/
  /* Publish */
  // System interface
  if (!nh.getParam("/topics/publish/system_interface/control_command",
                   config.control_command_topic_)) {
    return errorFunc("/topics/publish/system_interface/control_command");
  };
  if (!nh.param("/topics/publish/system_interface/feedback_law",
                config.feedback_law_topic_, std::string("/feedback_law"))) {
    defaultFunc("/topics/publish/system_interface/feedback_law");
  };
  // Modules
  if (!nh.param("/topics/publish/modules/static_polyhedron_constraints/"
                "local_occupancy_grid",
                config.local_occ_grid_topic_,
                std::string("/mpc/local_occupancy_grid"))) {
    defaultFunc(
        "/topics/publish/modules/static_polyhedron_constraints/"
        "local_occupancy_grid");
  };

  /* Subscribe */
  // System interface
  if (!nh.getParam("/topics/subscribe/system_interface/state",
                   config.state_topic_)) {
    return errorFunc("/topics/subscribe/system_interface/state");
  };
  // Modules
  if (!nh.param("/topics/subscribe/modules/goal_oriented/goal",
                config.goal_topic_, std::string("/goal"))) {
    defaultFunc("/topics/subscribe/modules/goal_oriented/goal");
  };
  if (!nh.param("/topics/subscribe/modules/static_polyhedron_constraints/"
                "occupancy_grid",
                config.occ_grid_topic_, std::string("/occupancy_grid"))) {
    defaultFunc(
        "/topics/subscribe/modules/static_polyhedron_constraints/"
        "occupancy_grid");
  };

  /* Visualization */
  // System interface
  if (!nh.getParam("/topics/visualization/system_interface/current_position",
                   config.current_position_vis_topic_)) {
    return errorFunc("/topics/visualization/system_interface/current_position");
  };
  config.current_position_ground_vis_topic_ =
      config.current_position_vis_topic_ + "/ground";
  if (!nh.getParam("/topics/visualization/system_interface/current_region",
                   config.current_region_vis_topic_)) {
    return errorFunc("/topics/visualization/system_interface/current_region");
  };
  if (!nh.getParam(
          "/topics/visualization/system_interface/predicted_positions" +
              layer_idx_str,
          config.predicted_positions_vis_topic_)) {
    return errorFunc(
        "/topics/visualization/system_interface/predicted_positions" +
        layer_idx_str);
  };
  if (config.predicted_positions_vis_topic_.size() > 0) {
    if (config.n_layers_ > 1)
      config.predicted_positions_ground_vis_topic_ =
          config.predicted_positions_vis_topic_.substr(
              0, config.predicted_positions_vis_topic_.size() - 2) +
          "/ground" + layer_idx_str;
    else
      config.predicted_positions_ground_vis_topic_ =
          config.predicted_positions_vis_topic_ + "/ground";
  } else {
    config.predicted_positions_ground_vis_topic_ = "";
  }
  if (!nh.getParam("/topics/visualization/system_interface/predicted_regions" +
                       layer_idx_str,
                   config.predicted_regions_vis_topic_)) {
    return errorFunc(
        "/topics/visualization/system_interface/predicted_regions" +
        layer_idx_str);
  };
  // Modules
  if (!nh.param("/topics/visualization/modules/goal_oriented/goal_position" +
                    layer_idx_str,
                config.goal_position_vis_topic_,
                std::string("/goal_position"))) {
    defaultFunc("/topics/visualization/modules/goal_oriented/goal_position" +
                layer_idx_str);
  };
  if (config.goal_position_vis_topic_.size() > 0) {
    if (config.n_layers_ > 1)
      config.goal_position_ground_vis_topic_ =
          config.goal_position_vis_topic_.substr(
              0, config.goal_position_vis_topic_.size() - 2) +
          "/ground" + layer_idx_str;
    else
      config.goal_position_ground_vis_topic_ =
          config.goal_position_vis_topic_ + "/ground";
  } else {
    config.goal_position_ground_vis_topic_ = "";
  }
  if (!nh.param("/topics/visualization/modules/reference_trajectory/"
                "reference_trajectory" +
                    layer_idx_str,
                config.reference_trajectory_vis_topic_,
                std::string("/reference_trajectory"))) {
    defaultFunc(
        "/topics/visualization/modules/reference_trajectory/"
        "reference_trajectory" +
        layer_idx_str);
  };
  if (config.reference_trajectory_vis_topic_.size() > 0) {
    if (config.n_layers_ > 1)
      config.reference_trajectory_ground_vis_topic_ =
          config.reference_trajectory_vis_topic_.substr(
              0, config.reference_trajectory_vis_topic_.size() - 2) +
          "/ground" + layer_idx_str;
    else
      config.reference_trajectory_ground_vis_topic_ =
          config.reference_trajectory_vis_topic_ + "/ground";
  } else {
    config.reference_trajectory_ground_vis_topic_ = "";
  }
  if (!nh.param("/topics/visualization/modules/static_polyhedron_constraints/"
                "constraints" +
                    layer_idx_str,
                config.constraints_vis_topic_, std::string("/constraints"))) {
    defaultFunc(
        "/topics/visualization/modules/static_polyhedron_constraints/"
        "constraints" +
        layer_idx_str);
  };

  /* Recording */
  // System interface
  if (!nh.param("/topics/recording/system_interface/current_state",
                config.cur_state_rec_topic_,
                std::string("/mpc/rec/current_state"))) {
    defaultFunc("/topics/recording/system_interface/current_state");
  };
  if (!nh.param("/topics/recording/system_interface/interp_data",
                config.interp_data_topic_,
                std::string("/mpc/rec/interp_data"))) {
    defaultFunc("/topics/recording/system_interface/interp_data");
  };
  if (!nh.param("/topics/recording/system_interface/nominal_reference",
                config.nominal_reference_topic_,
                std::string("/mpc/rec/nominal_reference"))) {
    defaultFunc("/topics/recording/system_interface/nominal_reference");
  };
  if (!nh.param("/topics/recording/system_interface/predicted_trajectory" +
                    layer_idx_str,
                config.pred_traj_rec_topic_,
                std::string("/mpc/rec/predicted_trajectory"))) {
    defaultFunc("/topics/recording/system_interface/predicted_trajectory" +
                layer_idx_str);
  };
  if (!nh.param("/topics/recording/system_interface/slack" + layer_idx_str,
                config.slack_rec_topic_, std::string("/mpc/rec/slack"))) {
    defaultFunc("/topics/recording/system_interface/slack" + layer_idx_str);
  };
  if (!nh.param(
          "/topics/recording/system_interface/time_reference" + layer_idx_str,
          config.time_reference_rec_topic_,
          std::string("/mpc/rec/time_reference"))) {
    defaultFunc("/topics/recording/system_interface/time_reference" +
                layer_idx_str);
  };
  // Modules
  if (!nh.param("/topics/recording/modules/goal_oriented/goal_position" +
                    layer_idx_str,
                config.goal_position_rec_topic_,
                std::string("/mpc/rec/goal_position"))) {
    defaultFunc("/topics/recording/modules/goal_oriented/goal_position" +
                layer_idx_str);
  };
  if (!nh.param("/topics/recording/modules/reference_trajectory/"
                "reference_trajectory" +
                    layer_idx_str,
                config.reference_trajectory_rec_topic_,
                std::string("/mpc/rec/reference_trajectory"))) {
    defaultFunc(
        "/topics/recording/modules/reference_trajectory/reference_trajectory" +
        layer_idx_str);
  };
  if (!nh.param("/topics/recording/modules/static_polyhedron_constraints/"
                "constraints" +
                    layer_idx_str,
                config.constraints_rec_topic_,
                std::string("/mpc/rec/reference_trajectory"))) {
    defaultFunc(
        "/topics/recording/modules/static_polyhedron_constraintss/constraints" +
        layer_idx_str);
  };

  /* Hierarchical communication */
  if (!nh.param("/topics/hierarchical/start_mpc_layer",
                config.start_mpc_layer_topic_,
                std::string("/hierarchical/start_mpc_layer"))) {
    defaultFunc("/topics/hierarchical/start_mpc_layer");
  };
  if (!nh.param("/topics/hierarchical/reference", config.ref_topic_,
                std::string("/hierarchical/reference"))) {
    defaultFunc("/topics/hierarchical/reference");
  };
  if (!nh.param("/topics/hierarchical/pmpc_objective_reached",
                config.pmpc_obj_reached_topic_,
                std::string("/hierarchical/pmpc_objective_reached"))) {
    defaultFunc("/topics/hierarchical/pmpc_objective_reached");
  };
  if (!nh.param("/topics/hierarchical/pmpc_failure", config.pmpc_failure_topic_,
                std::string("/hierarchical/pmpc_failure"))) {
    defaultFunc("/topics/hierarchical/pmpc_failure");
  };
  /************************************************************/

  ROS_WARN_STREAM("[MPC config] Parameter configuration initialized");

  return true;
}

bool errorFunc(const std::string name) {
  ROS_ERROR_STREAM("[MPC config] Parameter \"" + name + "\" not defined!");
  return false;
}

void defaultFunc(const std::string name) {
  ROS_WARN_STREAM("[MPC config] Parameter \"" + name +
                  "\" not defined, using default value (see "
                  "update_mpc_configuration.cpp)!");
}

void rosRecordingWarning(const std::string name) {
  ROS_WARN_STREAM("[MPC config] Recording \"" + name +
                  "\" is enabled, but recording ROS data is disabled, so \"" +
                  name + "\" will not be recorded!");
}
