#ifndef MPC_BASE_CONFIGURATION_SETTINGS
#define MPC_BASE_CONFIGURATION_SETTINGS

#include <string>
#include <vector>

struct ConfigurationMPC {
  /************************** PARAMS **************************/
  /* Experiment settings */
  bool sim_;

  /* Hierarchical settings */
  int n_layers_, layer_idx_;

  /* Control loop and actuation */
  bool enable_output_, auto_enable_plan_, plan_, synchronized_actuation_;
  double feedback_law_frequency_;
  bool sync_mode_, sync_simple_sim_;

  /* Solver */
  bool use_custom_initial_guess_;
  int time_shift_;

  /* Robot description */
  int n_dim_;
  double robot_length_, robot_width_, robot_center_of_mass_to_back_;
  std::string robot_base_link_, robot_target_frame_;

  /* Modules */
  // Goal oriented
  double goal_reached_threshold_;
  bool auto_send_goals_;
  std::string goals_trajectory_;
  std::vector<double> goals_x_, goals_y_, goals_z_, goals_yaw_;
  // Static polyhedron constraints
  int n_constraints_per_stage_, occupied_threshold_, occ_cell_selection_method_;
  double bounding_box_width_;
  int line_segment_to_stage_;
  double delta_segment_, safety_margin_;
  std::vector<double> safety_margins_;

  /* Random */
  int random_seed_;

  /* Printing */
  bool debug_output_, debug_data_;

  /* System interface */
  bool apply_feedback_, use_observer_, use_nominal_reference_;

  /* Visualization */
  // General
  double ground_projection_marker_alpha_, ground_projection_marker_z_;
  // System interface
  int draw_current_position_ground_air_;
  std::vector<double> current_position_marker_color_,
      current_position_marker_scale_;
  bool draw_current_region_;
  int region_min_n_points_, current_region_n_points_,
      predicted_regions_n_points_;
  std::vector<double> current_region_marker_scale_;
  double current_region_marker_z_;
  int draw_predicted_positions_ground_air_;
  std::vector<double> predicted_positions_marker_color_,
      predicted_positions_marker_scale_;
  bool draw_predicted_regions_;
  std::vector<double> predicted_regions_marker_scale_;
  double predicted_regions_marker_z_;
  std::vector<int> region_indices_to_draw_;
  // Modules
  int draw_goal_position_ground_air_;
  std::vector<double> goal_position_marker_color_, goal_position_marker_scale_;
  int draw_reference_trajectory_ground_air_;
  std::vector<double> reference_trajectory_marker_color_,
      reference_trajectory_marker_scale_;
  bool draw_constraints_;
  double constraints_marker_alpha_hierarchical_;
  std::vector<double> constraints_marker_scale_;
  double constraints_marker_z_;
  std::vector<int> indices_to_draw_;
  bool draw_vertices_;

  /* Recording */
  // General
  bool enable_ros_recordings_;
  bool record_experiment_json_;
  std::string recording_name_json_;
  // System interface
  bool record_cur_state_, record_interp_data_, record_nominal_reference_,
      record_pred_traj_, record_slack_, record_time_reference_;
  // Modules
  bool record_goal_position_;
  bool record_reference_trajectory_;
  bool record_constraints_;
  /************************************************************/

  /************************** TOPICS **************************/
  /* Publish */
  // System interface
  std::string control_command_topic_, feedback_law_topic_;
  // Modules
  std::string local_occ_grid_topic_;

  /* Subscribe */
  // System interface
  std::string state_topic_;
  // Modules
  std::string goal_topic_;
  std::string occ_grid_topic_;

  /* Visualization */
  // System interface
  std::string current_position_vis_topic_, current_position_ground_vis_topic_;
  std::string current_region_vis_topic_;
  std::string predicted_positions_vis_topic_,
      predicted_positions_ground_vis_topic_;
  std::string predicted_regions_vis_topic_;
  // Modules
  std::string goal_position_vis_topic_, goal_position_ground_vis_topic_;
  std::string reference_trajectory_vis_topic_,
      reference_trajectory_ground_vis_topic_;
  std::string constraints_vis_topic_;

  /* Recording */
  // System interface
  std::string cur_state_rec_topic_, interp_data_topic_,
      nominal_reference_topic_, pred_traj_rec_topic_, slack_rec_topic_,
      time_reference_rec_topic_;
  // Modules
  std::string goal_position_rec_topic_;
  std::string reference_trajectory_rec_topic_;
  std::string constraints_rec_topic_;

  /* Hierarchical communication */
  std::string start_mpc_layer_topic_, ref_topic_, pmpc_obj_reached_topic_,
      pmpc_failure_topic_;
  /************************************************************/
};

#endif
