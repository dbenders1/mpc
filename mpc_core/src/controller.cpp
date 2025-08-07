
#include <mpc_core/controller.h>
#include <mpc_core/functions/error_function.h>
#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>

#include <cstdlib>
#include <string>

Controller::Controller(ros::NodeHandle& nh, ConfigurationMPC* ptr_config,
                       SolverBase* ptr_solver, InterfaceBase* ptr_interface,
                       ModuleHandlerBase* ptr_module_handler,
                       DataSaverJson* ptr_data_saver_json)
    : nh_(nh),
      ptr_config_(ptr_config),
      ptr_solver_(ptr_solver),
      ptr_interface_(ptr_interface),
      ptr_module_handler_(ptr_module_handler),
      ptr_data_saver_json_(ptr_data_saver_json),
      timer_(nh.createTimer(
          ros::Duration(ptr_solver->dt_),
          std::bind(&Controller::runNode, this, std::placeholders::_1))) {
  MPC_WARN_ALWAYS("Initializing controller");

  // Initialize profiling
  Helpers::Instrumentor::Get().BeginSession(
      "mpc", std::string("mpc_") + std::to_string(ptr_config_->layer_idx_),
      std::string("mpc_profiler_") + std::to_string(ptr_config_->layer_idx_) +
          std::string(".json"));

  // Benchmarking
  control_loop_benchmarker_.initialize("Full Control Loop", true);
  optimization_benchmarker_.initialize("Optimization", true);
  module_benchmarkers_.initialize("Module Updates", true);

  // See if any modules can update
  ptr_module_handler_->onDataReceived("Init");

  // Setup publisher for computation times
  if (ptr_config_->n_layers_ > 1) {
    pub_computation_times_ = nh.advertise<sensor_msgs::Temperature>(
        "/mpc/computation_times/" + std::to_string(ptr_config_->layer_idx_), 1);
  } else {
    pub_computation_times_ =
        nh.advertise<sensor_msgs::Temperature>("/mpc/computation_times", 1);
  }
  computation_time_msg_.header.frame_id = "mpc";
  computation_time_msg_.temperature = 0.0;

  std::vector<int> indexes = ptr_solver_->returnDataPositionsState({"x", "y"});
  x_data_position_ = indexes[0];
  y_data_position_ = indexes[1];

  // Set random seed number for reproducibility of the random matrix generation
  // in the control loop
  std::srand(ptr_config_->random_seed_);

  // Stop the control loop timer by default, so its start can be regulated in
  // the system interface
  timer_.stop();

  MPC_WARN_ALWAYS("Controller initialized");
}

Controller::~Controller() {
  ptr_interface_->DeployEmergencyStrategy();

  // Save profiling data
  Helpers::Instrumentor::Get().EndSession();
}

void Controller::setInterface(InterfaceBase* ptr_interface) {
  ptr_interface_ = ptr_interface;

  // Setup recording for data now that interface pointer is also correct
  if (ptr_config_->record_experiment_json_) {
    // Get name based on the modules
    ptr_config_->recording_name_json_ += ptr_module_handler_->getMethodName();
    MPC_WARN_STREAM(
        "Recording json file with name: " << ptr_config_->recording_name_json_);

    // Configure JSON data saver with experiment name and MPC layer index
    ptr_data_saver_json_->set_store_in_file(true);
    ptr_data_saver_json_->set_exp_name(ptr_config_->recording_name_json_);
    ptr_data_saver_json_->set_layer_idx(ptr_config_->layer_idx_);

    // Add static data about general MPC problem setup
    ptr_data_saver_json_->AddStaticData("dt", ptr_solver_->dt_);
    ptr_data_saver_json_->AddStaticData("N", ptr_solver_->n_);

    // Add static JSON data in system interface
    ptr_interface_->AddStaticData();
  } else {
    MPC_WARN("Recording is disabled");
  }
  // Now we need to fire the forces solver ones to activate the license
  // especially for floating licenses, which takes time.
  {
    PROFILE_AND_LOG_WARN("First forces fire");
    bool store_printing_setting = ptr_solver_->print_solver_data_;
    ptr_solver_->print_solver_data_ = false;
    int solver_exit = ptr_solver_->runSolverStep();
    errorPrintingSolver(solver_exit);
    ptr_solver_->print_solver_data_ = store_printing_setting;
  }
}

double Controller::getTime() {
  if (t_ >= 0)
    return t_;
  else
    return ros::Time::now().toSec();
}

void Controller::setTime(const double t) { t_ = t; }

void Controller::runNode(const ros::TimerEvent& event) { controlLoop(); }

void Controller::controlLoopStart() {
  PROFILE_FUNCTION();

  computation_time_msg_.header.stamp = ros::Time(getTime());
  pub_computation_times_.publish(computation_time_msg_);

  mpc_header_.t_start = ros::Time(getTime());
  mpc_header_.count_since_reset = timer_.getCountSinceReset();
  mpc_header_.count_since_start = timer_.getCountSinceStart();
  mpc_header_.count_total = timer_.getCountTotal();

  control_loop_benchmarker_.start();

  MPC_INFO("");
  MPC_INFO("============ START OF CONTROL LOOP ============");

  MPC_INFO("Time: " << std::setprecision(10) << getTime());
  MPC_INFO("Count total: " << timer_.getCountTotal());
  MPC_INFO("Count since reset: " << timer_.getCountSinceReset());
}

void Controller::controlLoopEnd() {
  timer_.countUp();

  MPC_INFO("============= END OF CONTROL LOOP =============");
  MPC_INFO("");

  double duration = control_loop_benchmarker_.stop();
  computation_time_msg_.temperature = duration;
}

void Controller::controlLoopEndNotCompleted() {
  timer_.countUpNotCompleted();

  MPC_INFO("============= END OF CONTROL LOOP =============");
  MPC_INFO("");
}

void Controller::controlLoop() {
  PROFILE_FUNCTION();
  control_loop_profiling_string_ =
      "controlLoop (count total: " + std::to_string(timer_.getCountTotal()) +
      ")";

  {
    PROFILE_SCOPE(control_loop_profiling_string_.c_str());
    controlLoopStart();

    // Actuate at the start of the control loop is we expect one control loop of
    // delay
    if (ptr_config_->synchronized_actuation_) {
      ptr_interface_->Actuate();
    };

    // System interface-dependant computations at start control loop
    // Need to call it at this place, because module data might need to be
    // updated in this function before the checkModulesReady() call
    ptr_interface_->AtControlLoopStart();

    controller_status_ = ControllerStatus::WAITING_FOR_DATA;

    bool modules_are_ready_ = ptr_module_handler_->checkModulesReady();
    objective_reached_ = ptr_module_handler_->checkObjectiveReached();

    if (objective_reached_) {
      PROFILE_AND_LOG_WARN("Objective reached");
      ptr_interface_->DeployObjectiveReachedStrategy();
      controlLoopEndNotCompleted();
      return;
    }

    // The control loop runs if we are planning and all data was received else
    // we return from the loop
    if (!ptr_config_->plan_ || !modules_are_ready_ ||
        (ptr_config_->layer_idx_ == 0 && !state_received_)) {
      PROFILE_SCOPE("No control");
      if (!ptr_config_->plan_) {
        PROFILE_AND_LOG_WARN("Planning disabled");
      }
      if (!modules_are_ready_) {
        PROFILE_AND_LOG_WARN("Modules not ready");
      }
      if (!state_received_) {
        PROFILE_AND_LOG_WARN("No state received");
      }
      ptr_interface_->DeployEmergencyStrategy();
      controlLoopEndNotCompleted();
      return;
    }

    // Do not run the control loop in synchronous mode with simple_sim and
    // lowest layer MPC before having received a state update
    if (ptr_config_->sync_simple_sim_ && ptr_config_->layer_idx_ == 0 &&
        !updated_state_received_) {
      PROFILE_SCOPE("No control simplesim");
      MPC_WARN(
          "Need to sync with simplesim, but no state received from simplesim "
          "yet");
      controlLoopEndNotCompleted();
      return;
    } else {
      updated_state_received_ = false;
    };

    // Everything OK with the checks
    controller_status_ = ControllerStatus::SUCCESS;

    // In the first full run, we have to load in the state in all stages since
    // stage_vars_ is still empty
    if (first_loop_) {
      ptr_solver_->updateMeasuredStateInHorizon();
      first_loop_ = false;
    }

    // Update modules and system interface accordingly
    // Note: module updates should come before shiftHorizonInMatrix(), because
    // information is lost by this step (initial state of previous run), which
    // is used in some modules
    {
      PROFILE_AND_LOG_INFO("Modules update");
      module_benchmarkers_.start();
      ptr_module_handler_->updateModules();
      module_benchmarkers_.stop();
    }
    ptr_interface_->AfterUpdateModules();

    // Print resulting real-time data if desired
    if (ptr_config_->debug_data_) {
      ptr_module_handler_->printRealTimeData();
    };

    // Load warm start (using time shift, by default 1), initial state and
    // parameters
    if (!first_loop_) {
      ptr_solver_->shiftHorizonInMatrix(ptr_config_->time_shift_);
    }
    ptr_solver_->updateMeasuredState();
    {
      PROFILE_AND_LOG_INFO("Modules set parameters");
      ptr_module_handler_->setParametersObjective();
      ptr_module_handler_->setParametersConstraint();
    }
    ptr_interface_->AfterSetParametersModules();

    if (ptr_config_->enable_ros_recordings_) {
      PROFILE_AND_LOG_INFO("Modules publish data");
      ptr_module_handler_->publishData(mpc_header_);
    }

    // Save JSON data if desired
    if (ptr_config_->record_experiment_json_) {
      ptr_module_handler_->exportDataJson(timer_.getCountSinceStart(),
                                          timer_.getCountTotal());
    };

    // OPTIMIZATION: input the data into the solver structs and solve the
    // optimisation
    {
      PROFILE_AND_LOG_INFO("Optimization");
      optimization_benchmarker_.start();

      previous_exit_code_ = exit_code_;

      // In TMPC: use reference trajectory to warm-start the solver
      if (ptr_config_->n_layers_ > 1 && ptr_config_->layer_idx_ == 0) {
        ptr_solver_->stage_vars_.topRightCorner(4, ptr_solver_->n_) =
            ptr_solver_->par_objectives_.topRightCorner(4, ptr_solver_->n_);
        ptr_solver_->stage_vars_.bottomRightCorner(ptr_solver_->nx_,
                                                   ptr_solver_->n_) =
            ptr_solver_->par_objectives_.bottomRightCorner(ptr_solver_->nx_,
                                                           ptr_solver_->n_);

        // Improve the warm-start of the solver to avoid reaching the maximum
        // number of iterations
        Eigen::MatrixXd rand =
            1 * Eigen::MatrixXd::Random(ptr_solver_->nu_ + ptr_solver_->nx_,
                                        ptr_solver_->n_ - 1) -
            0.5 * Eigen::MatrixXd::Ones(ptr_solver_->nu_ + ptr_solver_->nx_,
                                        ptr_solver_->n_ - 1);
        ptr_solver_->stage_vars_.topRightCorner(
            ptr_solver_->nu_ + ptr_solver_->nx_, ptr_solver_->n_ - 1) += rand;
      }

      exit_code_ = ptr_solver_->runSolverStep();
      int i = 0;
      while ((exit_code_ == 0 || exit_code_ == -6 || exit_code_ == -7) &&
             i < 3) {
        MPC_WARN_STREAM("[Solver] exit_code: " << exit_code_
                                               << "    iter: " << i);

        Eigen::MatrixXd rand =
            1 * Eigen::MatrixXd::Random(ptr_solver_->nu_ + ptr_solver_->nx_,
                                        ptr_solver_->n_ - 1) -
            0.5 * Eigen::MatrixXd::Ones(ptr_solver_->nu_ + ptr_solver_->nx_,
                                        ptr_solver_->n_ - 1);
        ptr_solver_->stage_vars_.topRightCorner(
            ptr_solver_->nu_ + ptr_solver_->nx_, ptr_solver_->n_ - 1) += rand;

        exit_code_ = ptr_solver_->runSolverStep();
        i++;
      };

      optimization_benchmarker_.stop();
    }

    ptr_interface_->AfterOptimization();

    if (exit_code_ == 1) {
      PROFILE_AND_LOG_INFO("Success");
      controller_status_ = ControllerStatus::SUCCESS;

      if (timer_.getCountSinceReset() != 0 && exit_code_ != previous_exit_code_)
        PROFILE_AND_LOG_WARN("Recovered from infeasibility");

      ptr_module_handler_->onDataReceived("Plan");

      if (ptr_config_->enable_output_) {
        ptr_interface_->ComputeActuation();
        if (!ptr_config_->synchronized_actuation_) {
          ptr_interface_->Actuate();
        };
      } else {
        MPC_WARN("Output is disabled. Deploy emergency strategy");
        ptr_interface_->DeployEmergencyStrategy();
      }

      MPC_INFO("Creating visualizations");
      ptr_module_handler_->createVisualizationsModules();
      ptr_interface_->CreateVisualizations();
    } else {
      PROFILE_AND_LOG_INFO("Infeasible");
      controller_status_ = ControllerStatus::FAILURE;
      printInfoAtNoSuccess(exit_code_);

      ptr_interface_->DeployEmergencyStrategy();
    }

    controlLoopEnd();
  }
}

void Controller::startTimer(int max_control_loop_calls) {
  timer_.start(max_control_loop_calls);
  runNode(ros::TimerEvent());
}

void Controller::stopTimer() { timer_.stop(); }

bool Controller::timerIsRunning() { return timer_.isRunning(); }

int Controller::getCountSinceReset() { return timer_.getCountSinceReset(); }

int Controller::getCountTotal() { return timer_.getCountTotal(); }

void Controller::setMpcHeader(mpc_msgs::MpcHeader& mpc_header) {
  mpc_header = mpc_header_;
}

void Controller::resetTimerFlag() { timer_.resetFlag(); }

void Controller::OnStateReceived() {
  MPC_INFO("OnStateReceived()");

  double current_x = ptr_solver_->last_updated_stage_vars_(x_data_position_, 0);
  double current_y = ptr_solver_->last_updated_stage_vars_(y_data_position_, 0);
  bool state_jumped = ((Eigen::Vector2d(current_x, current_y) -
                        Eigen::Vector2d(prev_x_, prev_y_))
                           .norm() > 5.0);
  prev_x_ = current_x;
  prev_y_ = current_y;

  updated_state_received_ = true;

  // Continue if first state received or state jumped
  if (state_received_ && !state_jumped) {
    return;
  };

  if (!first_loop_) {
    MPC_WARN("Jump in state detected!");
  };

  state_received_ = true;

  ptr_solver_->updateMeasuredStateInHorizon();
}

void Controller::OnReset() {
  MPC_INFO("OnReset()");
  controller_status_ = ControllerStatus::RESET;

  // Disable planning / output
  bool prev_plan = ptr_config_->plan_;
  bool prev_enable_output = ptr_config_->enable_output_;
  ptr_config_->plan_ = false;
  ptr_config_->enable_output_ = false;

  // Stop the timer
  if (ptr_config_->sync_mode_) {
    timer_.stop();
  };

  updated_state_received_ = false;

  // Reset other classes
  ptr_interface_->Reset();
  ptr_module_handler_->onReset();

  timer_.resetCount();

  // Start the timer again
  if (ptr_config_->sync_mode_) {
    timer_.start();
  };

  // Will trigger a state update before the controller starts
  objective_reached_ = false;

  // Reenable planning and the output
  ptr_config_->plan_ = prev_plan;
  ptr_config_->enable_output_ = prev_enable_output;

  MPC_INFO_ALWAYS("Reset Completed");
}

void Controller::printInfoAtNoSuccess(int& exit_code) {
  errorPrintingSolver(exit_code);
}

void Controller::OnObjectiveReached() {
  MPC_INFO("OnObjectiveReached()");
  objective_reached_ = true;
}
