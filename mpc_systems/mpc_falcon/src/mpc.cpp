#include <mpc_falcon/mpc.h>

Mpc::Mpc(const int layer_idx) {
  // Initialize the mpc configuration
  mpc_configuration_initialize(nh_, config_mpc_);

  nh_ = ros::NodeHandle("~");

  // Overwrite the layer index in the config
  config_mpc_.layer_idx_ = layer_idx;

  // Initialize the solver pointer
  ptr_solver_ = returnSolverPointer(0);

  // Initialize the dynamic reconfiguration
  ReconfigureCallback<solver_config0> test(nh_, &config_mpc_, ptr_solver_.get(),
                                           &loadMPCWeights0);

  // Initialize the robot region
  ptr_robot_region_ = std::unique_ptr<RobotRegion>(new RobotRegion(
      Eigen::Vector2d(0, 0), 0, ptr_solver_->n_discs_, config_mpc_.robot_width_,
      config_mpc_.robot_length_, config_mpc_.robot_center_of_mass_to_back_));

  // Initialize the JSON data saver
  ptr_data_saver_json_ = std::unique_ptr<DataSaverJson>(new DataSaverJson());

  // Initialize the module handler
  ptr_module_handler_ = std::unique_ptr<ModuleHandler>(new ModuleHandler(
      nh_, &config_mpc_, ptr_solver_.get(), ptr_data_saver_json_.get()));

  // Initialize the controller
  ptr_controller_ = std::unique_ptr<Controller>(
      new Controller(nh_, &config_mpc_, ptr_solver_.get(), ptr_interface_.get(),
                     ptr_module_handler_.get(), ptr_data_saver_json_.get()));

  // Initialize the interface pointer
  std::mutex mutex;
  ptr_interface_ = std::unique_ptr<InterfaceBase>(new FalconInterface(
      nh_, ptr_controller_.get(), &config_mpc_, ptr_solver_.get(),
      ptr_module_handler_.get(), ptr_robot_region_.get(),
      ptr_data_saver_json_.get(), &mutex));

  // Correctly set the interface pointer in the controller after assigning the
  // interface pointer
  ptr_controller_->setInterface(ptr_interface_.get());
}

Mpc::~Mpc() { ptr_controller_.reset(); }

bool Mpc::setInterval(const double dt) {
  return ptr_interface_->setInterval(dt);
}

bool Mpc::setLastCommand(const Eigen::VectorXd &last_command) {
  return ptr_interface_->setLastCommand(last_command);
}

bool Mpc::setState(const double t, const Eigen::VectorXd &state) {
  return ptr_interface_->setState(t, state);
}

bool Mpc::run() { return ptr_interface_->run(); }

Eigen::VectorXd Mpc::getCommand() { return ptr_interface_->getCommand(); }
