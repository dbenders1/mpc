#include <mutex>

#include <ros/ros.h>

#include <mpc_solver/hovergames/hovergames_solver_interface.h>
#include <mpc_solver/hovergames/dynamic_reconfigure_files/reconfigure_callback_options.h>

#include <mpc_base/solver_base.h>
#include <mpc_base/configuration_mpc.h>
#include <mpc_base/reconfigure_callback_base.h>
#include <mpc_base/interface_base.h>
#include <mpc_base/controller_base.h>


#include <mpc_core/functions/update_mpc_configuration.h>
#include <mpc_core/interface.h>
#include <mpc_core/controller.h>
#include <mpc_core/reconfigure_callback.h>

#include <mpc_modules/loader/module_handler.h>

#include <mpc_tools/robot_region.h>
#include <mpc_tools/data_saver_json.h>

#ifdef PX4
  #include <mpc_hovergames/hovergames_px4_interface.h>
#endif
#ifdef SIMPLESIM
  #include <mpc_hovergames/hovergames_simplesim_interface.h>
#endif

// remove later on to its own function but first check if it works


int main(int argc, char *argv[])
{
  // check if there is more than one argument and use the second one
  // this is the specified solver
  //  (the first argument is the executable)
  std::string arg_solver;
  if (argc > 1) {arg_solver = argv[1];};

  ros::init(argc, argv, arg_solver+"_node");
  ros::NodeHandle nh("~");

  // Create the main mpc configuration struct
  // Should be made as first
  ConfigurationMPC configuration_mpc;
  if (!mpc_configuration_initialize(nh, configuration_mpc)){return 1;}

  // Initialise class pointers
  std::unique_ptr<SolverBase> ptr_solver;
  std::unique_ptr<ReconfigureCallbackBase> ptr_reconfigure_callback;
  std::unique_ptr<InterfaceBase> ptr_interface;
  std::unique_ptr<ControllerBase> ptr_controller;
  std::unique_ptr<ModuleHandlerBase> ptr_module_handler;

  ptr_solver = returnSolverPointer(0);
  ros::Rate loop_rate(0.5);
  ReconfigureCallback<solver_config0> test(ptr_solver.get(), nh, &loadMPCWeights0);
  loop_rate.sleep();
  // ptr_reconfigure_callback = std::unique_ptr<ReconfigureCallback<solver_config0>>(new ReconfigureCallback<solver_config0>(ptr_solver.get(), nh, &loadMPCWeights0));

  // Construct the robot region and data saver objects
  std::unique_ptr<RobotRegion> ptr_robot_region = std::unique_ptr<RobotRegion>(new RobotRegion(Eigen::Vector2d(0, 0), 0, ptr_solver->n_discs_, configuration_mpc.robot_width_,
                                        configuration_mpc.robot_length_, configuration_mpc.robot_center_of_mass_to_back_));
  std::unique_ptr<DataSaverJson> ptr_data_saver_json = std::unique_ptr<DataSaverJson>(new DataSaverJson());
  
  // Create a mutex to ensure thread safety in case of multithreading
  std::mutex mutex;

  // Initialise the modules, Interface setup en controller setup
  ptr_module_handler = std::unique_ptr<ModuleHandler>(new ModuleHandler(nh, &configuration_mpc, ptr_solver.get(), ptr_data_saver_json.get()));
  ptr_controller = std::unique_ptr<Controller>(new Controller(nh, &configuration_mpc, ptr_solver.get(), ptr_interface.get(), ptr_module_handler.get(), ptr_data_saver_json.get()));

#ifdef PX4
  ptr_interface = std::unique_ptr<HovergamesPX4Interface>(new HovergamesPX4Interface(nh, ptr_controller.get(), &configuration_mpc, ptr_solver.get(), ptr_module_handler.get(), ptr_robot_region.get(), ptr_data_saver_json.get(), &mutex)); 
#endif
#ifdef SIMPLESIM
  ptr_interface = std::unique_ptr<HovergamesSimpleSimInterface>(new HovergamesSimpleSimInterface(nh, ptr_controller.get(), &configuration_mpc, ptr_solver.get(), ptr_module_handler.get(), ptr_robot_region.get(), ptr_data_saver_json.get(), &mutex)); 
#endif


  // To pass in the correct pointer since after resetting unique pointer, it gets a new pointer value
  ptr_controller->setInterface(ptr_interface.get());
  
  ros::spin();

  ptr_controller.reset();

  return 0;
}
