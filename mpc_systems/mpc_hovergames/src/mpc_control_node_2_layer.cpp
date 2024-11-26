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


#if defined(PX4)
  #include <mpc_hovergames/hovergames_px4_interface.h>
#elif defined(SIMPLESIM)
  #include <mpc_hovergames/hovergames_simplesim_interface.h>
#endif

int main(int argc, char *argv[])
{
  // check if there is more than one argument and use the second one
  // this is the specified solver
  //  (the first argument is the executable)
  std::string arg_solver;
  if (argc > 1) {arg_solver = argv[1];};

  std::string node_name;
  if (arg_solver == "mpc_0")
  {
    ROS_INFO_STREAM("Tracker solver is selected");
    node_name = "tracker";
    
  } else if (arg_solver == "mpc_1")
  {
    ROS_INFO_STREAM("Planner solver is selected");
    node_name = "planner";
  }

  ros::init(argc, argv, node_name+"_node");
  ros::NodeHandle nh("~");

  // Create the main mpc configuration struct
  ConfigurationMPC configuration_mpc;
  if (!mpc_configuration_initialize(nh, configuration_mpc)){return 1;}

  // Initialise class pointers
  std::unique_ptr<SolverBase> ptr_solver;
  std::unique_ptr<ReconfigureCallbackBase> ptr_reconfigure_callback;
  std::unique_ptr<InterfaceBase> ptr_interface;
  std::unique_ptr<ControllerBase> ptr_controller;
  std::unique_ptr<ModuleHandlerBase> ptr_module_handler;

  ros::Rate loop_rate(0.5);

  // Check which solver we need to load in
  if (node_name == "tracker")
  {
    ptr_solver = returnSolverPointer(0);
    // ptr_reconfigure_callback = std::unique_ptr<ReconfigureCallback<solver_config0>>(new ReconfigureCallback<solver_config0>(ptr_solver.get(), nh, &loadMPCWeights0));
    ReconfigureCallback<solver_config0> test(ptr_solver.get(), nh, &loadMPCWeights0);
    loop_rate.sleep();
  } else if (node_name == "planner")
  {
    ptr_solver = returnSolverPointer(1);
    // ptr_reconfigure_callback = std::unique_ptr<ReconfigureCallback<solver_config1>>(new ReconfigureCallback<solver_config1>(ptr_solver.get(), nh, &loadMPCWeights1));
    ReconfigureCallback<solver_config1> test(ptr_solver.get(), nh, &loadMPCWeights1);
    loop_rate.sleep();
  } else
  {
    ROS_ERROR_STREAM("No solver is specified");
    return 0;
  }

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

  return 0;
}
