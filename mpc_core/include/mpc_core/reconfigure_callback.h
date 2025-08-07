#ifndef MPC_CORE_RECONFIGURE_CALLBACK
#define MPC_CORE_RECONFIGURE_CALLBACK

#include <dynamic_reconfigure/server.h>
#include <mpc_base/configuration_mpc.h>
#include <mpc_base/reconfigure_callback_base.h>
#include <mpc_base/solver_base.h>
#include <mpc_tools/printing.h>
#include <ros/node_handle.h>

#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <functional>

template <class TConfig>
class ReconfigureCallback : public ReconfigureCallbackBase {
 public:
  boost::recursive_mutex reconfig_mutex_;
  bool first_reconfigure_callback_ = true;

  //    Interface* system_interface_;
  SolverBase* ptr_solver_;
  ConfigurationMPC* ptr_config_;
  ros::NodeHandle nh_;
  // For HMPC: use dynamic reconfigure object
  // For SMPC: use dynamic reconfigure object pointer
  dynamic_reconfigure::Server<TConfig> reconfigure_server_;
  // dynamic_reconfigure::Server<TConfig>* reconfigure_server_;

  // Pointer to the load weights function from the solver interface
  void (*func_)(TConfig&, SolverBase*);

  ReconfigureCallback(ros::NodeHandle& nh, ConfigurationMPC* ptr_config,
                      SolverBase* ptr_solver,
                      void (*func)(TConfig&, SolverBase*))
      : nh_(nh),
        ptr_config_(ptr_config),
        ptr_solver_(ptr_solver),
        reconfigure_server_(reconfig_mutex_, nh_),
        func_(func) {
    // reconfigure_server_ = new
    // dynamic_reconfigure::Server<TConfig>(reconfig_mutex_, nh_);
    func_ = func;
    reconfigure_server_.setCallback(boost::bind(
        &ReconfigureCallback<TConfig>::callbackFunction, this, _1, _2));
    // reconfigure_server_->setCallback(boost::bind(&ReconfigureCallback<TConfig>::callbackFunction,
    // this, _1, _2));
    MPC_INFO("Setup dynamic_reconfigure server for the parameters");
  }

  void callbackFunction(TConfig& ptr_callback_config, uint32_t level) {
    MPC_INFO("Reconfigure callback!");

    // Load the weights using external function from ptr_solver pkg
    func_(ptr_callback_config, ptr_solver_);

    if (first_reconfigure_callback_) {
      // ptr_callback_config.enable_output = ptr_config_->enable_output_;
      // ptr_callback_config.enable_debug = ptr_config_->debug_output_;

      first_reconfigure_callback_ = false;

    } else {
      // ptr_config_->enable_output_ = ptr_callback_config.enable_output;
      // ptr_config_->debug_output_ = ptr_callback_config.enable_debug;
    }

    ptr_solver_->print_solver_info_ = ptr_callback_config.print_solver_info;
    ptr_solver_->print_solver_param_ =
        ptr_callback_config.print_solver_parameters;
    ptr_solver_->print_solver_output_ = ptr_callback_config.print_solver_output;
    ptr_solver_->print_solver_data_ = ptr_callback_config.enable_debug_solver;
  }
};

#endif