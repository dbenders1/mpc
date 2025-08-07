#ifndef MPC_CORE_UPDATE_MPC_CONFIGURATION
#define MPC_CORE_UPDATE_MPC_CONFIGURATION

#include <mpc_base/configuration_mpc.h>
#include <ros/node_handle.h>

#include <string>

/**
 * @brief intialize: Read all parameters from the ros parameter server
 *
 * @param nh ros nodehandle
 * @param config de mpc configuration file with all the settings
 * @return TRUE iff all parameter initialize successfully
 */
bool mpc_configuration_initialize(const ros::NodeHandle& nh,
                                  ConfigurationMPC& config);

/**
 * @brief function that is called when the ros param has no default value and is
 * not defined
 *
 * @param name ros param name
 * @return returns false to stop initialisation
 */
bool errorFunc(const std::string name);

/**
 * @brief function that is called when default parameter is used
 *
 * @param name ros param name
 */
void defaultFunc(const std::string name);

/**
 * @brief function that prints a warning, called when ROS recordings are
 * disabled, but the recording variable is set to true
 *
 * @param name
 */
void rosRecordingWarning(const std::string name);

#endif
