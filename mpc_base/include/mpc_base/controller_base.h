#ifndef MPC_BASE_MPC_BASE
#define MPC_BASE_MPC_BASE

#include <mpc_base/interface_base.h>
#include <mpc_msgs/MpcHeader.h>

/**
 * @brief This class holds the functions that need to be accessed outside of the
 * main class. Using pointers we can use this base to ease compilation.
 *
 */

/**
 * @brief Enumerator used for tracking the control status
 */
enum class ControllerStatus {
  RESET = 0,
  WAITING_FOR_DATA = 1,
  SUCCESS = 2,
  FAILURE = 3,
  DONE = 4
};

class ControllerBase {
 public:
  ControllerStatus controller_status_; /* Controller status */

  ControllerBase() : controller_status_(ControllerStatus::WAITING_FOR_DATA) {};

  virtual ~ControllerBase() {};

  // This function is needed to load in the correct pointer to interface
  virtual void setInterface(InterfaceBase* ptr_interface) = 0;

  // These can be used in the interfaces
  virtual void controlLoop() = 0;
  virtual void OnStateReceived() = 0;
  virtual void OnReset() = 0;
  virtual void OnObjectiveReached() = 0;
  virtual void stopTimer() = 0;
  virtual bool timerIsRunning() = 0;
  virtual void startTimer(int max_control_loop_calls = -1) = 0;
  virtual int getCountSinceReset() = 0;
  virtual int getCountTotal() = 0;
  virtual void setMpcHeader(mpc_msgs::MpcHeader& mpc_header) = 0;
  virtual void resetTimerFlag() = 0;
  virtual double getTime() = 0;
  virtual void setTime(const double t) = 0;
};

#endif
