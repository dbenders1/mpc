#include <mpc_core/functions/error_function.h>
#include <ros/console.h>

void errorPrintingSolver(int exit_code) {
  switch (exit_code) {
    case 0:
      ROS_WARN(
          "[Solver] Max iterations reached in solver. Check the ForcesPro "
          "documentation for more information (exit_code = 0)");
      break;

    case 2:
      ROS_WARN("[Solver] Specified timeout is reached (exit_code = 2)");
      break;

    case -4:
      ROS_WARN(
          "[Solver] Wrong number of inequalities input to solver (exit_code = "
          "-4)");
      break;

    case -5:
      ROS_WARN(
          "[Solver] Error occurred during matrix factorization (exit_code = "
          "-5)");
      break;

    case -6:
      ROS_WARN(
          "[Solver] NaN or INF occurred during functions evaluations "
          "(exit_code = -6)");
      break;

    case -7:
      ROS_WARN(
          "[Solver] The solver could not proceed. Check the ForcesPro "
          "documentation for more information (exit_code = -7)");
      break;

    case -8:
      ROS_WARN(
          "[Solver] The internal QP solver could not proceed. Check the "
          "ForcesPro for more information (exit_code = -8)");
      break;

    case -10:
      ROS_WARN(
          "[Solver] NaN or INF occurred during evaluation of functions and "
          "derivatives. Check the ForcesPro documentation for "
          "more details (exit_code = -10)");
      break;

    case -11:
      ROS_WARN(
          "[Solver] Invalid values in problem parameters (exit_code = -11)");
      break;

    case -100:
      ROS_WARN("[Solver] License error (exit_code = -100)");
      break;

    default:
      ROS_WARN_STREAM(
          "[Solver] Exit code is not known (exit_code = " << exit_code << ")");
      break;
  }
};
