
#ifndef PRINTING_H
#define PRINTING_H

#include <ros/ros.h>

#include <iostream>
#include <string>

/** Logging Pragmas */
#define MPC_COMMON_MSG_START std::string("[MPC")

// Printing macros for all code having access to ptr_config_
#define MPC_INFO(msg)                 \
  do {                                \
    if (ptr_config_->debug_output_) { \
      std::string type = "info";      \
      MPC_PRINT_CONFIG(type, msg)     \
    }                                 \
  } while (0);

#define MPC_SUCCES(msg)               \
  do {                                \
    if (ptr_config_->debug_output_) { \
      std::string type = "succes";    \
      MPC_PRINT_CONFIG(type, msg)     \
    }                                 \
  } while (0);

#define MPC_WARN(msg)                 \
  do {                                \
    if (ptr_config_->debug_output_) { \
      std::string type = "warn";      \
      MPC_PRINT_CONFIG(type, msg)     \
    }                                 \
  } while (0);

#define MPC_ERROR(msg)          \
  do {                          \
    std::string type = "error"; \
    MPC_PRINT_CONFIG(type, msg) \
  } while (0);

#define MPC_INFO_STREAM(msg)          \
  do {                                \
    if (ptr_config_->debug_output_) { \
      std::string type = "info";      \
      MPC_PRINT_CONFIG(type, msg)     \
    }                                 \
  } while (0);

#define MPC_WARN_STREAM(msg)          \
  do {                                \
    if (ptr_config_->debug_output_) { \
      std::string type = "warn";      \
      MPC_PRINT_CONFIG(type, msg)     \
    }                                 \
  } while (0);

#define MPC_ERROR_STREAM(msg)   \
  do {                          \
    std::string type = "error"; \
    MPC_PRINT_CONFIG(type, msg) \
  } while (0);

#define MPC_INFO_ALWAYS(msg)    \
  do {                          \
    std::string type = "info";  \
    MPC_PRINT_CONFIG(type, msg) \
  } while (0);

#define MPC_WARN_ALWAYS(msg)    \
  do {                          \
    std::string type = "warn";  \
    MPC_PRINT_CONFIG(type, msg) \
  } while (0);

#define MPC_PRINT_CONFIG(type, msg)                                          \
  do {                                                                       \
    std::string msg_start = MPC_COMMON_MSG_START;                            \
    if (ptr_config_->n_layers_ > 1)                                          \
      msg_start = msg_start + " " + std::to_string(ptr_config_->layer_idx_); \
    msg_start.append("] ");                                                  \
    MPC_PRINT_ROS(type, msg_start, msg)                                      \
  } while (0);

// Printing macros for solver interface header (non-hierarchical)
#define MPC_INFO_SOLVER(msg)                              \
  do {                                                    \
    std::string type = "info";                            \
    MPC_PRINT_ROS(type, MPC_COMMON_MSG_START + "] ", msg) \
  } while (0);

#define MPC_WARN_SOLVER(msg)                              \
  do {                                                    \
    std::string type = "warn";                            \
    MPC_PRINT_ROS(type, MPC_COMMON_MSG_START + "] ", msg) \
  } while (0);

// Printing macros for solver interface header (hierarchical)
#define MPC_INFO_SOLVER_LAYER(layer_idx, msg)                          \
  do {                                                                 \
    std::string type = "info";                                         \
    std::string msg_start =                                            \
        MPC_COMMON_MSG_START + " " + std::to_string(layer_idx) + "] "; \
    MPC_PRINT_ROS(type, msg_start, msg)                                \
  } while (0);

#define MPC_WARN_SOLVER_LAYER(layer_idx, msg)                          \
  do {                                                                 \
    std::string type = "warn";                                         \
    std::string msg_start =                                            \
        MPC_COMMON_MSG_START + " " + std::to_string(layer_idx) + "] "; \
    MPC_PRINT_ROS(type, msg_start, msg)                                \
  } while (0);

// Common print macro
#define MPC_PRINT_ROS(type, msg_start, msg)  \
  do {                                       \
    if (type.compare("info") == 0) {         \
      ROS_INFO_STREAM(msg_start << msg);     \
    } else if (type.compare("warn") == 0) {  \
      ROS_WARN_STREAM(msg_start << msg);     \
    } else if (type.compare("error") == 0) { \
      ROS_ERROR_STREAM(msg_start << msg);    \
    } else {                                 \
      ROS_INFO_STREAM(msg_start << msg);     \
    }                                        \
  } while (0);

// From
// https://stackoverflow.com/questions/3692954/add-custom-messages-in-assert
#ifndef NDEBUG
#define MPC_ASSERT(Expr, Msg) __MPC_ASSERT(#Expr, Expr, __FILE__, __LINE__, Msg)
#define MPC_HOOK std::cout << __FILE__ << " Line " << __LINE__ << std::endl;
#define MPC_HOOK_MSG(MSG)                                                      \
  std::cout << __FILE__ << " Line " << __LINE__ << " [Message: " << MSG << "]" \
            << std::endl;
#else
#define MPC_ASSERT(Expr, Msg)
#define MPC_HOOK
#define MPC_HOOK_MSG(MSG)
#endif
inline void __MPC_ASSERT(const char *expr_str, bool expr, const char *file,
                         int line, const char *msg) {
  if (!expr) {
    std::cerr << "Assert failed:\t" << msg << "\n"
              << "Expected:\t" << expr_str << "\n"
              << "Source:\t\t" << file << ", line " << line << "\n";
    abort();
  }
}

#endif