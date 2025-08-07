#ifndef MPC_TOOLS_BENCHMARKER_H
#define MPC_TOOLS_BENCHMARKER_H

#include <std_msgs/Float64MultiArray.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

namespace Helpers {

// Use as static to print average run time
class Benchmarker {
 public:
  Benchmarker(const std::string& name, bool record_duration = false,
              int ignore_first = 10);

  // Simpler
  Benchmarker() {}

  void initialize(const std::string& name, bool record_duration = false,
                  int ignore_first = 10);

  // Print results on destruct
  ~Benchmarker();

  void start();

  double stop();

  void dataToMessage(std_msgs::Float64MultiArray& msg);

  void reset();

  bool isRunning();

  int getTotalRuns();

  double getLast();

 private:
  std::chrono::system_clock::time_point start_time_;

  double total_duration_ = 0.0;
  double max_duration_ = -1.0;
  double min_duration_ = 99999.0;

  double last_ = -1.0;

  int total_runs_ = 0;

  std::string name_;
  bool record_duration_;
  std::vector<double> duration_list_;
  bool running_ = false;

  int ignore_first_;
};

}; /* namespace Helpers*/

#endif