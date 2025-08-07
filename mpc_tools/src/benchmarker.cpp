
#include <mpc_tools/benchmarker.h>

namespace Helpers {

Benchmarker::Benchmarker(const std::string& name, bool record_duration,
                         int ignore_first) {
  name_ = name;
  record_duration_ = record_duration;
  running_ = false;
  ignore_first_ = ignore_first;
}

void Benchmarker::initialize(const std::string& name, bool record_duration,
                             int ignore_first) {
  name_ = name;
  record_duration_ = record_duration;
  ignore_first_ = ignore_first;
}

// Print results on destruct
Benchmarker::~Benchmarker() {
  double average_run_time = total_duration_ / ((double)total_runs_) * 1000.0;

  std::cout << "Timing Results for [" << name_ << "]\n";
  std::cout << "Average: " << average_run_time << " ms\n";
  std::cout << "Min: " << min_duration_ * 1000.0 << " ms\n";
  std::cout << "Max: " << max_duration_ * 1000.0 << " ms\n";
}

void Benchmarker::start() {
  running_ = true;
  start_time_ = std::chrono::system_clock::now();
}

double Benchmarker::stop() {
  if (!running_) return 0.0;

  // Don't time the first 10, there may be some startup behavior
  if (total_runs_ < ignore_first_) {
    total_runs_++;
    return 0.0;
  }

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> current_duration = end_time - start_time_;

  if (record_duration_)
    duration_list_.push_back(current_duration.count() * 1000.0);  // in ms

  if (current_duration.count() < min_duration_)
    min_duration_ = current_duration.count();

  if (current_duration.count() > max_duration_)
    max_duration_ = current_duration.count();

  total_duration_ += current_duration.count();
  total_runs_++;
  running_ = false;

  last_ = current_duration.count();
  return last_;
}

void Benchmarker::dataToMessage(std_msgs::Float64MultiArray& msg) {
  msg.data.resize(duration_list_.size());

  for (size_t i = 0; i < duration_list_.size(); i++)
    msg.data[i] = duration_list_[i];
}

void Benchmarker::reset() {
  total_runs_ = 0;
  total_duration_ = 0.0;
  max_duration_ = -1.0;
  min_duration_ = 99999.0;
}

inline bool Benchmarker::isRunning() { return running_; }

inline int Benchmarker::getTotalRuns() { return total_runs_; }

inline double Benchmarker::getLast() { return last_; }

}; /* namespace Helpers*/
