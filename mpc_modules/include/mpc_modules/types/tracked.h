#ifndef MPC_MODULES_TYPES_TRACKED_H
#define MPC_MODULES_TYPES_TRACKED_H

#include <utility>
#include <vector>

/**
 * @brief A wrapper class to track if data was used yet and if new data has been
 * received
 *
 * @tparam T The variable type
 */
template <class T>
class Tracked {
 public:
  Tracked() { data_is_new_ = false; };

 public:
  void Set(const T &new_value) {
    value_ = new_value;
    data_is_new_ = true;
  }

  /**
   * @brief Get the data
   *
   * @param keep_data_flag Set to true if the operation should not change the
   * used status of the data to false
   * @return T&
   */
  T &Get(bool keep_data_flag = false) {
    data_is_new_ = false & keep_data_flag;
    return value_;
  };

  bool DataIsNew() const { return data_is_new_; }

 private:
  T value_;
  bool data_is_new_;
};

/**
 * @brief A wrapper for std::vector that tracks if new data was inserted. To be
 * used for tracking the arrival of real-time data.
 *
 * @tparam T
 */
template <class T>
class TrackedVector : public std::vector<T> {
 public:
  TrackedVector() : std::vector<T>() { data_is_new_ = false; };

  /**
   * @brief Copy constructor from a vector
   *
   * @param other a std::vector
   */
  TrackedVector(const std::vector<T> &other) : std::vector<T>(other) {
    data_is_new_ = true;
  };

 public:
  /**
   * @brief A wrapper for emplace_back, sets new data arrived to true
   *
   * @tparam Args
   * @param args
   */
  template <typename... Args>
  void emplace_back(Args &&...args) {
    std::vector<T>::emplace_back(std::forward<Args>(args)...);
    data_is_new_ = true;
  }

  /**
   * @brief Wrapper for push_back that sets new data arrived to true
   *
   * @param new_value
   */
  void push_back(const T &new_value) {
    std::vector<T>::push_back(new_value);
    data_is_new_ = true;
  }

  /**
   * @brief True if new data was received and not yet used
   *
   * @return true
   * @return false
   */
  bool DataIsNew() const { return data_is_new_; }

 private:
  bool data_is_new_;
};

#endif