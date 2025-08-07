#ifndef __DATA_SAVER_JSON_H__
#define __DATA_SAVER_JSON_H__

#define TAB_SIZE 2

#include <ros/package.h>  // getPath

#include <boost/filesystem.hpp>  // create_directories
#include <chrono>                // system_clock
#include <fstream>               // ofstream, open, close
#include <iomanip>               // setw
#include <iostream>              // cout, endl
#include <string>                // string, to_string

#include "thirth_party/json.hpp"  // json

using json = nlohmann::json;

class DataSaverJson {
 public:
  /* Constructor and destructor */
  /**
   * @brief Construct a new Data Saver JSON object
   *
   */
  DataSaverJson();

  /**
   * @brief Destroy the Data Saver JSON object after saving the data to a file
   *
   */
  ~DataSaverJson() {
    // Only save data if layer index is set
    if (store_in_file_) StoreInFile();
  }

  /* Main public methods */
  /**
   * @brief Add static data to the json object. Supports STL containers (see
   * https://github.com/nlohmann/json#conversion-from-stl-containers)
   *
   * @tparam T
   * @param data_name
   * @param data_value
   */
  template <typename T>
  void AddStaticData(const std::string &&data_name, const T &data_value) {
    // Save static data in static data object
    data_["static_data"][data_name] = data_value;
  }

  /**
   * @brief Add runtime data to the json object. Supports STL containers (see
   * https://github.com/nlohmann/json#conversion-from-stl-containers)
   *
   * @tparam T
   * @param data_name Name of the data object
   * @param count_since_start Control loop iteration count since start of the
   * current control run
   * @param count_total Control loop iteration count since start of node
   * @param data_value Data of the data object
   */
  template <typename T>
  void AddRuntimeData(const std::string &&data_name,
                      const int &count_since_start, const int &count_total,
                      const T &data_value) {
    // Create json object for runtime data element
    json data;
    data["number"] = runtime_data_count_;
    data["time"] = std::chrono::duration<double>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
    data["name"] = data_name;
    data["countSinceStart"] = count_since_start;
    data["countTotal"] = count_total;
    data["value"] = data_value;

    // Save runtime data element in runtime data array
    data_["runtime_data"].push_back(data);

    runtime_data_count_++;
  }

  /* Setters */
  /**
   * @brief Indicate whether to store the data in a JSON file or not
   *
   */
  void set_store_in_file(bool store_in_file);

  /**
   * @brief Set the exp name object
   *
   * @param exp_name
   */
  void set_exp_name(std::string &exp_name);

  /**
   * @brief Set the mpc layer idx object
   *
   * @param layer_idx
   */
  void set_layer_idx(int layer_idx);

 private:
  /* Main private methods */
  /**
   * @brief Save data in JSON format to specified file in
   * mpc_tools/recorded_data
   *
   */
  void StoreInFile();

  /* Helper methods */
  /**
   * @brief Parse the year in tm struct to a string that makes sense
   *
   * @param value
   * @return std::string
   */
  std::string ParseYear(int value);

  /**
   * @brief Determine whether to put a zero in front of month, day, hour or
   * minutes
   *
   * @param value
   * @return std::string
   */
  std::string ValueWithZero(int value);

  /* Members */
  // json data object
  json data_;
  int runtime_data_count_;

  // Output file
  std::string start_date_time_stamp_;
  std::string exp_name_;
  int layer_idx_ = -1;

  // Indication whether to save data
  bool store_in_file_ = false;
};

#endif  // __DATA_SAVER_JSON_H__
