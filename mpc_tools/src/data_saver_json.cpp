#define TAB_SIZE 2

#include <mpc_tools/data_saver_json.h>
#include <ros/package.h>

using json = nlohmann::json;

DataSaverJson::DataSaverJson() {
  // Determine data and time at start of experiment
  auto tt =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  auto local_time = *localtime(&tt);

  start_date_time_stamp_ = ParseYear(local_time.tm_year) + "-" +
                           ValueWithZero(local_time.tm_mon + 1) + "-" +
                           ValueWithZero(local_time.tm_mday) + "_" +
                           ValueWithZero(local_time.tm_hour) + "-" +
                           ValueWithZero(local_time.tm_min);

  // Initialize json data
  data_["static_data"] = json::object();
  data_["runtime_data"] = json::array();
  // Reserve memory for at least 100 json objects in the runtime data array
  json::array_t* data_ptr = data_["runtime_data"].get_ptr<json::array_t*>();
  data_ptr->reserve(100);

  // Set runtime data counter to 0
  runtime_data_count_ = 0;
}

void DataSaverJson::set_store_in_file(bool store_in_file) {
  store_in_file_ = store_in_file;
}

void DataSaverJson::set_exp_name(std::string& exp_name) {
  exp_name_ = exp_name;
}

void DataSaverJson::set_layer_idx(int layer_idx) { layer_idx_ = layer_idx; }

void DataSaverJson::StoreInFile() {
  // Create directory path if it does not exist yet
  // Data is always stored in mpc_tools/recorded_data
  std::string dir_path = ros::package::getPath("mpc_tools") + "/recorded_data";
  if (boost::filesystem::create_directories(dir_path))
    std::cout << "Data saver JSON: Creating directory path " << dir_path
              << std::endl;

  // Determine file path based on available data
  std::string file_name = start_date_time_stamp_ + "_" + exp_name_;
  if (layer_idx_ != -1) file_name += "_" + std::to_string(layer_idx_);
  file_name += ".json";
  std::string file_path = dir_path + "/" + file_name;

  // Save json object to file with tab size TAB_SIZE
  std::ofstream file;
  file.open(file_path);
  file << std::setw(TAB_SIZE) << data_ << std::endl;
  file.close();
  std::cout << "Data saver JSON: Data saved in " << file_path << std::endl;
}

std::string DataSaverJson::ParseYear(int value) {
  return "20" + std::to_string(value).erase(0, 1);
}

std::string DataSaverJson::ValueWithZero(int value) {
  if (value < 10)
    return "0" + std::to_string(value);
  else
    return std::to_string(value);
}