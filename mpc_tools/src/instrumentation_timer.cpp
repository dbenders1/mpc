
#include <mpc_tools/instrumentation_timer.h>
#include <ros/package.h>

namespace Helpers {

Instrumentor::Instrumentor() : m_CurrentSession_(nullptr), m_ProfileCount_(0) {}

void Instrumentor::BeginSession(const std::string& name,
                                const std::string& node_name,
                                const std::string& filepath) {
  std::string full_filepath =
      ros::package::getPath("mpc_tools") + "/profiling/" + filepath;
  node_name_ = node_name;
  m_OutputStream_.open(full_filepath);
  WriteHeader();
  m_CurrentSession_ = new InstrumentationSession{name};
}

void Instrumentor::EndSession() {
  WriteFooter();
  m_OutputStream_.close();
  delete m_CurrentSession_;
  m_CurrentSession_ = nullptr;
  m_ProfileCount_ = 0;
}

void Instrumentor::WriteProfile(const ProfileResult& result) {
  std::lock_guard<std::mutex> lock(m_lock_);

  if (m_ProfileCount_++ > 0) m_OutputStream_ << ",\n";

  std::string name = result.Name;
  std::replace(name.begin(), name.end(), '"', '\'');

  m_OutputStream_ << "    {\n";
  m_OutputStream_ << "      \"cat\": \"function\",\n";
  m_OutputStream_ << "      \"dur\": " << (result.End - result.Start) << ",\n";
  m_OutputStream_ << "      \"name\": \"" << name + " (" + node_name_ + ")"
                  << "\",\n";
  m_OutputStream_ << "      \"ph\": \"X\",\n";
  m_OutputStream_ << "      \"pid\": 0,\n";
  m_OutputStream_ << "      \"tid\": " << result.ThreadID << ",\n";
  m_OutputStream_ << "      \"ts\": " << result.Start << "\n";
  m_OutputStream_ << "    }";

  m_OutputStream_.flush();
}

void Instrumentor::WriteHeader() {
  m_OutputStream_ << "{\n  \"otherData\": {},\n  \"traceEvents\": [\n";
  m_OutputStream_.flush();
}

void Instrumentor::WriteFooter() {
  m_OutputStream_ << "\n  ]\n}\n";
  m_OutputStream_.flush();
}

Instrumentor& Instrumentor::Get() {
  static Instrumentor* instance = new Instrumentor();
  return *instance;
}

InstrumentationTimer::InstrumentationTimer(const char* name)
    : m_Name(name), m_Stopped(false) {
  m_StartTimepoint = std::chrono::system_clock::now();
}

InstrumentationTimer::~InstrumentationTimer() {
  if (!m_Stopped) Stop();
}

void InstrumentationTimer::Stop() {
  auto endTimepoint = std::chrono::system_clock::now();

  long long start =
      std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint)
          .time_since_epoch()
          .count();
  long long end =
      std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint)
          .time_since_epoch()
          .count();

  uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
  Instrumentor::Get().WriteProfile({m_Name, start, end, threadID});

  m_Stopped = true;
}

};  // namespace Helpers