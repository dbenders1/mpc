
#include <mpc_tools/instrumentation_timer.h>

#include <ros/package.h>


namespace Helpers {

Instrumentor::Instrumentor()
    : m_CurrentSession(nullptr), m_ProfileCount(0)
{
}

void Instrumentor::BeginSession(const std::string &name, const std::string &ros_node_name, const std::string &filepath)
{
    std::string full_filepath = ros::package::getPath("mpc_tools") + "/profiling/" + filepath;
    node_name = ros_node_name;
    m_OutputStream.open(full_filepath);
    WriteHeader();
    m_CurrentSession = new InstrumentationSession{name};
}

void Instrumentor::EndSession()
{
    WriteFooter();
    m_OutputStream.close();
    delete m_CurrentSession;
    m_CurrentSession = nullptr;
    m_ProfileCount = 0;
}

void Instrumentor::WriteProfile(const ProfileResult& result)
{
    std::lock_guard<std::mutex> lock(m_lock);

    if (m_ProfileCount++ > 0)
        m_OutputStream << ",\n";

    std::string name = result.Name;
    std::replace(name.begin(), name.end(), '"', '\'');

    m_OutputStream << "    {\n";
    m_OutputStream << "      \"cat\":\"function\",\n";
    m_OutputStream << "      \"dur\":" << (result.End - result.Start) << ",\n";
    m_OutputStream << "      \"name\":\"" << name + " (" + node_name + ")" << "\",\n";
    m_OutputStream << "      \"ph\":\"X\",\n";
    m_OutputStream << "      \"pid\":0,\n";
    m_OutputStream << "      \"tid\":" << result.ThreadID << ",\n";
    m_OutputStream << "      \"ts\":" << result.Start << "\n";
    m_OutputStream << "    }";

    m_OutputStream.flush();
}

void Instrumentor::WriteHeader()
{
    m_OutputStream << "{\n  \"otherData\": {},\n  \"traceEvents\": [\n";
    m_OutputStream.flush();
}

void Instrumentor::WriteFooter()
{
    m_OutputStream << "\n  ]\n}\n";
    m_OutputStream.flush();
}

Instrumentor& Instrumentor::Get()
{
    static Instrumentor* instance = new Instrumentor();
    return *instance;
}


InstrumentationTimer::InstrumentationTimer(const char* name) : m_Name(name), m_Stopped(false)
{
    m_StartTimepoint = std::chrono::system_clock::now();
}

InstrumentationTimer::~InstrumentationTimer()
{
    if (!m_Stopped)
        Stop();
}

void InstrumentationTimer::Stop()
{
    auto endTimepoint = std::chrono::system_clock::now();

    long long start =
        std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
    long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

    uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
    Instrumentor::Get().WriteProfile({ m_Name, start, end, threadID });

    m_Stopped = true;
}

};  // namespace Helpers