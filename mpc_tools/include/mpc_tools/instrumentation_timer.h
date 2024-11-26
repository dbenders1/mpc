#ifndef MPC_TOOLS_INSTRUMENTATION_TIMER_H
#define MPC_TOOLS_INSTRUMENTATION_TIMER_H

#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <fstream>
#include <ros/package.h>


/********** Some Fancy timing classes for profiling (from TheCherno) ***********/
#define PROFILER 1
#if PROFILER
#define PROFILE_SCOPE(name) Helpers::InstrumentationTimer timer##__LINE__(name);
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#define PROFILE_AND_LOG_INFO(name) MPC_INFO(name) Helpers::InstrumentationTimer timer_info##__LINE__(name);
#define PROFILE_AND_LOG_WARN(name) MPC_WARN(name) Helpers::InstrumentationTimer timer_warn##__LINE__(name);
#define PROFILE_AND_LOG_ERROR(name) MPC_ERROR(name) Helpers::InstrumentationTimer timer_error##__LINE__(name);
#else
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif


namespace Helpers { 

struct ProfileResult
{
    std::string Name;
    long long Start, End;
    uint32_t ThreadID;
};

struct InstrumentationSession
{
    std::string Name;
};

class Instrumentor
{
private:
    InstrumentationSession* m_CurrentSession;
    std::ofstream m_OutputStream;
    int m_ProfileCount;
    std::mutex m_lock;
    std::string node_name;

public:
    Instrumentor();

    void BeginSession(const std::string &name, const std::string &ros_node_name, const std::string &filepath = "mpc_profiler.json");

    void EndSession();

    void WriteProfile(const ProfileResult& result);

    void WriteHeader();

    void WriteFooter();

    static Instrumentor& Get();
};

class InstrumentationTimer
{
public:
    InstrumentationTimer(const char* name);

    ~InstrumentationTimer();

    void Stop();

private:
    const char* m_Name;
    std::chrono::system_clock::time_point m_StartTimepoint;
    bool m_Stopped;
};

};  // namespace Helpers

#endif // MPC_TOOLS_INSTRUMENTATION_TIMER_H
