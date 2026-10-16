#pragma once

#include <cstdint>
#include <string>

#include "wpiemu/halsim_ext/hal_ports.h"

namespace wpiemu::halsim {

// OS facts about the robot process (no HAL needed).
std::int64_t currentProcessId();
// File name (no directory) of the running executable, e.g. "java", "frcUserProgram".
std::string executableName();
// Java if a JVM or the WPILib JNI HAL is loaded into the process, else C++.
sdk::proto::RobotLanguage detectRobotLanguage();

class OsProcessControl final : public IProcessControl {
public:
    // Flushes stdio and ends the process immediately (no atexit handlers / static destructors: other
    // threads, e.g. a JVM, are still running).
    void terminate(int exitCode) override;
};

}  // namespace wpiemu::halsim
