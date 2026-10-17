#include "wpiemu/halsim_ext/os_process.h"

#include <cstdlib>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#endif

namespace wpiemu::halsim {

std::int64_t currentProcessId() {
#if defined(_WIN32)
    return static_cast<std::int64_t>(::GetCurrentProcessId());
#else
    return static_cast<std::int64_t>(::getpid());
#endif
}

std::string executableName() {
    std::string path;
#if defined(_WIN32)
    char buffer[MAX_PATH * 4] = {};
    const DWORD n = ::GetModuleFileNameA(nullptr, buffer, static_cast<DWORD>(sizeof(buffer)));
    if (n > 0 && n < sizeof(buffer)) path.assign(buffer, n);
#elif defined(__APPLE__)
    char buffer[4096] = {};
    std::uint32_t size = sizeof(buffer);
    if (_NSGetExecutablePath(buffer, &size) == 0) path = buffer;
#else
    char buffer[4096] = {};
    const ssize_t n = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (n > 0) path.assign(buffer, static_cast<std::size_t>(n));
#endif
    const std::size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

sdk::proto::RobotLanguage detectRobotLanguage() {
#if defined(_WIN32)
    if (::GetModuleHandleA("jvm.dll") != nullptr || ::GetModuleHandleA("wpiHaljni.dll") != nullptr)
        return sdk::proto::RobotLanguage::Java;
#else
    // The java launcher dlopen()s libjvm with RTLD_GLOBAL, so its symbols are visible process-wide.
    if (::dlsym(RTLD_DEFAULT, "JNI_GetCreatedJavaVMs") != nullptr) return sdk::proto::RobotLanguage::Java;
#endif
    return sdk::proto::RobotLanguage::Cpp;
}

void OsProcessControl::terminate(int exitCode) {
    // Emergency watchdog termination must not wait for stdio locks or a backed-up pipe.
#if defined(_WIN32)
    ::TerminateProcess(::GetCurrentProcess(), static_cast<UINT>(exitCode));
#endif
    std::_Exit(exitCode);
}

}  // namespace wpiemu::halsim
