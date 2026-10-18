#include "nt_binder.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstddef>
#include <iterator>
#elif defined(__unix__) || defined(__APPLE__)
#include <dlfcn.h>
#endif

namespace wpiemu::bridges::vision::binding {

namespace {

BindOutcome notLoaded(const char* module) {
    BindOutcome outcome;
    outcome.kind = BindOutcome::Kind::ModuleNotLoaded;
    outcome.module = module;
    return outcome;
}

}  // namespace

#if defined(_WIN32)

// The Java robot maps ntcore.dll a few milliseconds AFTER HALSIM_InitExtension returns, a C++ robot before (verified
// by the nt spike, WPILib 2026.2.1, Windows x64). In both the module base name is "ntcore.dll", once per process.
BindOutcome bindProcessModule(NtFunctions& out, const char* moduleName) {
    // The names are ASCII literals: widen them on the stack (no allocation, no ANSI conversion on every retry).
    wchar_t wideName[64] = {};
    for (std::size_t i = 0; moduleName[i] != '\0' && i + 1 < std::size(wideName); ++i)
        wideName[i] = static_cast<wchar_t>(static_cast<unsigned char>(moduleName[i]));
    const HMODULE module = GetModuleHandleW(wideName);  // takes no reference and never loads
    if (module == nullptr) return notLoaded(moduleName);
    return bindSymbols(out, moduleName,
                       [module](const char* name) { return reinterpret_cast<void*>(GetProcAddress(module, name)); });
}

#elif defined(__unix__) || defined(__APPLE__)

// UNVERIFIED (no Linux or macOS run so far): the library names are the ones WPILib ships for desktop simulation.
BindOutcome bindProcessModule(NtFunctions& out, const char* moduleName) {
    void* handle = dlopen(moduleName, RTLD_NOLOAD | RTLD_LAZY);  // a reference only when the robot already loaded it
    if (handle == nullptr) return notLoaded(moduleName);
    const BindOutcome outcome =
        bindSymbols(out, moduleName, [handle](const char* name) { return dlsym(handle, name); });
    // The reference of a successful binding is kept on purpose: ntcore may be torn down at any time while the robot
    // exits and the handles we publish with must never dangle.
    if (!outcome.bound()) dlclose(handle);
    return outcome;
}

#else

BindOutcome bindProcessModule(NtFunctions&, const char*) { return BindOutcome{}; }

#endif

}  // namespace wpiemu::bridges::vision::binding
