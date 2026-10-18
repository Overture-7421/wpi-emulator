#pragma once

// Filling an NtFunctions table (internal to src/binding): from a symbol lookup (any source, so tests can fake one) and
// from the ntcore module the robot process has loaded.

#include <cstdint>
#include <type_traits>

#include "nt_functions.h"

namespace wpiemu::bridges::vision::binding {

// The result of one binding attempt. The strings are always string literals (static storage), so building an outcome
// never allocates and a failed attempt can be repeated on every call.
struct BindOutcome {
    enum class Kind : std::uint8_t {
        Bound,
        ModuleNotLoaded,  // the robot process has not (yet) mapped the ntcore module
        SymbolMissing,    // the module lacks an entry point: a different WPILib than the one this build was pinned to
        Unsupported,      // no way to find a loaded module on this platform
    };
    Kind kind = Kind::Unsupported;
    const char* module = "";  // file name of the ntcore module
    const char* symbol = "";  // SymbolMissing: the first symbol that could not be resolved

    bool bound() const { return kind == Kind::Bound; }
};

// Resolves every entry of the table with `lookup(const char* symbolName) -> void*` (nullptr when absent). `out` is
// only written when ALL symbols resolve, so a half-bound table never exists.
template <class Lookup>
BindOutcome bindSymbols(NtFunctions& out, const char* module, Lookup&& lookup) {
    NtFunctions found;
    const char* missing = nullptr;
    found.visit([&](const char* name, auto& entry) {
        if (missing != nullptr) return;
        entry = reinterpret_cast<std::remove_reference_t<decltype(entry)>>(lookup(name));
        if (entry == nullptr) missing = name;
    });
    BindOutcome outcome;
    outcome.module = module;
    if (missing != nullptr) {
        outcome.kind = BindOutcome::Kind::SymbolMissing;
        outcome.symbol = missing;
        return outcome;
    }
    out = found;
    outcome.kind = BindOutcome::Kind::Bound;
    return outcome;
}

// Attaches to the module `moduleName` (a string literal) that this process has ALREADY loaded: GetModuleHandle on
// Windows, dlopen(RTLD_NOLOAD) elsewhere (Linux and macOS are written to the same pattern as the other bridges but
// UNVERIFIED). It never loads a copy: a private ntcore would be a different NetworkTables instance, invisible to the
// robot. Cheap and allocation free when the module is not there, so it may be retried on every call.
BindOutcome bindProcessModule(NtFunctions& out, const char* moduleName);

// The ntcore module of the robot process: ntcore.dll / libntcore.so / libntcore.dylib (WPILib's desktop libraries).
#if defined(_WIN32)
inline constexpr const char* kNtcoreModuleName = "ntcore.dll";
#elif defined(__APPLE__)
inline constexpr const char* kNtcoreModuleName = "libntcore.dylib";
#else
inline constexpr const char* kNtcoreModuleName = "libntcore.so";
#endif

inline BindOutcome bindProcessNtcore(NtFunctions& out) { return bindProcessModule(out, kNtcoreModuleName); }

// True when this build was compiled against the staged ntcore C headers, i.e. every declaration of the binding was
// proven identical to the real ABI (nt_abi_check.cpp). Without it the real binding must stay inert.
bool abiVerifiedAgainstHeaders();

}  // namespace wpiemu::bridges::vision::binding
