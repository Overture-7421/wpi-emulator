#pragma once

// Dynamic symbol lookup seam. Everything that talks to a native vendor/HAL library (CTRE sim ABI, wpiHal)
// does it through ISymbolResolver, so no link-time dependency on those libraries exists and unit tests
// can supply fake symbols. Absent symbols are tolerated: resolve() returns nullptr.

#include <memory>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::bridges::vendor {

class ISymbolResolver {
    WPIEMU_INTERFACE(ISymbolResolver)
public:
    virtual void* resolve(std::string_view symbol) const = 0;
};

enum class ModuleLoadMode {
    // Only attach to a module the process already loaded (GetModuleHandle / dlopen(RTLD_NOLOAD)). Required for
    // CTRE (docs/reference/bridges.md): we must talk to the very module instance the robot uses, never to a private copy.
    AttachOnly,
    // Load the module if it is absent. Intended for tools and tests, never for the robot process.
    LoadIfAbsent,
};

// RAII handle over a loaded native module. Cross-platform (LoadLibrary/dlopen family).
class ModuleSymbolResolver final : public ISymbolResolver {
public:
    // Returns nullptr (and fills `error` when given) if the module is not available.
    static std::unique_ptr<ModuleSymbolResolver> open(std::string_view moduleName, ModuleLoadMode mode,
                                                      std::string* error = nullptr);

    ~ModuleSymbolResolver() override;
    ModuleSymbolResolver(const ModuleSymbolResolver&) = delete;
    ModuleSymbolResolver& operator=(const ModuleSymbolResolver&) = delete;

    void* resolve(std::string_view symbol) const override;

private:
    ModuleSymbolResolver(void* handle, bool owned) : handle_(handle), owned_(owned) {}
    void* handle_;
    bool owned_;  // true when we took a reference that must be released
};

// Platform file names of the libraries we attach to.
std::string ctreSimModuleName();  // CTRE_PhoenixTools_Sim.dll / libCTRE_PhoenixTools_Sim.so / .dylib
std::string wpiHalModuleName();   // wpiHal.dll / libwpiHal.so / libwpiHal.dylib

}  // namespace wpiemu::bridges::vendor
