#include "wpiemu/bridges/vendor/symbol_resolver.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace wpiemu::bridges::vendor {

std::unique_ptr<ModuleSymbolResolver> ModuleSymbolResolver::open(std::string_view moduleName, ModuleLoadMode mode,
                                                                 std::string* error) {
    const std::string name(moduleName);
#if defined(_WIN32)
    HMODULE module = GetModuleHandleA(name.c_str());  // no reference taken
    bool owned = false;
    if (module == nullptr && mode == ModuleLoadMode::LoadIfAbsent) {
        module = LoadLibraryA(name.c_str());
        owned = module != nullptr;
    }
    if (module == nullptr) {
        if (error != nullptr) *error = "module not loaded: " + name;
        return nullptr;
    }
    return std::unique_ptr<ModuleSymbolResolver>(new ModuleSymbolResolver(reinterpret_cast<void*>(module), owned));
#else
    void* handle = dlopen(name.c_str(), RTLD_NOLOAD | RTLD_LAZY);  // takes a reference only if already loaded
    if (handle == nullptr && mode == ModuleLoadMode::LoadIfAbsent) handle = dlopen(name.c_str(), RTLD_LAZY | RTLD_LOCAL);
    if (handle == nullptr) {
        if (error != nullptr) {
            const char* why = dlerror();
            *error = "module not loaded: " + name + (why != nullptr ? std::string(" (") + why + ")" : std::string{});
        }
        return nullptr;
    }
    return std::unique_ptr<ModuleSymbolResolver>(new ModuleSymbolResolver(handle, true));
#endif
}

ModuleSymbolResolver::~ModuleSymbolResolver() {
    if (handle_ == nullptr || !owned_) return;
#if defined(_WIN32)
    FreeLibrary(reinterpret_cast<HMODULE>(handle_));
#else
    dlclose(handle_);
#endif
}

void* ModuleSymbolResolver::resolve(std::string_view symbol) const {
    if (handle_ == nullptr) return nullptr;
    const std::string name(symbol);
#if defined(_WIN32)
    return reinterpret_cast<void*>(GetProcAddress(reinterpret_cast<HMODULE>(handle_), name.c_str()));
#else
    return dlsym(handle_, name.c_str());
#endif
}

std::string ctreSimModuleName() {
#if defined(_WIN32)
    return "CTRE_PhoenixTools_Sim.dll";
#elif defined(__APPLE__)
    return "libCTRE_PhoenixTools_Sim.dylib";
#else
    return "libCTRE_PhoenixTools_Sim.so";
#endif
}

std::string wpiHalModuleName() {
#if defined(_WIN32)
    return "wpiHal.dll";
#elif defined(__APPLE__)
    return "libwpiHal.dylib";
#else
    return "libwpiHal.so";
#endif
}

}  // namespace wpiemu::bridges::vendor
