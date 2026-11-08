#include <cstring>
#include <iostream>

#include "client/client_module.h"
#include "wpiemu/net/module.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/version.h"

#ifndef WPIEMU_APP_VERSION
#define WPIEMU_APP_VERSION "0.0.0"
#endif

namespace {

int printVersion() {
    wpiemu::sdk::GamePluginRegistry plugins;
    if (const auto status = wpiemu::sdk::registerBuiltinGamePlugins(plugins); !status.ok()) {
        std::cerr << "plugin registration failed: " << status.error().message << '\n';
        return 1;
    }
    std::cout << wpiemu::app::client::moduleName() << ' ' << WPIEMU_APP_VERSION << " (sdk " << wpiemu::sdk::kSdkVersionMajor
              << '.' << wpiemu::sdk::kSdkVersionMinor << ", " << wpiemu::net::moduleName() << ", wpiemu_core, "
              << plugins.size() << " game plugin(s))\n";
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    // Headless smoke path used by CTest; avoids creating any window.
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--version") == 0) return printVersion();

    std::cerr << "wpiemu_client: joining a match is not implemented yet; try --version\n";
    return 2;
}
