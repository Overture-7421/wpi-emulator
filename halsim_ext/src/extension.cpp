// HALSIM extension entry point and composition root. Loaded into the robot process through
// HALSIM_EXTENSIONS (docs/reference/extension-ipc.md). All logic lives in testable classes (ExtensionRuntime, SimSession,
// BridgeSet, ...); this file only wires the concrete HAL / IPC / bridge implementations together.
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#if WPIEMU_HAL_ABI_SYSTEMCORE
#include "wpi/hal/HAL.h"
#include "wpi/hal/Extensions.h"
#else
#include "hal/HAL.h"
#include "hal/Extensions.h"
#endif
#include "ipc_bindings.h"
#include "wpiemu/halsim_ext/config.h"
#include "wpiemu/halsim_ext/extension_runtime.h"
#include "wpiemu/halsim_ext/hal_sim_platform.h"
#include "wpiemu/halsim_ext/os_process.h"
#include "wpiemu/sdk/devices/builtin_bridges.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

#if defined(_WIN32)
#define WPIEMU_EXPORT __declspec(dllexport)
#else
#define WPIEMU_EXPORT __attribute__((visibility("default")))
#endif

namespace {

using namespace wpiemu;

// WPIEMU_DISABLE_BRIDGES=ctre,hal.can : comma-separated bridge-name prefixes to leave out (debugging aid
// when one family misbehaves inside a particular robot process).
void dropDisabledBridges(std::vector<std::unique_ptr<sdk::IDeviceBridge>>& bridges) {
    const std::optional<std::string> spec = sdk::readEnvironment("WPIEMU_DISABLE_BRIDGES");
    if (!spec || spec->empty()) return;
    std::vector<std::string> prefixes;
    for (std::size_t start = 0; start <= spec->size();) {
        const std::size_t end = spec->find(',', start);
        const std::string item = spec->substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!item.empty()) prefixes.push_back(item);
        if (end == std::string::npos) break;
        start = end + 1;
    }
    std::erase_if(bridges, [&](const std::unique_ptr<sdk::IDeviceBridge>& b) {
        const std::string name(b->name());
        for (const std::string& p : prefixes) {
            if (name.rfind(p, 0) == 0) {
                std::fprintf(stderr, "[wpiemu] INFO extension: bridge '%s' disabled by WPIEMU_DISABLE_BRIDGES\n", name.c_str());
                return true;
            }
        }
        return false;
    });
}

// Everything the extension owns for the life of the process. It is intentionally never destroyed
// by static destruction (threads may still be running when the process exits); the HAL shutdown
// hook tears it down explicitly.
class ExtensionHost {
public:
    explicit ExtensionHost(halsim::ExtensionConfig config)
        : config_(std::move(config)), bindings_(halsim::makeIpcBindings()) {}

    sdk::Status start() {
        sdk::DeviceBridgeRegistry registry;
        if (const sdk::Status s = sdk::registerBuiltinBridges(registry); !s)
            std::fprintf(stderr, "[wpiemu] WARN extension: bridge registration problem: %s\n", s.error().message.c_str());
        std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
        if (registry.empty()) {
            std::fprintf(stderr, "[wpiemu] WARN extension: no device bridges are built in; every device will be ignored\n");
        } else {
            sdk::Result<std::vector<std::unique_ptr<sdk::IDeviceBridge>>> created = registry.createAll();
            if (!created) return created.error();
            bridges = std::move(created).value();
            dropDisabledBridges(bridges);
        }

        std::vector<std::unique_ptr<sdk::IVisionBridge>> visionBridges;
        sdk::VisionBridgeRegistry visionRegistry;
        if (const sdk::Status s = sdk::registerBuiltinVisionBridges(visionRegistry); !s)
            std::fprintf(stderr, "[wpiemu] WARN extension: vision bridge registration problem: %s\n", s.error().message.c_str());
        if (!visionRegistry.empty()) {
            sdk::Result<std::vector<std::unique_ptr<sdk::IVisionBridge>>> createdVision = visionRegistry.createAll();
            if (createdVision) visionBridges = std::move(createdVision).value();
            else std::fprintf(stderr, "[wpiemu] WARN extension: vision bridges unavailable: %s\n", createdVision.error().message.c_str());
        }

        halsim::RuntimeDeps deps{config_,
                                 halsim::HalSimPlatform::describeRobot(config_),
                                 halsim::HalPorts{platform_.clock(), platform_.driverStation(), platform_.power(), process_},
                                 *bindings_.client,
                                 bindings_.codec,
                                 std::move(bridges),
                                 {},
                                 std::move(visionBridges)};
        runtime_ = std::make_unique<halsim::ExtensionRuntime>(std::move(deps));
        return runtime_->start();
    }

    void shutdown() noexcept {
        if (runtime_) runtime_->stop();
    }

private:
    halsim::ExtensionConfig config_;
    halsim::HalSimPlatform platform_;
    halsim::OsProcessControl process_;
    halsim::IpcBindings bindings_;
    std::unique_ptr<halsim::ExtensionRuntime> runtime_;
};

ExtensionHost* g_host = nullptr;

void onHalShutdown(void*) {
    if (g_host != nullptr) g_host->shutdown();
}

}  // namespace

// Return 0 = loaded (also when the extension is inert), >0 = non-fatal failure (the HAL unloads this
// library; nothing of ours is left running), <0 would abort HAL initialisation (never used).
extern "C" WPIEMU_EXPORT int HALSIM_InitExtension(void) {
    try {
        if (g_host != nullptr) return 0;

        std::vector<std::string> warnings;
        halsim::ExtensionConfig config = halsim::parseExtensionConfig(&sdk::readEnvironment, &warnings);
        for (const std::string& w : warnings) std::fprintf(stderr, "[wpiemu] WARN extension: %s\n", w.c_str());
        if (config.disabled || config.endpoint.empty()) {
            std::fprintf(stderr, "[wpiemu] INFO extension: inert (%s); the robot runs normally\n",
                         config.disabled ? "WPIEMU_DISABLE is set" : "WPIEMU_IPC_ENDPOINT is not set");
            return 0;
        }

        auto host = std::make_unique<ExtensionHost>(std::move(config));
        if (const sdk::Status s = host->start(); !s) {
            std::fprintf(stderr, "[wpiemu] ERROR extension: could not attach to the emulator: %s\n", s.error().message.c_str());
            return 1;
        }
        g_host = host.release();
        HAL_OnShutdown(nullptr, &onHalShutdown);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[wpiemu] ERROR extension: initialisation failed: %s\n", e.what());
    } catch (...) {
        std::fprintf(stderr, "[wpiemu] ERROR extension: initialisation failed (unknown exception)\n");
    }
    return 1;
}
