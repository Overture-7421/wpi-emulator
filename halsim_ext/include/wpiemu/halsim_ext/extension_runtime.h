#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/halsim_ext/bridge_set.h"
#include "wpiemu/halsim_ext/config.h"
#include "wpiemu/halsim_ext/forwarding_logger.h"
#include "wpiemu/halsim_ext/hal_ports.h"
#include "wpiemu/halsim_ext/runtime_stats.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::halsim {

// Everything the extension needs, injected by the composition root (HALSIM_InitExtension) or a test.
// References must outlive the runtime.
struct RuntimeDeps {
    ExtensionConfig config;
    RobotInfo robot;
    HalPorts ports;
    sdk::proto::ITransportClient& client;
    std::shared_ptr<const sdk::proto::IMessageCodec> codec;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    // Console sink for the extension's log lines (default: stderr). Tests capture here.
    ForwardingLogger::ConsoleSink console;
    // Simulated vision cameras (Limelight / PhotonVision). Optional; empty = no cameras are published.
    std::vector<std::unique_ptr<sdk::IVisionBridge>> visionBridges;
};

// The extension's lifetime manager: connect + Hello/HelloAck, pause HAL time, start the bridges,
// run one SimSession at a time on the clock thread, and reconnect when the link drops.
class ExtensionRuntime {
public:
    explicit ExtensionRuntime(RuntimeDeps deps);
    ~ExtensionRuntime();
    ExtensionRuntime(const ExtensionRuntime&) = delete;
    ExtensionRuntime& operator=(const ExtensionRuntime&) = delete;

    // Synchronous: connects and completes the handshake (bounded by the configured timeouts); on
    // success HAL time is paused (lock-step), the bridges are started and the clock thread runs.
    // On failure nothing is left running and HAL timing is untouched.
    sdk::Status start();

    // Idempotent: resumes HAL timing, says Goodbye, joins the clock thread (bounded wait).
    void stop() noexcept;

    // The clock thread has exited (Stop command, reconnect disabled/exhausted, or stop()).
    bool finished() const;
    bool waitFinished(std::chrono::milliseconds timeout) const;

    RuntimeStats stats() const;
    sdk::ILogger& logger();

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

// Builds the Hello from `robot` + config and runs the handshake on `channel` (exposed for tests).
sdk::Result<sdk::proto::HelloAck> performHandshake(sdk::proto::MessageChannel& channel, const RobotInfo& robot,
                                                   const ExtensionConfig& config, sdk::ILogger& logger);

}  // namespace wpiemu::halsim
