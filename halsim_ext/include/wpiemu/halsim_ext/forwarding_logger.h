#pragma once

#include <functional>
#include <mutex>
#include <string>
#include <string_view>

#include "wpiemu/halsim_ext/config.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::halsim {

// ILogger for everything inside the robot process: writes a line to the console sink (stderr by
// default; the launcher captures it) and, while a remote sink is attached, forwards a proto::Log to
// the emulator. Thread-safe; never blocks on I/O beyond the console write (the remote sink only
// enqueues), so it is usable from HAL callbacks too.
class ForwardingLogger final : public sdk::ILogger {
public:
    using ConsoleSink = std::function<void(std::string_view line)>;
    using RemoteSink = std::function<void(sdk::proto::Log&&)>;

    explicit ForwardingLogger(sdk::LogLevel minLevel = sdk::LogLevel::Info, ConsoleSink console = {});

    void log(sdk::LogLevel level, std::string_view category, std::string_view message) override;

    void setMinLevel(sdk::LogLevel level);
    // Empty function detaches.
    void attachRemote(RemoteSink sink);
    // Stamps forwarded logs with the latest sim time.
    void setSimTimeUs(sdk::SimTimeUs t);

private:
    std::mutex mutex_;
    sdk::LogLevel minLevel_;
    ConsoleSink console_;
    RemoteSink remote_;
    sdk::SimTimeUs simTimeUs_ = 0;
};

}  // namespace wpiemu::halsim
