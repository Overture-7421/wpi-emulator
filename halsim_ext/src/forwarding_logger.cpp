#include "wpiemu/halsim_ext/forwarding_logger.h"

#include <cstdio>
#include <utility>

namespace wpiemu::halsim {

namespace {
void writeStderr(std::string_view line) {
    std::fwrite(line.data(), 1, line.size(), stderr);
    std::fputc('\n', stderr);
    std::fflush(stderr);
}
}  // namespace

ForwardingLogger::ForwardingLogger(sdk::LogLevel minLevel, ConsoleSink console)
    : minLevel_(minLevel), console_(console ? std::move(console) : ConsoleSink(&writeStderr)) {}

void ForwardingLogger::log(sdk::LogLevel level, std::string_view category, std::string_view message) {
    std::lock_guard lock(mutex_);
    if (level < minLevel_) return;
    if (console_) {
        std::string line = "[wpiemu] ";
        line += toString(level);
        line += ' ';
        line += category;
        line += ": ";
        line += message;
        console_(line);
    }
    if (remote_) {
        sdk::proto::Log out;
        out.simTimeUs = simTimeUs_;
        out.level = level;
        out.source = std::string(category);
        out.message = std::string(message);
        remote_(std::move(out));
    }
}

void ForwardingLogger::setMinLevel(sdk::LogLevel level) {
    std::lock_guard lock(mutex_);
    minLevel_ = level;
}

void ForwardingLogger::attachRemote(RemoteSink sink) {
    std::lock_guard lock(mutex_);
    remote_ = std::move(sink);
}

void ForwardingLogger::setSimTimeUs(sdk::SimTimeUs t) {
    std::lock_guard lock(mutex_);
    simTimeUs_ = t;
}

}  // namespace wpiemu::halsim
