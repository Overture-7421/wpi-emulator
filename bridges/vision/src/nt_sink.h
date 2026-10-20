#pragma once

// Shared plumbing of the two vision bridges (internal).

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "wpiemu/bridges/vision/nt_backend.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::bridges::vision::detail {

// Publishes through the backend, logging the first failure of the bridge only (a failing topic would otherwise
// flood the log every tick).
class NtSink {
public:
    NtSink(std::shared_ptr<INtBackend> backend, std::string category)
        : backend_(std::move(backend)), category_(std::move(category)) {}

    void setLogger(sdk::ILogger* logger) { logger_ = logger != nullptr ? logger : &null_; }
    INtBackend& backend() { return *backend_; }
    void error(const std::string& message) { logger_->error(category_, message); }
    void warn(const std::string& message) { logger_->warn(category_, message); }
    bool available() { return backend_ && backend_->available(); }

    void put(std::string_view path, const NtValue& value, std::int64_t timestampUs, std::string_view typeString = {}) {
        const sdk::Status s = backend_->publish(path, value, timestampUs, typeString);
        if (!s.ok() && !failureLogged_) {
            failureLogged_ = true;
            logger_->warn(category_, "publish failed for " + std::string(path) + ": " + s.error().message);
        }
    }
    void putDouble(std::string_view path, double v, std::int64_t ts) { put(path, ntDouble(v), ts); }
    void putInt(std::string_view path, std::int64_t v, std::int64_t ts) { put(path, ntInt(v), ts); }
    void putBool(std::string_view path, bool v, std::int64_t ts) { put(path, ntBool(v), ts); }
    void putString(std::string_view path, std::string v, std::int64_t ts) { put(path, ntString(std::move(v)), ts); }
    void putDoubles(std::string_view path, NtDoubles v, std::int64_t ts) { put(path, ntDoubles(std::move(v)), ts); }

    std::optional<NtValue> read(std::string_view path) { return backend_->read(path); }

private:
    std::shared_ptr<INtBackend> backend_;
    std::string category_;
    sdk::NullLogger null_;
    sdk::ILogger* logger_ = &null_;
    bool failureLogged_ = false;
};

}  // namespace wpiemu::bridges::vision::detail
