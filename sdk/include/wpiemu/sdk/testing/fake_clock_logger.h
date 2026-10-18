#pragma once

#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::sdk::testing {

class FakeClock final : public ISimClock {
public:
    explicit FakeClock(SimTimeUs stepUs = 5000) : stepUs_(stepUs) {}
    SimTimeUs simTimeUs() const override { return timeUs_; }
    std::uint64_t tickIndex() const override { return tick_; }
    SimTimeUs fixedStepUs() const override { return stepUs_; }
    void advance() { timeUs_ += stepUs_; ++tick_; }
    void reset() { timeUs_ = 0; tick_ = 0; }

private:
    SimTimeUs stepUs_;
    SimTimeUs timeUs_ = 0;
    std::uint64_t tick_ = 0;
};

class CapturingLogger final : public ILogger {
public:
    struct Entry {
        LogLevel level;
        std::string category;
        std::string message;
    };

    void log(LogLevel level, std::string_view category, std::string_view message) override {
        std::lock_guard lock(mutex_);
        entries_.push_back({level, std::string(category), std::string(message)});
    }
    std::vector<Entry> entries() const {
        std::lock_guard lock(mutex_);
        return entries_;
    }
    std::size_t count(LogLevel level) const {
        std::lock_guard lock(mutex_);
        std::size_t n = 0;
        for (const auto& e : entries_) n += e.level == level ? 1u : 0u;
        return n;
    }

private:
    mutable std::mutex mutex_;
    std::vector<Entry> entries_;
};

}  // namespace wpiemu::sdk::testing
