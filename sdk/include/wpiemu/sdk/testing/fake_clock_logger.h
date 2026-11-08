#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::sdk::testing {

class FakeClock final : public ISimClock {
public:
    explicit FakeClock(SimTimeUs fixedStepUs = kDefaultStepUs) : fixedStepUs_(fixedStepUs) {}

    SimTimeUs simTimeUs() const override { return simTimeUs_; }
    std::uint64_t tickIndex() const override { return tickIndex_; }
    SimTimeUs fixedStepUs() const override { return fixedStepUs_; }

    void advance() {
        simTimeUs_ += fixedStepUs_;
        ++tickIndex_;
    }
    void reset() {
        simTimeUs_ = 0;
        tickIndex_ = 0;
    }

private:
    SimTimeUs fixedStepUs_;
    SimTimeUs simTimeUs_ = 0;
    std::uint64_t tickIndex_ = 0;
};

// A wall clock that only moves when a test advances it, so watchdogs, pacing and network impairment are tested without
// sleeping. Starts away from the epoch so "a moment ago" is representable. Safe to advance from any thread.
class ManualWallClock final : public IWallClock {
public:
    std::chrono::steady_clock::time_point now() const override {
        return std::chrono::steady_clock::time_point{} + std::chrono::steady_clock::duration(ticks_.load());
    }
    void advance(std::chrono::steady_clock::duration d) { ticks_ += d.count(); }

private:
    std::atomic<std::chrono::steady_clock::rep> ticks_{
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::seconds(10)).count()};
};

struct LogEntry {
    LogLevel level = LogLevel::Info;
    std::string category;
    std::string source;
    std::string message;
};

class CapturingLogger final : public ILogger {
public:
    void log(LogLevel level, std::string_view category, std::string_view message) override {
        std::lock_guard lock(mutex_);
        entries_.push_back({level, std::string(category), std::string(category), std::string(message)});
    }

    std::size_t count(LogLevel level) const {
        std::lock_guard lock(mutex_);
        std::size_t total = 0;
        for (const LogEntry& entry : entries_)
            if (entry.level == level) ++total;
        return total;
    }

    std::vector<LogEntry> entries() const {
        std::lock_guard lock(mutex_);
        return entries_;
    }

    void clear() {
        std::lock_guard lock(mutex_);
        entries_.clear();
    }

private:
    mutable std::mutex mutex_;
    std::vector<LogEntry> entries_;
};

}  // namespace wpiemu::sdk::testing
