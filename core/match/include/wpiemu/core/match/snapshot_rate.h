#pragma once

// Chooses a client's snapshot rate (60 Hz LAN, 30 Hz WAN) from the transport's measured link health, with hysteresis so a
// link hovering at a threshold does not flap: it leaves the LAN rate only after `wanAfterSamples` consecutive bad samples
// and returns only after `lanAfterSamples` consecutive good ones (samples are taken at most every `sampleInterval` of the
// injected wall clock). A transport that cannot measure its link (no LinkStats) keeps the LAN rate.

#include <chrono>
#include <cstdint>
#include <optional>

#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::core::match {

struct SnapshotRateConfig {
    std::uint16_t lanHz = sdk::match::kSnapshotRateLanHz;
    std::uint16_t wanHz = sdk::match::kSnapshotRateWanHz;
    // A sample is "bad" at or above these; "good" at or below the lan* pair; in between it changes nothing.
    std::uint32_t wanRttMs = 60;
    std::uint32_t wanLossPermille = 30;
    std::uint32_t lanRttMs = 40;
    std::uint32_t lanLossPermille = 10;
    std::chrono::milliseconds sampleInterval{500};
    int wanAfterSamples = 2;
    int lanAfterSamples = 6;
};

class SnapshotRateSelector {
public:
    explicit SnapshotRateSelector(const SnapshotRateConfig& config = {}) : config_(config), hz_(config.lanHz) {}

    // Returns the rate in force after looking at `stats` (if a sample is due at `now`).
    bool sampleDue(std::chrono::steady_clock::time_point now) const { return !sampled_ || now - lastSample_ >= config_.sampleInterval; }

    std::uint16_t update(const std::optional<sdk::proto::LinkStats>& stats, std::chrono::steady_clock::time_point now) {
        if (!stats) return hz_;
        if (!sampleDue(now)) return hz_;
        sampled_ = true;
        lastSample_ = now;
        const bool bad = stats->rttMs >= config_.wanRttMs || stats->lossPermille >= config_.wanLossPermille;
        const bool good = stats->rttMs <= config_.lanRttMs && stats->lossPermille <= config_.lanLossPermille;
        if (hz_ == config_.lanHz && hz_ != config_.wanHz) {
            goodRun_ = 0;
            badRun_ = bad ? badRun_ + 1 : 0;
            if (badRun_ >= config_.wanAfterSamples) {
                hz_ = config_.wanHz;
                badRun_ = 0;
            }
        } else if (hz_ == config_.wanHz && hz_ != config_.lanHz) {
            badRun_ = 0;
            goodRun_ = good ? goodRun_ + 1 : 0;
            if (goodRun_ >= config_.lanAfterSamples) {
                hz_ = config_.lanHz;
                goodRun_ = 0;
            }
        }
        return hz_;
    }

    std::uint16_t hz() const { return hz_; }

private:
    SnapshotRateConfig config_;
    std::uint16_t hz_;
    bool sampled_ = false;
    std::chrono::steady_clock::time_point lastSample_{};
    int badRun_ = 0;
    int goodRun_ = 0;
};

}  // namespace wpiemu::core::match
