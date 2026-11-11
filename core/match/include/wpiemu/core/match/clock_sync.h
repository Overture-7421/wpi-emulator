#pragma once

// Clock synchronisation of a game client with the match host: maps the client's monotonic microseconds to the host's
// simulation time (and so to a host tick). The host's sim time only advances while the match runs, so the client resets the
// estimate whenever the host stops or restarts the world and re-measures.
//
// Samples are the PingRequest / PingReply exchanges (sdk/match/match_control.h): the host answers with its sim time at that
// instant, the client knows when it sent and received. Assuming the two directions take equally long, the host's clock read
// serverSim at the MIDPOINT of the exchange, so   offset = serverSim - (send + recv) / 2   (the asymmetry of a real path is
// not measurable from round trips; it biases the estimate by half the asymmetry and is absorbed by the input-lead feedback,
// see own_predictor / match_client).
//
// Filtering. The samples that were delayed by queueing are the noisy ones: only samples whose round trip is within a slack of
// the minimum round trip of the window count ("minimum-RTT filter"). From those the model is
//     serverSim(t) = t + offset + drift * (t - anchor)
// the offset being the median of the good samples and, once there are enough good samples over a long enough span, the drift
// (a client clock running slightly fast or slow against the host's) a least-squares slope, clamped to +-maxDriftPpm. A new
// estimate moves the model by `slewGain` of the difference at a time, unless the difference exceeds `stepThresholdUs` (then
// the model jumps), so the clock the client derives its prediction and render timeline from does not jitter.

#include <cstddef>
#include <cstdint>
#include <deque>

namespace wpiemu::core::match {

struct ClockSyncConfig {
    std::size_t window = 32;                    // newest samples kept
    std::int64_t goodSlackUs = 3000;            // a sample is good when rtt <= minRtt + max(goodSlackUs, minRtt / 4)
    std::size_t minDriftSamples = 8;            // good samples needed before a drift is fitted
    std::int64_t minDriftSpanUs = 4'000'000;    // and the span they must cover
    double maxDriftPpm = 1000.0;
    std::int64_t stepThresholdUs = 40'000;      // an offset change above this is a jump, below it is slewed
    double slewGain = 0.25;
};

class ClockSync {
public:
    explicit ClockSync(ClockSyncConfig config = {}) : config_(config) {}

    // Forgets everything (the host's world stopped, restarted or the session changed).
    void reset();

    // One ping exchange: sent at `sendUs`, answered with the host sim time `serverSimUs`, received at `recvUs` (client
    // microseconds). Ignored when recvUs < sendUs.
    void addPing(std::int64_t sendUs, std::int64_t recvUs, std::int64_t serverSimUs);
    // A one-way observation that seeds an UNSYNCED clock: the host sim time `serverSimUs` was current `assumedDelayUs` before
    // `recvUs`. Pings replace it.
    void seed(std::int64_t recvUs, std::int64_t serverSimUs, std::int64_t assumedDelayUs);

    bool synced() const { return synced_; }
    bool hasPings() const { return pingCount_ > 0; }

    // The host sim time at client time `clientUs` (extrapolated; negative before the world started).
    std::int64_t serverSimAt(std::int64_t clientUs) const;

    std::int64_t minRttUs() const { return minRttUs_; }
    std::int64_t smoothedRttUs() const { return smoothedRttUs_; }
    // How much worse than the minimum the slow round trips of the window are, one way: (p90 rtt - min rtt) / 2.
    std::int64_t jitterUs() const { return jitterUs_; }
    double driftPpm() const { return drift_ * 1e6; }
    std::size_t pingCount() const { return pingCount_; }
    std::size_t goodSamples() const { return goodCount_; }
    // Worst-case error of the model against the host's real clock from the path asymmetry alone: half the minimum RTT.
    std::int64_t asymmetryBoundUs() const { return minRttUs_ / 2; }

private:
    struct Sample {
        std::int64_t midUs = 0;     // client time of the exchange midpoint
        std::int64_t rttUs = 0;
        std::int64_t offsetUs = 0;  // serverSim - midUs
    };

    void refit(std::int64_t nowUs);

    ClockSyncConfig config_;
    std::deque<Sample> samples_;
    bool synced_ = false;
    std::int64_t anchorClientUs_ = 0;
    std::int64_t anchorServerUs_ = 0;   // model value at anchorClientUs_
    double drift_ = 0.0;                // rate error: serverSim advances (1 + drift_) per client microsecond
    std::int64_t minRttUs_ = 0;
    std::int64_t smoothedRttUs_ = 0;
    std::int64_t jitterUs_ = 0;
    std::size_t pingCount_ = 0;
    std::size_t goodCount_ = 0;
};

}  // namespace wpiemu::core::match
