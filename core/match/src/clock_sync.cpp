#include "wpiemu/core/match/clock_sync.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace wpiemu::core::match {

void ClockSync::reset() {
    samples_.clear();
    synced_ = false;
    anchorClientUs_ = 0;
    anchorServerUs_ = 0;
    drift_ = 0.0;
    minRttUs_ = 0;
    smoothedRttUs_ = 0;
    jitterUs_ = 0;
    pingCount_ = 0;
    goodCount_ = 0;
}

std::int64_t ClockSync::serverSimAt(std::int64_t clientUs) const {
    const std::int64_t dt = clientUs - anchorClientUs_;
    return anchorServerUs_ + dt + static_cast<std::int64_t>(std::llround(drift_ * static_cast<double>(dt)));
}

void ClockSync::seed(std::int64_t recvUs, std::int64_t serverSimUs, std::int64_t assumedDelayUs) {
    if (pingCount_ > 0) return;
    anchorClientUs_ = recvUs;
    anchorServerUs_ = serverSimUs + std::max<std::int64_t>(0, assumedDelayUs);
    drift_ = 0.0;
    synced_ = true;
}

void ClockSync::addPing(std::int64_t sendUs, std::int64_t recvUs, std::int64_t serverSimUs) {
    if (recvUs < sendUs) return;
    Sample s;
    s.rttUs = recvUs - sendUs;
    s.midUs = sendUs + s.rttUs / 2;
    s.offsetUs = serverSimUs - s.midUs;
    samples_.push_back(s);
    while (samples_.size() > config_.window) samples_.pop_front();
    ++pingCount_;
    smoothedRttUs_ = pingCount_ == 1 ? s.rttUs : smoothedRttUs_ + (s.rttUs - smoothedRttUs_) / 8;
    refit(recvUs);
}

void ClockSync::refit(std::int64_t nowUs) {
    std::vector<std::int64_t> rtts;
    rtts.reserve(samples_.size());
    for (const Sample& s : samples_) rtts.push_back(s.rttUs);
    std::sort(rtts.begin(), rtts.end());
    minRttUs_ = rtts.front();
    const std::size_t p90 = static_cast<std::size_t>(std::ceil(0.9 * static_cast<double>(rtts.size()))) - 1;
    jitterUs_ = (rtts[std::min(p90, rtts.size() - 1)] - minRttUs_) / 2;

    const std::int64_t threshold = minRttUs_ + std::max(config_.goodSlackUs, minRttUs_ / 4);
    std::vector<const Sample*> good;
    for (const Sample& s : samples_)
        if (s.rttUs <= threshold) good.push_back(&s);
    goodCount_ = good.size();

    double drift = drift_;
    double targetOffset = 0.0;
    const std::int64_t span = good.empty() ? 0 : good.back()->midUs - good.front()->midUs;
    if (good.size() >= config_.minDriftSamples && span >= config_.minDriftSpanUs) {
        double meanT = 0.0, meanO = 0.0;
        for (const Sample* s : good) {
            meanT += static_cast<double>(s->midUs);
            meanO += static_cast<double>(s->offsetUs);
        }
        meanT /= static_cast<double>(good.size());
        meanO /= static_cast<double>(good.size());
        double cov = 0.0, var = 0.0;
        for (const Sample* s : good) {
            const double dt = static_cast<double>(s->midUs) - meanT;
            cov += dt * (static_cast<double>(s->offsetUs) - meanO);
            var += dt * dt;
        }
        const double limit = config_.maxDriftPpm * 1e-6;
        drift = var > 0.0 ? std::clamp(cov / var, -limit, limit) : 0.0;
        targetOffset = meanO + drift * (static_cast<double>(nowUs) - meanT);
    } else {
        // Recent samples decide, so a change of the host's clock is followed within a handful of pings.
        std::vector<double> offsets;
        const std::size_t recent = std::min<std::size_t>(good.size(), 8);
        offsets.reserve(recent);
        for (std::size_t i = good.size() - recent; i < good.size(); ++i)
            offsets.push_back(static_cast<double>(good[i]->offsetUs) + drift * static_cast<double>(nowUs - good[i]->midUs));
        std::sort(offsets.begin(), offsets.end());
        targetOffset = offsets[offsets.size() / 2];
    }

    const std::int64_t target = nowUs + static_cast<std::int64_t>(std::llround(targetOffset));
    const std::int64_t current = synced_ ? serverSimAt(nowUs) : target;
    const std::int64_t diff = target - current;
    std::int64_t next = target;
    if (synced_ && pingCount_ > 1 && std::llabs(diff) <= config_.stepThresholdUs)
        next = current + static_cast<std::int64_t>(std::llround(config_.slewGain * static_cast<double>(diff)));
    anchorClientUs_ = nowUs;
    anchorServerUs_ = next;
    drift_ = drift;
    synced_ = true;
}

}  // namespace wpiemu::core::match
