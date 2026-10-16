#include "wpiemu/bridges/vendor/guarded_backend.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::bridges::vendor {

namespace {
constexpr const char* kCategory = "bridge.vendor.guard";
}

GuardedSimDeviceBackend::GuardedSimDeviceBackend(sdk::ISimDeviceBackend& inner, GuardOptions options)
    : inner_(&inner), options_(std::move(options)) {
    guards_.reserve(options_.guarded.size());
    for (const auto& spec : options_.guarded) {
        Guard g;
        g.spec = spec;
        try {
            g.deviceRe = std::regex(spec.deviceNameRegex, std::regex::ECMAScript);
        } catch (const std::regex_error&) {
            g.valid = false;  // an invalid guard simply never matches; rule tables are unit-tested
        }
        guards_.push_back(std::move(g));
    }
}

void GuardedSimDeviceBackend::setLogger(sdk::ILogger* logger) {
    std::lock_guard lock(mutex_);
    logger_ = logger;
}

void GuardedSimDeviceBackend::listDevices(std::vector<sdk::SimDeviceInfo>& out) const {
    const std::size_t first = out.size();
    inner_->listDevices(out);
    std::lock_guard lock(mutex_);
    for (std::size_t i = first; i < out.size(); ++i) deviceNames_[out[i].handle] = out[i].name;
}

void GuardedSimDeviceBackend::listValues(sdk::SimDeviceHandle device, std::vector<sdk::SimValueInfo>& out) const {
    const std::size_t first = out.size();
    inner_->listValues(device, out);
    std::lock_guard lock(mutex_);
    const auto nameIt = deviceNames_.find(device);
    const std::string deviceName = nameIt == deviceNames_.end() ? std::string{} : nameIt->second;
    for (std::size_t i = first; i < out.size(); ++i) {
        Meta meta{deviceName, out[i].name, -1};
        for (std::size_t g = 0; g < guards_.size(); ++g) {
            const Guard& guard = guards_[g];
            if (guard.valid && guard.spec.valueName == out[i].name && std::regex_match(deviceName, guard.deviceRe)) {
                meta.guardIndex = static_cast<int>(g);
                break;
            }
        }
        meta_[out[i].handle] = std::move(meta);
    }
}

bool GuardedSimDeviceBackend::close(double a, double b) const {
    return std::abs(a - b) <= options_.absTolerance + options_.relTolerance * std::max(std::abs(a), std::abs(b));
}

double GuardedSimDeviceBackend::unwrapped(State& s, const Guard& g, double incoming) const {
    if (!g.spec.unwrapDegrees) return incoming;
    if (!s.hasIncoming) {
        s.continuous = incoming;
    } else {
        double d = incoming - s.lastIncoming;
        d -= 360.0 * std::round(d / 360.0);
        s.continuous += d;
    }
    s.hasIncoming = true;
    s.lastIncoming = incoming;
    return s.continuous;
}

void GuardedSimDeviceBackend::report(const Meta& meta, const State& s) {
    if (logger_ == nullptr) return;
    logger_->warn(kCategory, "'" + meta.device + "' input '" + meta.value + "' keeps being changed by someone else (" +
                                 std::to_string(s.mismatches) +
                                 " mismatches; last wrote " + std::to_string(s.lastWritten) + ", read back " +
                                 std::to_string(s.lastReadback) +
                                 "). Another writer is active (robot-side simulation / SimState code, or the vendor "
                                 "mirror). Guarded inputs are re-asserted every tick (last-writer-wins).");
}

bool GuardedSimDeviceBackend::verifyReadback(sdk::SimValueHandle h, const Meta& meta, State& s) {
    if (!s.hasWritten) return false;
    const double readback = inner_->getValue(h);
    s.lastReadback = readback;
    if (close(readback, s.lastWritten)) {
        if (!s.reasserted) s.streak = 0;
        s.reasserted = false;
        return false;
    }
    noteMismatch(meta, s);
    s.reasserted = false;
    return true;
}

void GuardedSimDeviceBackend::noteMismatch(const Meta& meta, State& s) {
    ++s.mismatches;
    ++s.streak;
    if (!s.foreign && s.streak >= options_.foreignWriteThreshold) {
        s.foreign = true;
        report(meta, s);
    }
}

void GuardedSimDeviceBackend::commit(sdk::SimValueHandle h, const Meta& meta, State& s, double target, bool force) {
    const bool disturbed = verifyReadback(h, meta, s);
    if (force || disturbed || !s.hasWritten || !close(target, s.lastWritten)) {
        inner_->setValue(h, target);
        s.lastWritten = target;
        s.hasWritten = true;
    }
}

void GuardedSimDeviceBackend::setValue(sdk::SimValueHandle value, double v) {
    std::lock_guard lock(mutex_);
    const auto metaIt = meta_.find(value);
    if (metaIt == meta_.end()) {
        inner_->setValue(value, v);
        return;
    }
    const Meta& meta = metaIt->second;
    State& s = state_[value];
    if (meta.guardIndex < 0) {
        commit(value, meta, s, v, /*force=*/true);
        return;
    }
    const double target = unwrapped(s, guards_[static_cast<std::size_t>(meta.guardIndex)], v);
    if (inBatch_) {
        if (!s.pending) pending_.push_back(value);
        s.pending = true;
        s.pendingValue = target;
        return;
    }
    commit(value, meta, s, target, /*force=*/false);
}

void GuardedSimDeviceBackend::beginWrites() {
    std::lock_guard lock(mutex_);
    inBatch_ = true;
}

void GuardedSimDeviceBackend::endWrites() {
    std::lock_guard lock(mutex_);
    inBatch_ = false;
    for (const sdk::SimValueHandle h : pending_) {
        const auto metaIt = meta_.find(h);
        const auto stateIt = state_.find(h);
        if (metaIt == meta_.end() || stateIt == state_.end() || !stateIt->second.pending) continue;
        State& s = stateIt->second;
        s.pending = false;
        commit(h, metaIt->second, s, s.pendingValue, /*force=*/false);
    }
    pending_.clear();
}

std::size_t GuardedSimDeviceBackend::reassertAfterMirror() {
    std::lock_guard lock(mutex_);
    std::size_t restored = 0;
    for (auto& [h, s] : state_) {
        if (!s.hasWritten) continue;
        const auto metaIt = meta_.find(h);
        if (metaIt == meta_.end() || metaIt->second.guardIndex < 0) continue;
        const double readback = inner_->getValue(h);
        if (close(readback, s.lastWritten)) continue;
        s.lastReadback = readback;
        ++s.reasserts;
        s.reasserted = true;
        noteMismatch(metaIt->second, s);
        inner_->setValue(h, s.lastWritten);
        ++restored;
    }
    return restored;
}

void GuardedSimDeviceBackend::resetHistory() {
    std::lock_guard lock(mutex_);
    state_.clear();
    pending_.clear();
}

std::vector<InputDisturbance> GuardedSimDeviceBackend::disturbances() const {
    std::lock_guard lock(mutex_);
    std::vector<InputDisturbance> out;
    for (const auto& [h, s] : state_) {
        if (s.mismatches == 0 && !s.foreign) continue;
        const auto metaIt = meta_.find(h);
        InputDisturbance d;
        if (metaIt != meta_.end()) {
            d.device = metaIt->second.device;
            d.value = metaIt->second.value;
        }
        d.mismatches = s.mismatches;
        d.reasserts = s.reasserts;
        d.foreignWriter = s.foreign;
        d.lastWritten = s.lastWritten;
        d.lastReadback = s.lastReadback;
        out.push_back(std::move(d));
    }
    std::sort(out.begin(), out.end(), [](const InputDisturbance& a, const InputDisturbance& b) {
        return a.device != b.device ? a.device < b.device : a.value < b.value;
    });
    return out;
}

}  // namespace wpiemu::bridges::vendor
