#include "wpiemu/bridges/vendor/incremental_backend.h"

#include <algorithm>

namespace wpiemu::bridges::vendor {

void IncrementalSimDeviceBackend::listDevices(std::vector<sdk::SimDeviceInfo>& out) const {
    std::vector<sdk::SimDeviceInfo> real;
    inner_->listDevices(real);

    std::lock_guard lock(mutex_);
    std::set<sdk::SimDeviceHandle> present;
    for (const auto& d : real) present.insert(d.handle);
    for (auto it = bases_.begin(); it != bases_.end();) {
        if (present.count(it->first) != 0) {
            ++it;
            continue;
        }
        for (const auto s : it->second.synthetics) synthetics_.erase(s);
        it = bases_.erase(it);
    }

    for (const auto& dev : real) {
        out.push_back(dev);
        Base& base = bases_[dev.handle];
        if (base.listed) {  // the consumer already bound this device: look for values created since
            std::vector<sdk::SimValueInfo> current;
            inner_->listValues(dev.handle, current);
            std::vector<sdk::SimValueInfo> fresh;
            for (auto& v : current)
                if (base.reported.insert(v.handle).second) fresh.push_back(std::move(v));
            if (!fresh.empty()) {
                const sdk::SimDeviceHandle handle{kSyntheticBit | nextSynthetic_++};
                synthetics_[handle] = {dev.handle, dev.name, std::move(fresh)};
                base.synthetics.push_back(handle);
            }
        }
        for (const auto s : base.synthetics) out.push_back({s, dev.name});
    }
}

void IncrementalSimDeviceBackend::listValues(sdk::SimDeviceHandle device, std::vector<sdk::SimValueInfo>& out) const {
    if ((device.value & kSyntheticBit) != 0) {
        std::lock_guard lock(mutex_);
        const auto it = synthetics_.find(device);
        if (it != synthetics_.end()) out.insert(out.end(), it->second.values.begin(), it->second.values.end());
        return;
    }
    std::vector<sdk::SimValueInfo> values;
    inner_->listValues(device, values);
    std::lock_guard lock(mutex_);
    Base& base = bases_[device];
    base.listed = true;
    for (auto& v : values) {
        base.reported.insert(v.handle);
        out.push_back(std::move(v));
    }
}

std::size_t IncrementalSimDeviceBackend::lateValueDeviceCount() const {
    std::lock_guard lock(mutex_);
    return synthetics_.size();
}

}  // namespace wpiemu::bridges::vendor
