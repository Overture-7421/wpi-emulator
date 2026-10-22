#pragma once

// A bounded, thread-safe sound cue mailbox: the sim thread posts, the UI side drains. When the consumer stalls the
// oldest cues are dropped (a cue is only worth playing soon after it was requested).

#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

#include "wpiemu/sdk/game/sound_cue.h"

namespace wpiemu::sdk {

class SoundCueQueue final : public ISoundCueSink {
public:
    static constexpr std::size_t kDefaultCapacity = 64;

    explicit SoundCueQueue(std::size_t capacity = kDefaultCapacity) : capacity_(capacity == 0 ? 1 : capacity) {}

    void post(SoundCue cue) override {
        std::lock_guard lock(mutex_);
        if (cues_.size() >= capacity_) cues_.pop_front();
        cues_.push_back(std::move(cue));
    }

    // Removes and returns up to `maxCues` cues, oldest first.
    std::vector<SoundCue> take(std::size_t maxCues = kDefaultCapacity) {
        std::vector<SoundCue> out;
        std::lock_guard lock(mutex_);
        while (!cues_.empty() && out.size() < maxCues) {
            out.push_back(std::move(cues_.front()));
            cues_.pop_front();
        }
        return out;
    }

    std::size_t size() const {
        std::lock_guard lock(mutex_);
        return cues_.size();
    }

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<SoundCue> cues_;
};

}  // namespace wpiemu::sdk
