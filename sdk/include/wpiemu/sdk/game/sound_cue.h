#pragma once

// Sound cues: sounds a game asks the app to play (match horns, ...). The plugin names and ships them
// (IGameModule::contributeSoundCues) and raises them while it ticks (SoundCueLog -> snapshot); the app owns playback.
// Declared cues are data: the app never learns which season they belong to.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::sdk {

struct SoundCueInfo {
    std::string id;           // prefixed with the plugin id: "game_2026.sound.match_start"
    std::string label;        // UI text
    std::string description;  // what triggers it
    // Audio file, relative to the plugin tree root (plugins/ in this repository), e.g. "game_2026/assets/audio/start.wav".
    // The app embeds the plugin's audio at build time and loads it under that path.
    std::string asset;
    float gain = 1.f;               // 0..1 loudness trim applied by the player
    bool interruptsOthers = false;  // stop every other playing cue first (a "stop everything" horn)
};

// The cues a game raised recently. Snapshots are published slower than the sim ticks and the UI may skip some, so a
// cue stays in the log for a while and carries a sequence number: the UI plays each sequence once.
// The sequence keeps increasing across clear(), so a reset never makes old and new cues collide.
class SoundCueLog {
public:
    static constexpr std::size_t kCapacity = 8;

    void raise(std::string_view cueId, SimTimeUs simTimeUs) {
        events_.push_back({std::string(cueId), ++lastSequence_, simTimeUs});
        while (events_.size() > kCapacity) events_.pop_front();
    }
    void clear() { events_.clear(); }
    void publish(SnapshotBuilder& out) const {
        for (const SoundCueEvent& e : events_) out.addSoundCue(e);
    }

    const std::deque<SoundCueEvent>& events() const { return events_; }
    std::uint64_t lastSequence() const { return lastSequence_; }

private:
    std::deque<SoundCueEvent> events_;
    std::uint64_t lastSequence_ = 0;
};

}  // namespace wpiemu::sdk
