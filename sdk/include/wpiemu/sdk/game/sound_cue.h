#pragma once

// Sound cues: what the Driver Station or a game plugin asks the app to play. The SDK owns the vocabulary and the
// hand-off (no audio, no Qt); the app owns playback. A cue is an id; SoundCueInfo says which sound file realises it.
//
// Ownership: an id's prefix decides who owns its sound file. The ids below (the FMS audio cues every season shares,
// manual Table 5-4) belong to the app: `asset` names app/runview/assets/sfx/<asset>.wav. A plugin's ids start with
// its plugin id and `asset` names seasons/<year>/game/assets/sfx/<asset>.wav.

#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

namespace sound_cue {
inline constexpr std::string_view kMatchStart = "match.start";          // "Cavalry Charge"
inline constexpr std::string_view kAutoEnd = "match.auto_end";          // "Buzzer"
inline constexpr std::string_view kTeleopStart = "match.teleop_start";  // "3 Bells"
inline constexpr std::string_view kEndGame = "match.end_game";          // "Steam Whistle"
inline constexpr std::string_view kMatchEnd = "match.end";              // "Buzzer"
inline constexpr std::string_view kMatchStopped = "match.stopped";      // "Foghorn"
inline constexpr std::string_view kMatchResult = "match.result";        // the final score is posted
inline constexpr std::string_view kFieldReset = "field.reset";
}  // namespace sound_cue

struct SoundCueInfo {
    std::string id;           // "match.start", or prefixed with the plugin id: "game_2026.shift_change"
    std::string label;        // UI text
    std::string description;
    std::string asset;        // base name (no extension) of a .wav in the owner's sfx folder
};

// The cues every season shares. The Driver Station requests the match lifecycle ones; a plugin requests the ones whose
// timing is its season's (end game, result).
inline const std::vector<SoundCueInfo>& commonSoundCues() {
    static const std::vector<SoundCueInfo> cues = {
        {std::string(sound_cue::kMatchStart), "Match start", "Cavalry Charge: AUTO begins", "start"},
        {std::string(sound_cue::kAutoEnd), "Autonomous ends", "Buzzer: AUTO ends", "end"},
        {std::string(sound_cue::kTeleopStart), "Teleop starts", "3 Bells: TELEOP begins", "resume"},
        {std::string(sound_cue::kEndGame), "End game", "Steam Whistle: END GAME begins", "warning"},
        {std::string(sound_cue::kMatchEnd), "Match ends", "Buzzer: the MATCH is over", "end"},
        {std::string(sound_cue::kMatchStopped), "Match stopped", "Foghorn: the MATCH was stopped (E-Stop)", "abort"},
        {std::string(sound_cue::kMatchResult), "Match result", "The final score is posted", "match_result"},
        {std::string(sound_cue::kFieldReset), "Field reset", "The field was reset", "field_reset"},
    };
    return cues;
}

struct SoundCue {
    std::string id;
    SimTimeUs simTimeUs = 0;   // simulation time of the request
};

// Where requests go. post() is THREAD-SAFE and non-blocking: the sim thread calls it, another thread consumes.
class ISoundCueSink {
    WPIEMU_INTERFACE(ISoundCueSink)
public:
    virtual void post(SoundCue cue) = 0;
};

class NullSoundCueSink final : public ISoundCueSink {
public:
    void post(SoundCue) override {}
};

}  // namespace wpiemu::sdk
