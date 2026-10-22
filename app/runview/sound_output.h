#pragma once

// Where match sounds are actually played. Qt-free so the view-model and its tests never need an audio device; the
// build's real implementation is created by createDefaultSoundOutput().

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/game/sound_cue.h"

namespace wpiemu::app::runview {

class ISoundOutput {
public:
    virtual ~ISoundOutput() = default;

    // Replaces the playable cues (loads their audio).
    virtual void setCatalog(const std::vector<sdk::SoundCueInfo>& cues) = 0;
    // Starts a cue; one that is already playing restarts.
    virtual void play(const std::string& cueId) = 0;
    virtual void stopAll() = 0;

    // Receives a readable description of a problem (no audio device, unreadable file), once per cause.
    void setProblemHandler(std::function<void(const std::string&)> handler) { onProblem_ = std::move(handler); }

protected:
    void reportProblem(const std::string& message) const {
        if (onProblem_) onProblem_(message);
    }

private:
    std::function<void(const std::string&)> onProblem_;
};

// The Qt Multimedia output of this build.
std::unique_ptr<ISoundOutput> createDefaultSoundOutput();

}  // namespace wpiemu::app::runview
