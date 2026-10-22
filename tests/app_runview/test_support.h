#pragma once

// Shared fixtures for the run view scene and camera tests.

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

#include <string>
#include <vector>

#include "sound_output.h"

namespace runview_test {

namespace sdk = wpiemu::sdk;

// Records what the run view asked a sound output to do, without any audio device.
class RecordingSoundOutput final : public wpiemu::app::runview::ISoundOutput {
public:
    void setCatalog(const std::vector<sdk::SoundCueInfo>& cues) override {
        ++catalogLoads;
        catalogSize = cues.size();
    }
    void play(const std::string& cueId) override { events.push_back(cueId); }
    void stopAll() override { events.push_back("<stop>"); }
    void problem(const std::string& message) { reportProblem(message); }

    std::vector<std::string> events;
    int catalogLoads = 0;
    std::size_t catalogSize = 0;
};

inline constexpr float kEps = 1e-3f;

inline bool near(const sdk::Vec3& a, const sdk::Vec3& b, float eps = kEps) { return sdk::length(a - b) < eps; }

// A connected robot standing at (3, 2, 0.2), facing +X.
inline sdk::SimSnapshotData baseSnapshot() {
    sdk::SimSnapshotData d;
    d.chassis.pose = {{3.f, 2.f, 0.2f}, sdk::Quat::fromYaw(0.f)};
    d.robotConnected = true;
    return d;
}

}  // namespace runview_test
