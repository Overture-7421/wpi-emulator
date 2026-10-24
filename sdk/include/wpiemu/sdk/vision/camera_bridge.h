#pragma once

// Lifecycle bookkeeping shared by the camera-coprocessor encoders (Limelight, PhotonVision, ...). Bridge modules cannot
// depend on each other, so what the encoders have in common lives here: the per-camera record, the cameras of one kind
// addressed by VisionSetup index, and the heartbeat cadence. Topics, encodings and the robot settings an encoder
// mirrors stay in the bridge.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/camera.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/vision/camera_frame.h"

namespace wpiemu::sdk {

// What every encoder keeps per camera; an encoder derives from it to add the settings it mirrors.
struct CameraBridgeState {
    CameraConfig config;
    std::string root;                  // topic prefix of the camera, e.g. "/limelight-front"
    bool initialized = false;          // its topics exist
    std::int64_t heartbeat = 0;
    SimTimeUs nextHeartbeatUs = 0;

    void restartHeartbeat() {
        heartbeat = 0;
        nextHeartbeatUs = 0;
    }

    // True, and counted, when a heartbeat is due at `simTimeUs`; the next one is due `periodUs` later.
    bool beat(SimTimeUs simTimeUs, SimTimeUs periodUs) {
        if (simTimeUs < nextHeartbeatUs) return false;
        ++heartbeat;
        nextHeartbeatUs = simTimeUs + periodUs;
        return true;
    }
};

// The cameras of one CameraKind. Slots keep the indices of VisionSetup::cameras (CameraFrame::cameraIndex), so the
// cameras of other kinds leave empty slots.
template <class Cam>
class CameraSet {
public:
    // Rebuilds the set: one camera per `kind` entry of `setup`, rooted at `rootOf(config)`.
    template <class RootOf>
    void configure(const VisionSetup& setup, CameraKind kind, RootOf&& rootOf) {
        slots_.clear();
        slots_.resize(setup.cameras.size());
        for (std::size_t i = 0; i < setup.cameras.size(); ++i) {
            const CameraConfig& config = setup.cameras[i].config;
            if (config.kind != kind) continue;
            Cam& cam = slots_[i].emplace();
            cam.config = config;
            cam.root = rootOf(config);
        }
    }

    // Null for an unknown index or a camera of another kind.
    Cam* find(std::uint32_t index) { return index < slots_.size() && slots_[index] ? &*slots_[index] : nullptr; }

    // True when the setup listed no camera at all, whatever its kind.
    bool empty() const { return slots_.empty(); }
    void clear() { slots_.clear(); }

    template <class F>
    void forEach(F&& f) {
        for (std::optional<Cam>& slot : slots_)
            if (slot) f(*slot);
    }
    template <class F>
    void forEach(F&& f) const {
        for (const std::optional<Cam>& slot : slots_)
            if (slot) f(*slot);
    }

    // New connection or epoch: every camera must create its topics again. `republish` runs for the cameras that had
    // them, which restores a "no target" state now instead of waiting for the next frame.
    template <class Republish>
    void reset(Republish&& republish) {
        forEach([&](Cam& cam) {
            const bool wasInitialized = cam.initialized;
            cam.initialized = false;
            cam.nextHeartbeatUs = 0;
            if (wasInitialized) republish(cam);
        });
    }

private:
    std::vector<std::optional<Cam>> slots_;
};

}  // namespace wpiemu::sdk
