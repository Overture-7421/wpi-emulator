#pragma once

// Shared IVisionBridge lifecycle for NetworkTables-backed camera bridges (internal).

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>

#include "nt_sink.h"
#include "wpiemu/bridges/vision/vision_bridges.h"
#include "wpiemu/sdk/vision/camera_bridge.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

namespace wpiemu::bridges::vision::detail {

// Common NetworkTables bridge lifecycle: owns the NtSink, heartbeat options, CameraSet<Cam>, and started state,
// and implements start(), poll(), apply(), reset(), and stop() with lazy ntcore initialization retry.
//
// `Derived` provides:
//   void onInitializeCamera(Cam& cam, std::int64_t ntNow);
//   void onPollCamera(Cam& cam, sdk::SimTimeUs simTimeUs, std::int64_t ntNow);
//   void onFrame(Cam& cam, const sdk::CameraFrame& frame, std::int64_t ntNow);
// and may optionally override:
//   bool isEnabled() const;                  // default: true
//   void onPollBridge(std::int64_t ntNow);   // default: no-op (called after at least one camera is initialized)
template <class Derived, class Cam>
class NtVisionBridgeBase : public sdk::IVisionBridge {
public:
    NtVisionBridgeBase(std::shared_ptr<INtBackend> backend, std::string logCategory, VisionBridgeOptions options)
        : sink_(std::move(backend), std::move(logCategory)), options_(options) {}

    sdk::Status start(sdk::ILogger& logger) override {
        sink_.setLogger(&logger);
        started_ = true;
        return sdk::okStatus();
    }

    void poll(sdk::SimTimeUs simTimeUs) override {
        if (!derived().isEnabled() || cams_.empty() || !sink_.available()) return;
        const std::int64_t ntNow = sink_.backend().nowUs();
        bool any = false;
        cams_.forEach([&](Cam& cam) {
            if (!ensureInitialized(cam, ntNow)) return;
            any = true;
            derived().onPollCamera(cam, simTimeUs, ntNow);
        });
        if (any) derived().onPollBridge(ntNow);
    }

    void apply(std::span<const sdk::CameraFrame> frames, sdk::SimTimeUs) override {
        if (!derived().isEnabled() || frames.empty() || !sink_.available()) return;
        const std::int64_t ntNow = sink_.backend().nowUs();
        for (const sdk::CameraFrame& frame : frames) {
            Cam* cam = cams_.find(frame.cameraIndex);
            if (cam == nullptr || !ensureInitialized(*cam, ntNow)) continue;
            derived().onFrame(*cam, frame, ntNow);
        }
    }

    void reset() override {
        // Republish an initial no-target / empty state now when ntcore is up, otherwise on the first available tick.
        cams_.reset([this](Cam& cam) {
            if (derived().isEnabled() && sink_.available()) ensureInitialized(cam, sink_.backend().nowUs());
        });
    }

    void stop() noexcept override {
        if (started_ && sink_.available()) {
            cams_.forEach([this](const Cam& cam) { sink_.backend().unpublish(cam.root + "/"); });
        }
        cams_.clear();
        started_ = false;
    }

protected:
    bool isEnabled() const { return true; }
    void onPollBridge(std::int64_t) {}

    template <class RootOf>
    void configureCameras(const sdk::VisionSetup& setup, sdk::CameraKind kind, RootOf&& rootOf) {
        cams_.configure(setup, kind, std::forward<RootOf>(rootOf));
    }

    // ntcore may not be ready until the robot constructor ran; callers retry on the next tick.
    bool ensureInitialized(Cam& cam, std::int64_t ntNow) {
        if (cam.initialized) return true;
        if (!sink_.available()) return false;
        cam.initialized = true;
        derived().onInitializeCamera(cam, ntNow);
        return true;
    }

    NtSink sink_;
    VisionBridgeOptions options_;
    sdk::CameraSet<Cam> cams_;
    bool started_ = false;

private:
    Derived& derived() { return static_cast<Derived&>(*this); }
    const Derived& derived() const { return static_cast<const Derived&>(*this); }
};

}  // namespace wpiemu::bridges::vision::detail
