#pragma once

// The vision bridge (ADR 0017): the IDeviceBridge "vision". It turns the emulator's vendor-neutral camera frames
// (SensorFrame::vision) into the NetworkTables entries of virtual PhotonVision / Limelight coprocessors, inside the
// robot process, and reports back what the robot did with each camera (ActuatorFrame::vision, informational only).
//
//   VisionBridge --owns--> INtApi            the only door to NetworkTables (shared: tests keep a handle)
//                --owns--> IVisionVendor     one per vendor, makes the publishers and recognises the robot's cameras
//                --owns--> ICameraPublisher  one per camera of host.visionSetup(), made by its vendor
//
// Everything runs on the extension's clock thread while the robot is quiescent: nothing blocks, nothing throws (a
// publisher that throws is disabled, alone, and logged once) and this bridge registers no HAL callbacks.
//
// start(host)
//     Keeps the host and logger and builds the vendors. No ntcore access.
// poll()
//     Waits for ntcore to bind AND the robot program to start (re-checked every poll; ONE warning with the reason
//     when ntcore is still missing after the program started). Then it creates and starts a publisher for every
//     camera of host.visionSetup(), exactly once each. Afterwards it polls the publishers, follows
//     visionSetupRevision() (a reconnect with another HelloAck: publishers of unchanged cameras are kept, changed
//     ones recreated, removed ones go quiet, nothing is ever unpublished) and, rate limited, scans the topics the
//     robot created to find the cameras it uses (also with no camera configured, so the editor can offer
//     "add detected camera").
// applyVision(frames)
//     Routes each frame, in order and without copying it, to the publisher of frame.cameraId. A frame that cannot
//     be published (unknown camera, no running publisher, before start, after stop) is dropped and counted.
// collectVision(out, full)
//     Appends the state of every configured camera and of every unconfigured camera the robot uses: all of them when
//     `full`, otherwise only what changed since the last call. A change of framesPublished alone is reported at most
//     every VisionBridgeOptions::framesReportIntervalUs.
// stop()
//     From then on nothing reaches INtApi or a publisher (ntcore may be torn down right after the extension's
//     shutdown hook). NT handles are never released.
//
// A configured camera without a running publisher is reported with status "ntcore_unavailable" (ntcore not bound),
// "vendor_unavailable" (no vendor, or a vendor that made no publisher) or "disabled" (its publisher threw). Cameras
// are name keyed: this bridge announces no devices (owns() is false).

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/bridges/vision/camera_publisher.h"
#include "wpiemu/bridges/vision/nt_api.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::bridges::vision {

struct VisionBridgeOptions {
    // The NetworkTables seam. Null: makeNtCoreApi(), the run-time binding of the robot's own ntcore.
    std::shared_ptr<INtApi> nt;
    // The vendors, at most one per CameraVendor. Unset: the shipped ones (PhotonVision, Limelight). A list that is set,
    // even an empty one, replaces them (a build without a vendor, tests).
    std::optional<std::vector<std::shared_ptr<IVisionVendor>>> vendors;
    // Minimum NT time between two scans of the topics the robot created (camera discovery).
    std::int64_t discoveryIntervalUs = 100'000;
    // Minimum NT time between two reports of a camera whose only change is CameraRobotState::framesPublished.
    std::int64_t framesReportIntervalUs = 250'000;
};

// Counters for tests and diagnostics (the extension does not read them yet).
struct VisionBridgeStats {
    std::uint64_t framesRouted = 0;       // frames handed to a running publisher
    std::uint64_t framesDropped = 0;      // frames nobody could publish
    std::uint64_t publishersStarted = 0;  // publishers created and started (a recreated camera counts again)
    std::uint64_t discoveryRuns = 0;      // scans of the robot's topics
    std::uint64_t failures = 0;           // exceptions caught from a vendor or a publisher
    std::size_t vendors = 0;              // vendors the bridge works with (set by start())
};

class VisionBridge final : public sdk::IDeviceBridge {
public:
    explicit VisionBridge(VisionBridgeOptions options = {});
    ~VisionBridge() override;

    // IDeviceBridge
    std::string_view name() const override { return "vision"; }
    sdk::Status start(sdk::IBridgeHost& host) override;
    void poll() override;
    void collectOutputs(std::vector<sdk::SignalSample>&) override {}
    void applyInputs(std::span<const sdk::SignalSample>) override {}
    void applyLoopback(sdk::DeviceId) override {}
    bool owns(sdk::DeviceId) const override { return false; }
    void stop() noexcept override;
    // Publishers keep their heartbeat and frame sequence monotonic across epochs: nothing to forget.
    void resetInputHistory() override {}
    void applyVision(std::span<const sdk::VisionFrame> frames) override;
    void collectVision(std::vector<sdk::CameraRobotState>& out, bool full) override;

    const VisionBridgeStats& stats() const { return stats_; }

private:
    enum class Phase : std::uint8_t {
        Waiting,   // no publisher yet: ntcore not bound, program not started, or just not created
        Running,   // publisher created and started
        NoVendor,  // no vendor, or the vendor made no publisher (a build without it): vendor_unavailable
        Disabled   // the publisher threw: no more calls into it
    };

    struct Camera {
        sdk::CameraSetup setup;
        IVisionVendor* vendor = nullptr;  // null: no vendor for setup.vendor
        std::unique_ptr<ICameraPublisher> publisher;
        Phase phase = Phase::Waiting;
        sdk::CameraRobotState placeholder;  // what collectVision reports while there is no running publisher
        sdk::CameraRobotState lastReported;
        bool hasReported = false;
        std::int64_t lastReportUs = 0;
        bool loggedNoVendor = false;
        bool loggedFailure = false;
    };

    struct VendorSlot {
        std::shared_ptr<IVisionVendor> vendor;
        sdk::CameraVendor kind = sdk::CameraVendor::PhotonVision;
        bool discoveryFailed = false;
    };

    // A camera the robot uses that is not configured (CameraRobotState::cameraId is empty).
    struct Detected {
        sdk::CameraRobotState state;
        bool reportPending = true;
        bool seen = false;  // scratch of one discovery run
    };

    bool running() const { return host_ != nullptr; }
    void buildVendors();
    IVisionVendor* vendorFor(sdk::CameraVendor kind) const;

    void pollImpl();
    void syncSetup();
    void adoptSetup(const sdk::VisionSetup& setup);
    void onNtBoundChanged(bool bound);
    void warnNtUnavailable();
    void startPendingCameras();
    void pollCameras();
    void markNoVendor(Camera& camera);
    void setPhase(Camera& camera, Phase phase);
    void refreshPlaceholder(Camera& camera) const;
    template <class Fn>
    bool guarded(Camera& camera, const char* what, Fn&& fn) noexcept;
    void disableCamera(Camera& camera, const char* what, const char* reason) noexcept;
    void internalError(const char* where) noexcept;

    void routeFrame(const sdk::VisionFrame& frame);
    void warnUnknownCamera(const std::string& cameraId);

    bool discoveryDue(std::int64_t nowUs) const;
    void runDiscovery(std::int64_t nowUs);
    void updateDetected(sdk::CameraVendor kind, const std::vector<std::string>& names);
    bool isConfigured(sdk::CameraVendor kind, const std::string& ntName) const;

    void collectImpl(std::vector<sdk::CameraRobotState>& out, bool full);
    bool framesReportDue(std::int64_t nowUs, std::int64_t lastReportUs) const;

    VisionBridgeOptions options_;
    std::shared_ptr<INtApi> nt_;
    std::vector<VendorSlot> vendors_;
    std::vector<Camera> cameras_;
    std::vector<Detected> detected_;

    sdk::IBridgeHost* host_ = nullptr;  // null before start() and after stop()
    sdk::ILogger* log_ = nullptr;
    bool stopped_ = false;
    bool vendorsBuilt_ = false;

    bool setupAdopted_ = false;
    std::uint64_t setupRevision_ = 0;
    bool ntBound_ = false;
    bool programStarted_ = false;
    bool warnedNtUnavailable_ = false;

    bool discoveryRan_ = false;
    std::int64_t lastDiscoveryUs_ = 0;
    std::vector<NtTopicInfo> topicScratch_;
    std::vector<std::string> nameScratch_;

    std::vector<std::string> warnedUnknownCameras_;
    bool warnedTooManyDetected_ = false;
    bool loggedInternalError_ = false;
    VisionBridgeStats stats_;
};

}  // namespace wpiemu::bridges::vision
