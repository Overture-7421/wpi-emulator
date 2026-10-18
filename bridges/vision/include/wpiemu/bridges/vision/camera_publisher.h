#pragma once

// The per-vendor seam of the vision bridge (ADR 0017): what a PhotonVision or a Limelight "virtual
// coprocessor" is to the bridge core.
//
//   VisionBridge (core/)  --owns-->  IVisionVendor (one per vendor: photon/, limelight/)
//                         --owns-->  ICameraPublisher (one per enabled camera, made by its vendor)
//                                      \-- uses INtApi, never the ntcore C API
//
// The bridge core knows nothing about topic names or message layouts, and a vendor knows nothing about
// the IPC, the lock-step or the bridge lifecycle. Adding a vendor = one folder under src/, one
// IVisionVendor, one line in the bridge's default vendor list (docs/CONTRIBUTING.md recipe).
//
// Threading: every method is called on the extension's clock thread while the robot is quiescent
// (between HALSIM_StepTiming calls), never concurrently; none may block, throw or touch ntcore other
// than through the INtApi it was given.

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/bridges/vision/nt_api.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::bridges::vision {

// Values of CameraRobotState::status ("" = healthy). They are part of the emulator-side warning mapping
// (core/runtime/src/runtime/session_runner.cpp), so extend them together.
namespace camera_status {
inline constexpr std::string_view kOk = "";
inline constexpr std::string_view kNtcoreUnavailable = "ntcore_unavailable";
inline constexpr std::string_view kPhotonUuidUnsupported = "photon_uuid_unsupported";
inline constexpr std::string_view kForeignPublisher = "foreign_publisher";
inline constexpr std::string_view kDisabled = "disabled";
// The vision bridge could not get a publisher for the camera's vendor (a build without that vendor).
inline constexpr std::string_view kVendorUnavailable = "vendor_unavailable";
}  // namespace camera_status

// One enabled camera as the robot process sees it. Created by IVisionVendor::createPublisher from the
// handshake's CameraSetup; owns its NT publishers and request subscribers.
class ICameraPublisher {
    WPIEMU_INTERFACE(ICameraPublisher)
public:
    virtual const sdk::CameraSetup& setup() const = 0;

    // Creates the camera's static topics and subscribes to the topics the robot writes (requests). Called
    // once, at the first poll after ntcore is bound and the robot program started (so everything the robot
    // constructor created already exists). A camera that must not publish (unknown PhotonLib message UUID,
    // a foreign publisher already on its result topic) stays INERT: it publishes nothing and reports the
    // reason through robotState().status, but still reads the robot's requests.
    virtual void start() = 0;

    // Every bridge poll(), after start(): drain the robot's writes (pipeline, driver mode, LED mode,
    // orientation, camera pose, ...) and keep the state-echo topics current. Cheap when nothing changed.
    virtual void poll() = 0;

    // One processed frame of THIS camera (VisionFrame::cameraId == setup().cameraId), applied between the
    // inputs and HALSIM_StepTiming of its tick. Publishes the result with ONE nowUs() reading for every
    // value of the frame (kept >= the previous frame's, ntcore drops older stamps), capture time
    // nowUs - frame.latencyUs. A no-op while the publisher is inert.
    virtual void publish(const sdk::VisionFrame& frame) = 0;

    // What the robot did with this camera (informational, ActuatorFrame::vision): status, publishing,
    // framesPublished, and every request the robot made so far. `present` = the robot code uses the camera
    // (its request topics exist).
    virtual sdk::CameraRobotState robotState() const = 0;
};

class IVisionVendor {
    WPIEMU_INTERFACE(IVisionVendor)
public:
    virtual sdk::CameraVendor vendor() const = 0;

    // Topic prefix under which this vendor's cameras live ("/photonvision/", "/limelight"); the bridge lists
    // the existing topics below it for discoverRobotCameras().
    virtual std::string_view topicPrefix() const = 0;

    // Never returns null for a supported setup; an unsupported one (for example a PhotonVision message UUID
    // without a profile) still gets a publisher that is inert and reports the reason.
    virtual std::unique_ptr<ICameraPublisher> createPublisher(INtApi& nt, sdk::ILogger& log,
                                                              const sdk::CameraSetup& setup) const = 0;

    // From the topics that currently exist below topicPrefix(), appends the NetworkTables name of every
    // camera the ROBOT uses (judged only from topics the robot code creates, never from the bridge's own
    // publications, so a configured camera does not count as discovered by itself). No duplicates.
    virtual void discoverRobotCameras(std::span<const NtTopicInfo> topics, std::vector<std::string>& ntNames) const = 0;
};

}  // namespace wpiemu::bridges::vision
