#include <doctest/doctest.h>

#include <array>
#include <cstdint>
#include <map>
#include <random>
#include <span>
#include <string>
#include <vector>

#include "vision_bridge_test_support.h"

// A property test over random sessions: whatever the robot, the publishers and the emulator do (setup changes, request
// changes, frames, cameras appearing on the robot), a receiver that applies the bridge's delta reports to what it was
// told at the start of the connection always ends up with exactly what a full report says.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

constexpr int kPoolSize = 5;

std::string keyOf(const sdk::CameraRobotState& state) {
    if (!state.cameraId.empty()) return "camera:" + state.cameraId;
    return "detected:" + std::to_string(static_cast<int>(state.vendor)) + ":" + state.ntName;
}

// What the emulator keeps of ActuatorFrame::vision: a full report replaces everything, a delta updates entries.
class Receiver {
public:
    void apply(const std::vector<sdk::CameraRobotState>& reports, bool full) {
        if (full) states_.clear();
        for (const sdk::CameraRobotState& state : reports) states_[keyOf(state)] = state;
    }
    // The entries a full report lists: everything except detections the robot stopped using.
    std::map<std::string, sdk::CameraRobotState> inUse() const {
        std::map<std::string, sdk::CameraRobotState> out;
        for (const auto& [key, state] : states_)
            if (state.cameraId.empty() ? state.present : true) out[key] = state;
        return out;
    }

private:
    std::map<std::string, sdk::CameraRobotState> states_;
};

sdk::CameraSetup poolCamera(int index, int variant) {
    const std::string id = "cam" + std::to_string(index);
    sdk::CameraSetup setup = index % 2 == 0 ? photonCamera(id, "photon-" + id) : limelightCamera(id, "limelight-" + id);
    setup.latencyUs = 25000 + 15000 * variant;
    return setup;
}

void runRandomSession(std::uint32_t seed) {
    CAPTURE(seed);
    std::mt19937 rng(seed);
    const auto chance = [&](int percent) { return static_cast<int>(rng() % 100) < percent; };
    const auto pick = [&](int count) { return static_cast<int>(rng() % static_cast<std::uint32_t>(count)); };

    VisionRig rig;
    rig.fake->setAvailable(false);  // ntcore binds after a few ticks, like in a Java robot
    rig.start();

    std::vector<sdk::CameraSetup> current;  // the setup of the connection
    std::vector<std::uint64_t> sequence(kPoolSize, 0);
    Receiver receiver;
    bool newConnection = true;  // the next report is the full list of a new connection

    const auto adoptRandomSetup = [&] {
        sdk::VisionSetup setup;
        current.clear();
        for (int i = 0; i < kPoolSize; ++i) {
            if (!chance(60)) continue;
            current.push_back(poolCamera(i, pick(2)));
            setup.cameras.push_back(current.back());
        }
        rig.host.adopt(std::move(setup));
        newConnection = true;
    };
    const auto mutateRandomProbe = [&] {
        if (current.empty()) return;
        const sdk::CameraSetup& camera = current[static_cast<std::size_t>(pick(static_cast<int>(current.size())))];
        const auto vendor = camera.vendor == sdk::CameraVendor::PhotonVision ? rig.photon : rig.limelight;
        const auto probe = vendor->probeFor(camera.cameraId);
        if (probe == nullptr || !probe->alive) return;
        sdk::CameraRobotState& state = probe->state;
        switch (pick(6)) {
            case 0:
                state.pipelineIndex = pick(4);
                break;
            case 1:
                state.ledMode = pick(4);
                break;
            case 2:
                state.driverMode = chance(50);
                break;
            case 3:
                state.status = chance(50) ? std::string(camera_status::kForeignPublisher) : std::string();
                break;
            case 4:
                state.present = chance(50);
                break;
            default:
                state.robotOrientationDeg = std::array<double, 6>{static_cast<double>(pick(360)), 0, 0, 0, 0, 0};
                break;
        }
    };

    int robotCameras = 0;
    const auto tickAndReceive = [&](std::span<const sdk::VisionFrame> frames) {
        const bool full = newConnection;
        newConnection = false;
        const auto reports = rig.tick(frames, full);
        receiver.apply(reports, full);
    };

    adoptRandomSetup();
    for (int tick = 1; tick <= 1500; ++tick) {
        if (tick == 12) rig.fake->setAvailable(true);
        if (chance(4)) adoptRandomSetup();
        if (chance(25)) mutateRandomProbe();
        if (chance(3) && robotCameras < 6) {  // the robot code creates another camera object
            const std::string name = "robot-" + std::to_string(robotCameras++);
            rig.fake->robotPublish("/photonvision/" + name + "/pipelineIndexRequest", NtType::Integer);
        }
        if (chance(2)) {
            const std::string name = "robot-" + std::to_string(pick(6));
            if (rig.photon->ignoredNames.count(name) != 0) {
                rig.photon->ignoredNames.erase(name);
            } else {
                rig.photon->ignoredNames.insert(name);
            }
        }

        std::vector<sdk::VisionFrame> frames;
        for (const sdk::CameraSetup& camera : current)
            if (chance(40)) {
                const std::size_t index = static_cast<std::size_t>(camera.cameraId.back() - '0');
                frames.push_back(frameFor(camera, ++sequence[index]));
            }
        tickAndReceive(frames);

        if (tick % 97 == 0) {
            for (int i = 0; i < 60; ++i) tickAndReceive({});  // 300 ms of quiet: every pending refresh is due
            const auto expected = receiver.inUse();
            const auto full = rig.collect(true);
            REQUIRE(full.size() == expected.size());
            for (const sdk::CameraRobotState& state : full) {
                const auto it = expected.find(keyOf(state));
                REQUIRE(it != expected.end());
                CHECK(it->second == state);
            }
            receiver.apply(full, true);
        }
    }

    // Whatever happened, every started publisher was started exactly once and no frame was lost track of.
    for (const auto& vendor : {rig.photon, rig.limelight})
        for (const auto& probe : vendor->probes) CHECK(probe->starts == 1);
    CHECK(rig.bridge->stats().publishersStarted == rig.photon->probes.size() + rig.limelight->probes.size());
}

TEST_CASE(
    "vision bridge: delta reports applied to a connection's first full report always match a full report (random)") {
    for (const std::uint32_t seed : {1u, 2u, 3u, 7u, 42u, 20260101u}) runRandomSession(seed);
}

}  // namespace
