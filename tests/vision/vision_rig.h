#pragma once

// A tick-by-tick harness for VisionSystem: a fake world with a kinematic chassis, an Rng, and the
// per-tick call order of SimulationSession (step at T(N), then collectDue for SensorFrame N).

#include <doctest/doctest.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vision_test_support.h"
#include "wpiemu/core/vision/vision_system.h"

namespace vision_test {

inline vis::VisionWorldView viewOf(const FakeWorld& w,
                                   const sdk::IMechanismDirectory* mechanisms = nullptr) {
    return {w.world, w.chassis, mechanisms};
}
inline vis::VisionWorldView viewOf(const BoxWorld& w,
                                   const sdk::IMechanismDirectory* mechanisms = nullptr) {
    return {*w.world, w.chassis, mechanisms};
}

// Three tags on a wall 4 m ahead of the origin (facing the origin) at camera height 0.5 m.
inline sdk::FiducialLayout wallLayout() {
    return layoutOf({tagAt(1, 4.f, 0.5f, 0.5f), tagAt(2, 4.f, -0.5f, 0.5f), tagAt(3, 4.f, 0.f, 1.f)});
}

// A chassis-mounted camera at (0, 0, 0.5) looking along +X (1280x800, 90 degrees).
inline sdk::CameraConfig frontCamera(const std::string& id = "front") {
    sdk::CameraConfig c = makeCamera(id);
    c.local = sdkPose(0.f, 0.f, 0.5f);
    return c;
}

struct Collected {
    int tick = 0;  // 1-based: the SensorFrame of tick `tick` carried it
    sdk::VisionFrame frame;
};

struct Rig {
    FakeWorld world;
    sdk::Rng rng{2024};
    std::unique_ptr<wpiemu::core::vision::VisionSystem> system;
    wpiemu::core::vision::RobotCameraStates robotStates;  // what the robot reported: shown in snapshots only
    sdk::SimTimeUs stepUs = 5000;
    sdk::SimTimeUs timeUs = 0;
    int tickCount = 0;
    const sdk::IMechanismDirectory* mechanisms = nullptr;

    explicit Rig(const sdk::VisionConfig& config, sdk::FiducialLayout layout = wallLayout(),
                 sdk::SimTimeUs step = 5000, const sdk::IMechanismDirectory* mech = nullptr)
        : stepUs(step), mechanisms(mech) {
        auto created = wpiemu::core::vision::VisionSystem::create(config, std::move(layout), step, mech);
        REQUIRE(created.ok());
        system = std::move(created).value();
    }

    // One tick: capture at T(N), then the frames due in the SensorFrame of tick N.
    std::vector<sdk::VisionFrame> tick() {
        timeUs += stepUs;
        ++tickCount;
        system->step(viewOf(world, mechanisms), rng, timeUs);
        std::vector<sdk::VisionFrame> out;
        system->collectDue(timeUs, out);
        return out;
    }

    std::vector<Collected> run(int ticks) {
        std::vector<Collected> all;
        for (int i = 0; i < ticks; ++i)
            for (sdk::VisionFrame& f : tick()) all.push_back({tickCount, std::move(f)});
        return all;
    }

    // The camera samples of a snapshot built right now.
    std::vector<sdk::CameraSample> cameras() const {
        sdk::SnapshotBuilder b;
        system->contribute(b, viewOf(world, mechanisms), robotStates);
        return b.data().cameras;
    }
    sdk::SimSnapshotData snapshot() const {
        sdk::SnapshotBuilder b;
        system->contribute(b, viewOf(world, mechanisms), robotStates);
        return std::move(b.data());
    }
    // Rng draws so far, given the seed the rig started with.
    std::uint64_t draws(std::uint64_t seed = 2024) const {
        sdk::Rng ref(seed);
        std::uint64_t n = 0;
        while (ref.state() != rng.state() && n < 1000000) {
            ref.nextU64();
            ++n;
        }
        return n;
    }
};

}  // namespace vision_test
