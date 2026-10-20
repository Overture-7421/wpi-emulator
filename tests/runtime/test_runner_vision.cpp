#include <doctest/doctest.h>

#include <algorithm>

#include "runner_rig.h"

// SessionRunner, part 3: cameras - the VisionSetup that follows the HelloAck, frames after their latency, and a config without cameras.

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::Rig;
using wpiemu::test_runtime::sampleConfig;

namespace {

RobotConfig cameraConfig() {
    RobotConfig c = sampleConfig();
    c.gamePluginId = "tag_game";
    c.cameras.push_back(wpiemu::test_runtime::frontCamera());
    return c;
}

void addTagGame(Registries& reg) { REQUIRE(reg.plugins.add(std::make_unique<wpiemu::test_runtime::TagGameModule>()).ok()); }

}  // namespace

TEST_CASE("runner: VisionSetup follows the HelloAck and precedes the first frame of every epoch and connection") {
    RunnerOptions options = Rig::fast();
    options.photonVersion = "v2026.3.4";
    Rig rig(options, cameraConfig(), addTagGame);
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 10; }));
    REQUIRE(rig.ext->trace.size() >= 4);
    CHECK(rig.ext->trace[0] == "ack");
    CHECK(rig.ext->trace[1] == "vision");
    CHECK(rig.ext->trace[2] == "vision");
    CHECK(rig.ext->trace[3] == "frame");
    REQUIRE(rig.ext->visionSetups.size() == 2);
    for (const VisionSetup& setup : rig.ext->visionSetups) {
        REQUIRE(setup.cameras.size() == 1);
        CHECK(setup.cameras[0].config.id == "front");
        CHECK(setup.photonVersion == "v2026.3.4");
        CHECK(setup.layoutName == "tag-layout");
    }

    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    const auto firstEpoch2Frame = [&] {
        const auto& frames = rig.ext->sensorFrames;
        return static_cast<std::size_t>(
            std::find_if(frames.begin(), frames.end(), [](const proto::SensorFrame& f) { return f.runEpoch == 2; }) - frames.begin());
    };
    REQUIRE(rig.run([&] { return firstEpoch2Frame() < rig.ext->sensorFrames.size(); }));
    CHECK(rig.ext->visionSetups.size() == 3);
    std::size_t frameNumber = 0;
    for (std::size_t i = 0; i < rig.ext->trace.size(); ++i) {
        if (rig.ext->trace[i] != "frame" || frameNumber++ != firstEpoch2Frame()) continue;
        CHECK(rig.ext->trace[i - 1] == "vision");
    }

    // a reconnect gets its own setup right after the ack
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return !rig.ext->visionSetups.empty(); }));
    CHECK(rig.ext->trace[0] == "ack");
    CHECK(rig.ext->trace[1] == "vision");
}

TEST_CASE("runner: camera frames reach the SensorFrame after their latency") {
    Rig rig(Rig::fast(), cameraConfig(), addTagGame);
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 12; }));
    const auto& frames = rig.ext->sensorFrames;
    for (std::size_t i = 0; i < 4; ++i) CHECK(frames[i].vision.empty());  // 20 ms of latency = 4 steps
    for (std::size_t i = 4; i < 12; ++i) {
        REQUIRE(frames[i].vision.size() == 1);
        const CameraFrame& v = frames[i].vision[0];
        CHECK(v.captureTimeUs == static_cast<SimTimeUs>((i - 4 + 1) * 5000));
        CHECK(v.publishTimeUs == frames[i].simTimeUs);
        CHECK(v.cameraIndex == 0);
        REQUIRE(v.tags.size() == 1);
        CHECK(v.tags[0].id == 3);
    }
}

TEST_CASE("runner: a camera-less config sends an empty VisionSetup and no camera frames") {
    Rig rig;
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 6; }));
    REQUIRE(rig.ext->visionSetups.size() == 2);
    for (const VisionSetup& setup : rig.ext->visionSetups) {
        CHECK(setup.cameras.empty());
        CHECK(setup.photonVersion.empty());
    }
    for (const proto::SensorFrame& f : rig.ext->sensorFrames) CHECK(f.vision.empty());
}
