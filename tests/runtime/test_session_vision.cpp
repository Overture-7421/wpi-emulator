#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

#include "runtime_fixtures.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/core/vision/vision_system.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::ScratchDir;
using wpiemu::test_runtime::wallCamera;
using wpiemu::test_runtime::wallLayoutJson;

namespace {

class LayoutModule final : public testing::FakeGameModule {
public:
    LayoutModule() : FakeGameModule("layout_game", 2998) {}
    std::vector<FiducialLayout> contributeFiducialLayouts() const override {
        FiducialLayout first, second;
        first.id = "layout_game.first";
        first.tags.push_back({11, Pose3{Vec3{4.f, 0.f, 0.5f}, Quat::fromYaw(3.14159265f)}, 0.0});
        second.id = "layout_game.second";
        second.tags.push_back({22, Pose3{Vec3{5.f, 0.f, 0.5f}, Quat::fromYaw(3.14159265f)}, 0.0});
        return {first, second};
    }
};

// Moves the robot during the game step, like a game that pushes it or a plugin that respawns it.
class PushModule final : public testing::FakeGameModule {
public:
    PushModule() : FakeGameModule("push_game", 2996) {}
    void onTick(SimContext& ctx, double) override {
        ctx.physics().setPose(ctx.chassisBody(), Pose3{Vec3{2.f, 0.f, 0.2f}, Quat{}});
    }
};

// Draws from SimContext::rng() on every tick, like a game module with random spawn jitter.
class RngConsumerModule final : public testing::FakeGameModule {
public:
    explicit RngConsumerModule(std::shared_ptr<std::vector<std::uint64_t>> draws)
        : FakeGameModule("rng_game", 2997), draws_(std::move(draws)) {}
    void onTick(SimContext& ctx, double) override { draws_->push_back(ctx.rng().nextU64()); }

private:
    std::shared_ptr<std::vector<std::uint64_t>> draws_;
};

struct Fixture {
    Registries reg;
    testing::CapturingLogger logger;
    ScratchDir dir;
    std::unique_ptr<SimulationSession> session;
    testing::FakePhysicsWorld* world = nullptr;

    // A config with one camera and the wall layout in a file.
    RobotConfig config(std::vector<CameraConfig> cameras = {wallCamera()}) {
        RobotConfig c = sampleConfig();
        c.vision.cameras = std::move(cameras);
        c.vision.layoutFile = dir.write("layout.json", wallLayoutJson()).string();
        return c;
    }
    Result<std::unique_ptr<SimulationSession>> build(RobotConfig cfg) {
        SessionParts p;
        auto w = std::make_unique<testing::FakePhysicsWorld>(cfg.sim.physics);
        world = w.get();
        p.world = std::move(w);
        p.config = std::move(cfg);
        p.mechanisms = &reg.mechanisms;
        p.drivetrains = &reg.drivetrains;
        p.tireModels = &reg.tires;
        p.plugins = &reg.plugins;
        p.logger = &logger;
        return SessionBuilder::build(std::move(p));
    }
    SimulationSession& make(RobotConfig cfg) {
        auto r = build(std::move(cfg));
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        session = std::move(r).value();
        return *session;
    }
};

struct TickFrames {
    std::uint64_t tick = 0;
    SimTimeUs simTimeUs = 0;
    std::vector<VisionFrame> frames;
};

// Ticks with ONE reused SensorFrame, like a caller that does not clear it.
std::vector<TickFrames> run(SimulationSession& s, int ticks) {
    std::vector<TickFrames> out;
    proto::SensorFrame frame;
    for (int i = 0; i < ticks; ++i) {
        s.tick(frame, i == 0);
        out.push_back({frame.tick, frame.simTimeUs, frame.vision});
    }
    return out;
}

bool hasWarning(const SimulationSession& s, const char* code) {
    return std::any_of(s.sessionWarnings().begin(), s.sessionWarnings().end(),
                       [&](const HubWarning& w) { return w.code == code; });
}

}  // namespace

TEST_CASE("session vision: a project without cameras has no engine and its sensor frames carry no vision") {
    Fixture f;
    SimulationSession& s = f.make(sampleConfig());
    CHECK(s.vision() == nullptr);
    for (const TickFrames& t : run(s, 20)) CHECK(t.frames.empty());
    const auto snap = s.buildSnapshot(false, false);
    CHECK(snap->cameras().empty());
    CHECK(snap->fiducials().empty());
    CHECK(snap->detectedCameras().empty());
    CHECK_FALSE(hasWarning(s, "vision_layout_missing"));
}

TEST_CASE("session vision: disabled cameras alone neither start the engine nor ask for a layout") {
    Fixture f;
    RobotConfig c = sampleConfig();
    CameraConfig off = wallCamera();
    off.enabled = false;
    c.vision.cameras = {off};  // no layout file, no plugin layouts
    SimulationSession& s = f.make(c);
    CHECK(s.vision() == nullptr);
    CHECK_FALSE(hasWarning(s, "vision_layout_missing"));
    CHECK(run(s, 5).back().frames.empty());
}

TEST_CASE("session vision: a frame rides SensorFrame N0 + 1 + k and keeps the configured latency") {
    Fixture f;
    SimulationSession& s = f.make(f.config());
    REQUIRE(s.vision() != nullptr);
    const auto ticks = run(s, 12);
    for (std::size_t i = 0; i < ticks.size(); ++i) {
        CHECK(ticks[i].tick == i);                                      // 0-based, as before
        CHECK(ticks[i].simTimeUs == static_cast<SimTimeUs>((i + 1) * 5000));
        if (i < 6) {
            CHECK(ticks[i].frames.empty());  // the first capture (tick 0) is published in tick 0 + 1 + 5
        } else {
            REQUIRE(ticks[i].frames.size() == 1);  // a reused SensorFrame never accumulates frames
            const VisionFrame& frame = ticks[i].frames[0];
            CHECK(frame.cameraId == "front");
            CHECK(frame.sequence == i - 5);
            CHECK(frame.captureTimeUs == static_cast<SimTimeUs>((i - 5) * 5000));
            CHECK(frame.latencyUs == 25000);
            CHECK(frame.observations.size() == 2);
            // the extension's clock reads T(N-1) when it applies the frame
            CHECK(static_cast<SimTimeUs>(ticks[i].tick) * 5000 == frame.captureTimeUs + frame.latencyUs);
        }
    }
}

TEST_CASE("session vision: the capture uses the chassis pose after this tick's physics step") {
    Fixture f;
    RobotConfig c = f.config();
    c.vision.cameras[0].latencyUs = 0;
    SimulationSession& s = f.make(c);
    s.world().setLinearVelocity(s.chassisBody(), Vec3{2.f, 0.f, 0.f});  // 2 m/s toward the wall
    const auto ticks = run(s, 3);
    // tick 0 captures at T(0) = 5 ms after a 10 mm step; tick 1 is published with that capture
    REQUIRE(ticks[1].frames.size() == 1);
    const FiducialObservation& o = ticks[1].frames[0].observations[0];
    CHECK(o.distanceM == doctest::Approx(std::hypot(4.0 - 0.01, 0.5)).epsilon(1e-3));
}

TEST_CASE("session vision: the capture sees what the game step did to the robot") {
    Fixture f;
    REQUIRE(f.reg.plugins.add(std::make_unique<PushModule>()).ok());
    REQUIRE(f.reg.registrar.ensureRegistered(*f.reg.plugins.find("push_game")).ok());
    RobotConfig c = f.config();
    c.gamePluginId = "push_game";
    c.vision.cameras[0].latencyUs = 0;
    const auto ticks = run(f.make(c), 3);
    REQUIRE(ticks[1].frames.size() == 1);
    // captured in tick 0, after the game moved the robot to x = 2 m: the wall is 2 m ahead, not 4
    CHECK(ticks[1].frames[0].observations[0].distanceM < 2.2);
}

TEST_CASE("session vision: a camera on a mechanism link is posed through the session's link directory") {
    Fixture f;
    RobotConfig c = f.config();
    CameraConfig cam = wallCamera("tip");
    cam.attachment.parentMechanismId = "intake";  // mount (0.4, 0, 0.1), template link "tip" at (0, 0, 0.3)
    cam.attachment.parentLink = "tip";
    cam.local = Pose3{Vec3{0.1f, 0.f, 0.f}, Quat{}};
    cam.latencyUs = 0;
    c.vision.cameras = {cam};
    SimulationSession& s = f.make(c);
    REQUIRE(s.vision() != nullptr);
    const auto ticks = run(s, 3);
    REQUIRE(ticks[1].frames.size() == 1);
    const Pose3& mount = ticks[1].frames[0].robotToCamera;
    CHECK(mount.position.x == doctest::Approx(0.5f));  // 0.4 + 0.1
    CHECK(mount.position.z == doctest::Approx(0.4f));  // 0.1 + 0.3
    // and the handshake default is the same physical pose
    const VisionSetup setup = s.vision()->setup("");
    REQUIRE(setup.cameras.size() == 1);
    CHECK(setup.cameras[0].defaultAssumedRobotToCamera.position.x == doctest::Approx(0.5f));
    CHECK(s.buildSnapshot(false, false)->cameras()[0].status.empty());  // the attachment resolved
}

TEST_CASE("session vision: the layout is the file, else the plugin's named layout, else its default") {
    {
        Fixture f;
        auto module = std::make_unique<LayoutModule>();
        REQUIRE(f.reg.plugins.add(std::move(module)).ok());
        REQUIRE(f.reg.registrar.ensureRegistered(*f.reg.plugins.find("layout_game")).ok());
        RobotConfig c = sampleConfig();
        c.gamePluginId = "layout_game";
        c.vision.cameras = {wallCamera()};

        SimulationSession& defaultLayout = f.make(c);  // the plugin's first layout
        REQUIRE(defaultLayout.vision() != nullptr);
        CHECK(defaultLayout.vision()->layout().id == "layout_game.first");

        c.vision.layoutId = "layout_game.second";
        SimulationSession& named = f.make(c);
        REQUIRE(named.vision() != nullptr);
        CHECK(named.vision()->layout().id == "layout_game.second");
        CHECK(named.vision()->layout().tags[0].id == 22);

        c.vision.layoutFile = f.dir.write("layout.json", wallLayoutJson()).string();  // a file beats the plugin
        SimulationSession& file = f.make(c);
        REQUIRE(file.vision() != nullptr);
        CHECK(file.vision()->layout().id == "file:layout.json");
        CHECK(file.vision()->layout().tags.size() == 2);
    }
    {
        // a relative file is looked up below the project directory
        Fixture f;
        RobotConfig c = sampleConfig();
        c.project.path = f.dir.path().string();
        f.dir.write("fields/custom.json", wallLayoutJson());
        c.vision.cameras = {wallCamera()};
        c.vision.layoutFile = "fields/custom.json";
        SimulationSession& s = f.make(c);
        REQUIRE(s.vision() != nullptr);
        CHECK(s.vision()->layout().id == "file:custom.json");
    }
}

TEST_CASE("session vision: without a layout vision stays off, the session runs on and an error is raised") {
    struct Case {
        const char* name;
        std::function<void(RobotConfig&, Fixture&)> prepare;
    };
    const Case cases[] = {
        {"no file and no plugin layouts", [](RobotConfig&, Fixture&) {}},
        {"missing file", [](RobotConfig& c, Fixture& f) { c.vision.layoutFile = (f.dir.path() / "gone.json").string(); }},
        {"broken file", [](RobotConfig& c, Fixture& f) { c.vision.layoutFile = f.dir.write("bad.json", "{\"tags\":[]}").string(); }},
        {"unknown layout id", [](RobotConfig& c, Fixture&) { c.vision.layoutId = "fake_game.nope"; }},
    };
    for (const Case& test : cases) {
        INFO(test.name);
        Fixture f;
        RobotConfig c = sampleConfig();
        c.vision.cameras = {wallCamera()};
        test.prepare(c, f);
        SimulationSession& s = f.make(c);
        CHECK(s.vision() == nullptr);
        REQUIRE(hasWarning(s, "vision_layout_missing"));
        const auto w = std::find_if(s.sessionWarnings().begin(), s.sessionWarnings().end(),
                                    [](const HubWarning& x) { return x.code == "vision_layout_missing"; });
        CHECK(w->severity == Severity::Error);
        CHECK_FALSE(w->message.empty());
        for (const TickFrames& t : run(s, 10)) CHECK(t.frames.empty());  // runs as if there were no cameras
        const auto snap = s.buildSnapshot(false, false);
        CHECK(snap->cameras().empty());
        CHECK(std::any_of(snap->warnings().begin(), snap->warnings().end(),
                          [](const HubWarning& x) { return x.code == "vision_layout_missing"; }));
    }
}

TEST_CASE("session vision: the builder refuses cameras the engine cannot simulate") {
    Fixture f;
    CameraConfig bad = wallCamera();
    bad.frameRateHz = 0.0;
    const auto r = f.build(f.config({bad}));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("vision") != std::string::npos);
    CHECK(r.error().message.find("frame rate") != std::string::npos);
}

TEST_CASE("session vision: reset restarts the schedule and drops queued frames, sequences continue") {
    Fixture f;
    SimulationSession& s = f.make(f.config());
    const auto before = run(s, 8);  // captures at ticks 0..7, the first two already published
    REQUIRE(before[7].frames.size() == 1);
    CHECK(before[7].frames[0].sequence == 2);

    s.reset();
    const auto after = run(s, 8);
    for (std::size_t i = 0; i < 6; ++i) CHECK(after[i].frames.empty());  // nothing from before the reset
    REQUIRE(after[6].frames.size() == 1);
    CHECK(after[6].tick == 6);  // tick numbering restarts
    CHECK(after[6].frames[0].sequence == 9);  // eight were captured before the reset
    CHECK(after[6].frames[0].captureTimeUs == 5000);
    CHECK(after[6].frames[0].latencyUs == 25000);
}

TEST_CASE("session vision: a teleport is seen by the next capture and the schedule continues") {
    Fixture f;
    RobotConfig c = f.config();
    c.vision.cameras[0].latencyUs = 0;
    SimulationSession& s = f.make(c);
    const auto before = run(s, 4);
    REQUIRE(before[3].frames.size() == 1);
    const double farM = before[3].frames[0].observations[0].distanceM;
    CHECK(farM > 3.9);

    s.setRobotPose(Pose3{Vec3{1.f, 0.f, 0.2f}, Quat{}});
    const auto after = run(s, 3);
    // tick 4's frame is published in tick 5; the teleport applies before tick 4's capture
    REQUIRE(after.size() == 3);
    REQUIRE(after[1].frames.size() == 1);
    CHECK(after[1].frames[0].sequence == 5);
    CHECK(after[1].frames[0].observations[0].distanceM < 3.2);
}

TEST_CASE("session vision: snapshots carry the cameras and tags and place the camera on the live chassis") {
    Fixture f;
    SimulationSession& s = f.make(f.config());
    run(s, 8);
    auto snap = s.buildSnapshot(false, true);
    REQUIRE(snap->cameras().size() == 1);
    const CameraSample& cam = snap->cameras()[0];
    CHECK(cam.id == "front");
    CHECK(cam.enabled);
    CHECK(cam.framesCaptured == 8);
    CHECK(cam.framesPublished == 2);
    CHECK(cam.lastLatencyMs == doctest::Approx(25.0));
    CHECK(cam.visibleTagIds.size() == 2);
    CHECK_FALSE(cam.robotConnected);
    REQUIRE(snap->fiducials().size() == 2);
    CHECK(snap->fiducials()[0].id == 1);
    CHECK(snap->fiducials()[0].pose.position.x == doctest::Approx(4.f));
    CHECK(snap->fiducials()[0].sizeM == doctest::Approx(0.1651f));

    const Pose3 chassis = s.world().pose(s.chassisBody());
    CHECK(cam.pose.position.x == doctest::Approx(chassis.position.x));
    CHECK(cam.pose.position.z == doctest::Approx(chassis.position.z + 0.3f));

    s.setRobotPose(Pose3{Vec3{2.f, 1.f, 0.2f}, Quat::fromYaw(0.5f)});
    snap = s.buildSnapshot(false, true);
    CHECK(snap->cameras()[0].pose.position.x == doctest::Approx(2.f));
    CHECK(snap->cameras()[0].pose.position.y == doctest::Approx(1.f));

    // robot-reported state is shown next to the camera
    CameraRobotState state;
    state.ntName = "front";
    state.cameraId = "front";
    state.publishing = true;
    s.vision()->onRobotStates(std::span<const CameraRobotState>(&state, 1));
    snap = s.buildSnapshot(false, true);
    CHECK(snap->cameras()[0].robotConnected);
    REQUIRE(snap->detectedCameras().size() == 1);
    CHECK(snap->detectedCameras()[0] == state);
}

TEST_CASE("session vision: identical sessions give identical frames, and so does a run after a reset") {
    CameraConfig noisy = wallCamera("noisy");
    noisy.frameRateHz = 90.0;
    noisy.realism = {0.6, 10000, 0.1};
    CameraConfig other = wallCamera("other");
    other.ntName = "second";
    other.frameRateHz = 60.0;
    other.local = Pose3{Vec3{0.1f, 0.2f, 0.3f}, Quat::fromYaw(0.05f)};
    other.realism.cornerNoisePx = 0.3;

    Fixture a, b;
    SimulationSession& one = a.make(a.config({noisy, other}));
    SimulationSession& two = b.make(b.config({noisy, other}));
    const auto first = run(one, 400);
    const auto second = run(two, 400);
    std::size_t frames = 0;
    for (std::size_t i = 0; i < first.size(); ++i) {
        CHECK(first[i].frames == second[i].frames);
        frames += first[i].frames.size();
    }
    CHECK(frames > 100);

    // reset restores t = 0 and the RNG, so the same inputs give the same frames; only the sequence continues
    one.reset();
    const auto again = run(one, 400);
    std::uint64_t sequenceShift = 0;
    for (std::size_t i = 0; i < first.size(); ++i) {
        REQUIRE(again[i].frames.size() == first[i].frames.size());
        for (std::size_t k = 0; k < first[i].frames.size(); ++k) {
            VisionFrame x = first[i].frames[k], y = again[i].frames[k];
            if (sequenceShift == 0) sequenceShift = y.sequence - x.sequence;
            CHECK(y.sequence > x.sequence);
            y.sequence = x.sequence;
            CHECK(x == y);
        }
    }
    CHECK(sequenceShift > 0);
}

TEST_CASE("session vision: what a run produces goes through the real IPC codec unchanged") {
    CameraConfig noisy = wallCamera("noisy");
    noisy.frameRateHz = 90.0;
    noisy.realism = {0.6, 10000, 0.1};
    CameraConfig limelight = wallCamera("left");
    limelight.vendor = CameraVendor::Limelight;
    limelight.ntName = "limelight-left";
    limelight.frameRateHz = 60.0;
    Fixture f;
    SimulationSession& s = f.make(f.config({noisy, limelight}));
    const wpiemu::ipc::BinaryMessageCodec codec;

    proto::HelloAck ack;
    ack.vision = s.vision()->setup("derived-uuid");
    std::vector<std::byte> bytes;
    REQUIRE(codec.encode(ack, bytes).ok());
    auto decodedAck = codec.decode(bytes);
    REQUIRE(decodedAck.ok());
    REQUIRE(std::get_if<proto::HelloAck>(&decodedAck.value()) != nullptr);
    CHECK(std::get<proto::HelloAck>(decodedAck.value()).vision == ack.vision);

    proto::SensorFrame frame;
    std::size_t carrying = 0;
    for (int i = 0; i < 300; ++i) {
        s.tick(frame, i == 0);
        bytes.clear();
        REQUIRE(codec.encode(frame, bytes).ok());  // the encoder refuses what exceeds the caps
        auto decoded = codec.decode(bytes);
        REQUIRE(decoded.ok());
        REQUIRE(std::get_if<proto::SensorFrame>(&decoded.value()) != nullptr);
        CHECK(std::get<proto::SensorFrame>(decoded.value()).vision == frame.vision);
        if (!frame.vision.empty()) ++carrying;
    }
    CHECK(carrying > 50);
}

TEST_CASE("session vision: an ideal camera never touches the session RNG, a realistic one shares it") {
    auto gameDraws = [](std::vector<CameraConfig> cameras) {
        Fixture f;
        auto draws = std::make_shared<std::vector<std::uint64_t>>();
        REQUIRE(f.reg.plugins.add(std::make_unique<RngConsumerModule>(draws)).ok());
        REQUIRE(f.reg.registrar.ensureRegistered(*f.reg.plugins.find("rng_game")).ok());
        RobotConfig c = f.config(std::move(cameras));
        c.gamePluginId = "rng_game";
        run(f.make(c), 40);
        return *draws;
    };
    CameraConfig noisy = wallCamera("noisy");
    noisy.realism.cornerNoisePx = 0.5;

    const auto withoutCamera = gameDraws({});
    REQUIRE(withoutCamera.size() == 40);
    CHECK(gameDraws({wallCamera()}) == withoutCamera);  // enabling an ideal camera perturbs no other consumer
    CHECK(gameDraws({noisy}) != withoutCamera);         // measurement noise comes from the same stream
}

TEST_CASE("session vision: what the robot reports about its cameras is shown and warned about, and survives a reset") {
    Fixture f;
    SimulationSession& s = f.make(f.config());
    CameraRobotState refused;
    refused.ntName = "front";
    refused.cameraId = "front";
    refused.status = "photon_uuid_unsupported";
    CameraRobotState stray;
    stray.ntName = "extra";
    stray.vendor = CameraVendor::Limelight;
    const CameraRobotState states[] = {refused, stray};
    s.vision()->onRobotStates(states);

    auto codes = [&] {
        std::vector<std::pair<std::string, Severity>> out;
        const auto snap = s.buildSnapshot(false, true);
        for (const HubWarning& w : snap->warnings())
            if (w.code.rfind("vision_", 0) == 0) out.emplace_back(w.code, w.severity);
        return out;
    };
    using Codes = std::vector<std::pair<std::string, Severity>>;
    const Codes expected = {{"vision_camera_status", Severity::Error},
                            {"vision_camera_unconfigured", Severity::Warning}};
    CHECK(codes() == expected);
    CHECK(s.buildSnapshot(false, true)->detectedCameras().size() == 2);

    // the robot process outlives a simulation reset, so its report does too
    s.reset();
    CHECK(codes() == expected);
    CHECK(s.buildSnapshot(false, true)->detectedCameras().size() == 2);

    s.vision()->clearRobotStates();  // the connection is gone
    CHECK(codes().empty());
    CHECK(s.buildSnapshot(false, true)->detectedCameras().empty());
}

TEST_CASE("session vision: over the real 2026 plugin the default layout loads and sees the trench tag") {
    GamePluginRegistry plugins;
    REQUIRE(registerBuiltinGamePlugins(plugins).ok());
    MechanismFactory mechanisms;

    RobotConfig config;
    config.gamePluginId = "game_2026";
    config.sim.physics.gravity = Vec3{};  // the chassis stays where it is put
    CameraConfig cam = wallCamera("front");
    cam.local = Pose3{};
    cam.occlusion = true;  // the real field geometry is the point of this test
    cam.latencyUs = 0;
    config.vision.cameras = {cam};

    SessionParts parts;
    parts.world = wpiemu::core::physics::createBox3dWorld(config.sim.physics);
    REQUIRE(parts.world != nullptr);
    parts.config = config;
    parts.mechanisms = &mechanisms;
    parts.plugins = &plugins;
    auto built = SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    SimulationSession& s = *built.value();
    REQUIRE(s.vision() != nullptr);
    CHECK(s.vision()->layout().id == "game_2026.welded");  // the plugin's first layout
    CHECK(s.vision()->layout().tags.size() == 32);
    CHECK_FALSE(hasWarning(s, "vision_layout_missing"));

    // red zone, at the height of the trench tags, facing the low-Y trench arm: tag 7 faces +X, tag 6 faces -X
    s.setRobotPose(Pose3{Vec3{14.f, 0.644f, 0.889f}, Quat::fromYaw(3.14159265f)});
    std::set<int> seen;
    for (const TickFrames& t : run(s, 3))
        for (const VisionFrame& frame : t.frames)
            for (const FiducialObservation& o : frame.observations) seen.insert(o.id);
    CHECK(seen.count(7) == 1);
    CHECK(seen.count(6) == 0);
}
