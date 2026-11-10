#pragma once

// Fixtures of the remote robot tests: a SimulationSession over the REAL Box3D world, drivetrain and tire registries and the
// default FRC field, a recording game, remote robot specs (swerve, tank, mecanum) and a scripted client that turns chassis
// speeds into wheel setpoints with sdk::DriveKinematics and submits them as MatchInputs.

#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/runtime/physics_wiring.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/game/default_field.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/match/quantize.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"
#include "wpiemu/sdk/physics/robot_tags.h"
#include "wpiemu/sdk/testing/fake_game_module.h"

namespace remote_test {

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
namespace physics = wpiemu::core::physics;

inline constexpr float kHostStartX = 2.0f;
inline constexpr float kHostStartY = default_field::kWidthM * 0.5f;

// ---- specs ------------------------------------------------------------------------------------------------------

inline WheelSpec makeWheel(const char* name, float x, float y, WheelKind kind, bool steered) {
    WheelSpec w;
    w.name = name;
    w.kind = kind;
    w.positionM = {x, y, 0.f};   // hub at one radius
    w.steered = steered;
    w.driveCurrentLimitA = 60.f;
    return w;
}

inline RobotConfig robotWith(const char* name, DrivetrainSpec drivetrain, std::size_t zones = 0) {
    RobotConfig c;
    c.name = name;
    c.chassis.sizeM = {0.7f, 0.7f, 0.12f};
    c.chassis.massKg = 50.f;
    c.chassis.comHeightM = 0.2f;
    c.chassis.groundClearanceM = 0.03f;
    c.drivetrain.spec = std::move(drivetrain);
    for (std::size_t i = 0; i < zones; ++i) {
        ZoneConfig z;
        z.id = "zone" + std::to_string(i);
        z.kind = "recording_game.zone";
        c.zones.push_back(z);
    }
    return c;
}

inline RobotConfig swerveRobot(const char* name = "Swerve", std::size_t zones = 0) {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kSwerve;
    d.wheels = {makeWheel("fl", 0.25f, 0.25f, WheelKind::Swerve, true), makeWheel("fr", 0.25f, -0.25f, WheelKind::Swerve, true),
                makeWheel("rl", -0.25f, 0.25f, WheelKind::Swerve, true), makeWheel("rr", -0.25f, -0.25f, WheelKind::Swerve, true)};
    return robotWith(name, std::move(d), zones);
}

inline RobotConfig tankRobot(const char* name = "Tank", std::size_t zones = 0) {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kTank;
    d.wheels = {makeWheel("l0", 0.2f, 0.25f, WheelKind::Standard, false), makeWheel("l1", -0.2f, 0.25f, WheelKind::Standard, false),
                makeWheel("r0", 0.2f, -0.25f, WheelKind::Standard, false), makeWheel("r1", -0.2f, -0.25f, WheelKind::Standard, false)};
    return robotWith(name, std::move(d), zones);
}

inline RobotConfig mecanumRobot(const char* name = "Mecanum") {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kMecanum;
    d.wheels = {makeWheel("fl", 0.25f, 0.25f, WheelKind::Mecanum, false), makeWheel("fr", 0.25f, -0.25f, WheelKind::Mecanum, false),
                makeWheel("rl", -0.25f, 0.25f, WheelKind::Mecanum, false), makeWheel("rr", -0.25f, -0.25f, WheelKind::Mecanum, false)};
    return robotWith(name, std::move(d));
}

inline RemoteRobotRequest requestFor(ParticipantId id, proto::AllianceStation station, RobotConfig spec) {
    RemoteRobotRequest r;
    r.participantId = id;
    r.name = spec.name + "-" + std::to_string(id);
    r.station = station;
    r.spec = std::move(spec);
    return r;
}

// ---- game -------------------------------------------------------------------------------------------------------------

// The fake game over the default FRC field, with a state that records the multi-robot hooks.
class RecordingGameModule final : public testing::FakeGameModule {
public:
    RecordingGameModule() : FakeGameModule("recording_game", 2997) {}
    Result<std::unique_ptr<IField>> createField(IPhysicsWorld& world) override { return createDefaultField(world); }
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        auto state = std::make_unique<testing::RecordingGameState>();
        recording = state.get();
        return std::unique_ptr<IGameState>(std::move(state));
    }
    testing::RecordingGameState* recording = nullptr;   // owned by the session
};

// ---- the session -------------------------------------------------------------------------------------------------------

struct RigOptions {
    int workerCount = 1;
    bool recordingGame = false;           // the fake game (and its default field) instead of the fallback default field
    bool remoteSupport = true;            // inject the RemoteChassisFactory
    bool hostTag = true;                  // stamp makeRobotTag(host) on the host robot
    float hostX = kHostStartX;
    float hostY = kHostStartY;
};

struct Rig {
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    MechanismFactory mechanisms;
    GamePluginRegistry plugins;
    RecordingGameModule* game = nullptr;
    std::unique_ptr<SimulationSession> session;
    proto::SensorFrame frame;

    explicit Rig(const RigOptions& options = {}) {
        REQUIRE(physics::registerPhysicsComponents(drivetrains, tires).ok());
        RobotConfig host;
        host.name = "Host";
        host.sim.physics.workerCount = options.workerCount;
        SessionParts parts;
        parts.world = physics::createBox3dWorld(host.sim.physics);
        REQUIRE(parts.world != nullptr);
        if (options.recordingGame) {
            auto module = std::make_unique<RecordingGameModule>();
            game = module.get();
            REQUIRE(plugins.add(std::move(module)).ok());
            host.gamePluginId = "recording_game";
            parts.plugins = &plugins;
        }
        parts.config = host;
        parts.mechanisms = &mechanisms;
        parts.drivetrains = &drivetrains;
        parts.tireModels = &tires;
        parts.fallbackField = createDefaultField;
        const std::uint64_t hostTag = options.hostTag ? makeRobotTag(match::kHostParticipantId) : 0;
        parts.chassisBuilder = physicsChassisBuilder({options.hostX, options.hostY, 0.f}, hostTag);
        parts.coupling = std::make_unique<PhysicsCouplingDriver>();
        if (options.remoteSupport) parts.remoteChassisFactory = physicsRemoteChassisFactory(drivetrains, tires);
        auto built = SessionBuilder::build(std::move(parts));
        REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
        session = std::move(built).value();
        setEnabled(true);
    }

    void setEnabled(bool enabled, bool eStop = false) {
        proto::DsState ds = session->driverStation();
        ds.enabled = enabled;
        ds.eStop = eStop;
        session->setDriverStation(ds);
    }
    void tick(int count = 1) {
        for (int i = 0; i < count; ++i) session->tick(frame, false);
    }
    std::uint64_t nextTick() const { return session->clock().tickIndex(); }
    void add(ParticipantId id, proto::AllianceStation station, RobotConfig spec) {
        const Status s = session->addRemoteRobot(requestFor(id, station, std::move(spec)));
        REQUIRE_MESSAGE(s.ok(), (s.ok() ? "" : s.error().message));
    }
    std::uint64_t hash() const { return session->world().stateHash(); }

    // Pose, velocities of a remote robot's body via the directory.
    BodyId bodyOf(ParticipantId id) const {
        const RobotView* v = session->robots().find(id);
        REQUIRE(v != nullptr);
        return v->body;
    }
    Pose3 poseOf(ParticipantId id) const { return session->world().pose(bodyOf(id)); }
    // Linear velocity in the robot's own frame (x forward, y left).
    Vec3 bodyVelocity(ParticipantId id) const {
        const BodyId b = bodyOf(id);
        return session->world().pose(b).rotation.conjugate().rotate(session->world().linearVelocity(b));
    }
    float groundSpeed(ParticipantId id) const {
        const Vec3 v = session->world().linearVelocity(bodyOf(id));
        return std::hypot(v.x, v.y);
    }
};

// ---- scripted client ---------------------------------------------------------------------------------------------------

inline match::MatchInput inputOf(std::uint32_t seq, std::uint64_t targetTick, const std::vector<WheelSetpoint>& setpoints,
                                 std::uint32_t zoneFlags = 0) {
    match::MatchInput in;
    in.inputSeq = seq;
    in.targetTick = targetTick;
    in.zoneFlags = zoneFlags;
    for (const WheelSetpoint& sp : setpoints)
        in.wheels.push_back({sp.wheelSpeedRadPerSec, match::quantizeAngle(sp.steerAngleRad)});
    return in;
}

// What a game client does: chassis speeds -> wheel setpoints (DriveKinematics) -> MatchInput.
class ScriptedClient {
public:
    ScriptedClient(Rig& rig, ParticipantId id, const RobotConfig& spec) : rig_(rig), id_(id) {
        auto kin = DriveKinematics::create(spec.drivetrain.spec);
        REQUIRE_MESSAGE(kin.ok(), (kin.ok() ? "" : kin.error().message));
        kin_.emplace(std::move(kin).value());
    }

    std::vector<WheelSetpoint> setpointsFor(const ChassisSpeeds& speeds) {
        auto sp = kin_->update(speeds);
        REQUIRE(sp.ok());
        return std::move(sp).value();
    }
    // Submits the next sequence number for `targetTick`.
    Result<RemoteInputOutcome> send(const ChassisSpeeds& speeds, std::uint64_t targetTick, std::uint32_t zoneFlags = 0) {
        return rig_.session->submitRemoteInput(id_, inputOf(++seq_, targetTick, setpointsFor(speeds), zoneFlags));
    }
    // Runs `ticks` ticks sending a fresh input every `period` ticks, each targeting the tick it is sent on.
    void drive(const ChassisSpeeds& speeds, int ticks, int period = 4, std::uint32_t zoneFlags = 0) {
        for (int i = 0; i < ticks; ++i) {
            if (i % period == 0) REQUIRE(send(speeds, rig_.nextTick(), zoneFlags).ok());
            rig_.tick();
        }
    }
    std::uint32_t seq() const { return seq_; }

private:
    Rig& rig_;
    ParticipantId id_;
    std::optional<DriveKinematics> kin_;
    std::uint32_t seq_ = 0;
};

}  // namespace remote_test
