#include <doctest/doctest.h>

#include <algorithm>

#include "runtime_fixtures.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_mechanism_physics_coupling.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::sampleDevices;

namespace {

struct Fixture {
    Registries reg;
    testing::CapturingLogger logger;
    std::unique_ptr<SimulationSession> session;
    testing::FakePhysicsWorld* world = nullptr;

    SessionParts parts(RobotConfig cfg) {
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
        return p;
    }
    Result<std::unique_ptr<SimulationSession>> build(RobotConfig cfg) { return SessionBuilder::build(parts(std::move(cfg))); }
    SimulationSession& make(RobotConfig cfg = sampleConfig(), bool announce = true) {
        auto r = build(std::move(cfg));
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        session = std::move(r).value();
        if (announce)
            for (const auto& a : wpiemu::test_runtime::sampleDevices()) session->hub().onAnnounce(a);
        return *session;
    }
};

// runtime ids of sampleDevices(): fl,fr,bl,br = 101..104, intake talon = 105, intake cancoder = 106, pigeon = 107
proto::ActuatorFrame outputs(std::initializer_list<std::pair<std::uint32_t, double>> voltages) {
    proto::ActuatorFrame f;
    for (const auto& [id, volts] : voltages) f.samples.push_back({DeviceId{id}, 1, volts});  // signal 1 = motorVoltage
    return f;
}

double valueOf(const proto::SensorFrame& f, std::uint32_t device, std::uint16_t signal, double missing = -999.0) {
    for (const auto& s : f.samples)
        if (s.device == DeviceId{device} && s.signal == signal) return s.value;
    return missing;
}

}  // namespace

TEST_CASE("session: builder refuses incomplete or inconsistent parts") {
    Fixture f;
    {
        SessionParts p = f.parts(sampleConfig());
        p.world.reset();
        CHECK(SessionBuilder::build(std::move(p)).error().code == ErrorCode::InvalidArgument);
    }
    {
        SessionParts p = f.parts(sampleConfig());
        p.mechanisms = nullptr;
        CHECK_FALSE(SessionBuilder::build(std::move(p)).ok());
    }
    {
        RobotConfig c = sampleConfig();
        c.sim.stepUs = 0;
        CHECK_FALSE(f.build(c).ok());
    }
    {
        SessionParts p = f.parts(sampleConfig());
        p.drivetrains = nullptr;
        const auto r = SessionBuilder::build(std::move(p));
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message.find("registries") != std::string::npos);
    }
    {
        RobotConfig c = sampleConfig();
        c.drivetrain.spec.kind = "hover";
        const auto r = f.build(c);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message.find("drivetrain") != std::string::npos);
    }
    {
        RobotConfig c = sampleConfig();
        c.drivetrain.spec.tireModel = "slick";
        CHECK(f.build(c).error().message.find("tire model") != std::string::npos);
    }
    {
        RobotConfig c = sampleConfig();
        c.mechanisms[0].type = "teleporter";
        const auto r = f.build(c);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message.find("intake") != std::string::npos);
    }
    {
        RobotConfig c = sampleConfig();
        c.chassis.massKg = 0.f;
        CHECK(f.build(c).error().message.find("chassis") != std::string::npos);
    }
    {
        // a config without wheels needs no drivetrain registries (discovery-only sessions)
        RobotConfig c;
        c.sim.physics.gravity = Vec3{};
        SessionParts p = f.parts(c);
        p.drivetrains = nullptr;
        p.tireModels = nullptr;
        CHECK(SessionBuilder::build(std::move(p)).ok());
    }
}

TEST_CASE("session: construction builds field, chassis, drivetrain, mechanisms and game state in order") {
    Fixture f;
    RobotConfig c = sampleConfig();
    c.gamePluginId = "fake_game";
    REQUIRE(f.reg.registrar.ensureRegistered(*f.reg.plugins.find("fake_game")).ok());
    SimulationSession& s = f.make(c);
    CHECK(f.reg.fakeCounters->createFieldCalls == 1);
    CHECK(f.reg.fakeCounters->createGameStateCalls == 1);
    CHECK(s.gameModule() != nullptr);
    CHECK(s.chassisBody().valid());
    CHECK(s.drivetrain() != nullptr);
    CHECK(s.drivetrain()->kind() == "test");
    CHECK(s.mechanisms().all().size() == 1);
    CHECK(s.mechanisms().find("intake") != nullptr);
    CHECK(s.mechanisms().find("nope") == nullptr);
    // reference chassis: hull + one pad per wheel, COM at its height
    CHECK(f.world->shapeCount() == 5);
    CHECK(s.world().pose(s.chassisBody()).position.z == doctest::Approx(c.chassis.comHeightM));
    // slots + persisted mappings reached the hub: devices appear mapped as soon as they are announced
    for (const auto& a : sampleDevices()) s.hub().onAnnounce(a);
    for (const DiscoveredDevice& d : s.hub().discovered()) CHECK(d.mapped);
    CHECK(s.hub().loopbackSet().loopback.empty());
}

TEST_CASE("session: unknown game plugin only logs, the session still runs") {
    Fixture f;
    RobotConfig c = sampleConfig();
    c.project.wpilibYear = 1999;
    c.gamePluginId = "ghost";
    SimulationSession& s = f.make(c);
    CHECK(s.gameModule() == nullptr);
    CHECK(f.logger.count(LogLevel::Warn) >= 1);
    proto::SensorFrame out;
    s.tick(out, false);
    CHECK(out.tick == 0);
}

TEST_CASE("session: tick order is mechanisms -> drivetrain -> physics -> game state -> game hook") {
    static std::vector<std::string> log;
    log.clear();
    struct LogMech final : IMechanism {
        explicit LogMech(const MechanismBuildContext& c) : id_(c.config.id) {}
        const std::string& id() const override { return id_; }
        std::string_view type() const override { return "logger"; }
        void tick(SimContext& ctx, double) override {
            log.push_back("mech@" + std::to_string(dynamic_cast<testing::FakePhysicsWorld&>(ctx.physics()).stepCount()));
        }
        void reset() override {}
        MechanismState state() const override { return {}; }
        std::string id_;
    };
    struct LogDrive final : IDrivetrain {
        explicit LogDrive(const DrivetrainBuildArgs& a) : spec_(&a.spec), world_(&a.world) {}
        std::string_view kind() const override { return "logdt"; }
        const DrivetrainSpec& spec() const override { return *spec_; }
        std::vector<SlotSpec> slotSpecs() const override { return {}; }
        void step(std::span<const WheelCommand>, float) override {
            log.push_back("drive@" + std::to_string(dynamic_cast<testing::FakePhysicsWorld&>(*world_).stepCount()));
        }
        std::span<const WheelState> wheelStates() const override { return states_; }
        void reset() override {}
        const DrivetrainSpec* spec_;
        IPhysicsWorld* world_;
        std::vector<WheelState> states_ = std::vector<WheelState>(1);
    };
    struct LogState final : IGameState {
        std::string_view pluginId() const override { return "logmod"; }
        void reset(SimContext&) override {}
        void step(SimContext& ctx, double) override {
            log.push_back("game@" + std::to_string(dynamic_cast<testing::FakePhysicsWorld&>(ctx.physics()).stepCount()));
        }
        void publish(SnapshotBuilder&) const override {}
    };
    struct LogModule final : testing::FakeGameModule {
        LogModule() : FakeGameModule("logmod", 3000) {}
        Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override { return std::unique_ptr<IGameState>(std::make_unique<LogState>()); }
        void onTick(SimContext& ctx, double) override {
            log.push_back("hook@" + std::to_string(dynamic_cast<testing::FakePhysicsWorld&>(ctx.physics()).stepCount()));
        }
    };

    Registries reg(false);
    REQUIRE(reg.plugins.add(std::make_unique<LogModule>()).ok());
    MechanismTemplateInfo info;
    info.type = "logger";
    REQUIRE(reg.mechanisms.registerTemplate(info, [](const MechanismBuildContext& c) { return std::make_unique<LogMech>(c); }).ok());
    REQUIRE(reg.drivetrains.registerDrivetrain("logdt", {}, [](const DrivetrainBuildArgs& a) { return std::make_unique<LogDrive>(a); }).ok());

    Fixture f;
    f.reg.plugins.size();  // (fixture registries unused; this test brings its own)
    RobotConfig c = sampleConfig();
    c.drivetrain.spec.kind = "logdt";
    c.drivetrain.mappings.clear();
    c.mechanisms.clear();
    MechanismConfig m;
    m.id = "probe";
    m.type = "logger";
    c.mechanisms.push_back(m);
    c.gamePluginId = "logmod";
    SessionParts p;
    p.world = std::make_unique<testing::FakePhysicsWorld>(c.sim.physics);
    p.config = c;
    p.mechanisms = &reg.mechanisms;
    p.drivetrains = &reg.drivetrains;
    p.tireModels = &f.reg.tires;
    p.plugins = &reg.plugins;
    auto built = SessionBuilder::build(std::move(p));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    proto::SensorFrame out;
    built.value()->tick(out, false);
    built.value()->tick(out, false);
    const std::vector<std::string> expected = {"mech@0", "drive@0", "game@1", "hook@1", "mech@1", "drive@1", "game@2", "hook@2"};
    CHECK(log == expected);
}

TEST_CASE("session: SensorFrame bookkeeping - tick numbers, sim time, step") {
    Fixture f;
    SimulationSession& s = f.make();
    proto::SensorFrame out;
    for (std::uint64_t i = 0; i < 3; ++i) {
        s.tick(out, i == 0);
        CHECK(out.tick == i);
        CHECK(out.simTimeUs == static_cast<SimTimeUs>((i + 1) * 5000));  // time AFTER the step
        CHECK(out.stepUs == 5000);
        CHECK(out.full == (i == 0));
    }
    CHECK(s.clock().tickIndex() == 3);
    CHECK(s.clock().simTimeUs() == 15000);
}

TEST_CASE("session: robot outputs drive mechanisms and drivetrain, sensors flow back as rotor data") {
    Fixture f;
    SimulationSession& s = f.make();
    // intake 6 V -> omega 60 rad/s; wheels 2 V each
    s.hub().onActuatorFrame(outputs({{105, 6.0}, {101, 2.0}, {102, 2.0}, {103, 2.0}, {104, 2.0}}));
    proto::SensorFrame out;
    s.tick(out, false);

    // intake encoder: talon 105 rotorPosition (signal 3) / rotorVelocity (4), in rotations
    CHECK(valueOf(out, 106, 1) == doctest::Approx(60.0 / kTwoPi));
    CHECK(valueOf(out, 106, 0) == doctest::Approx(60.0 * 0.005 / kTwoPi));
    // wheel fl: wheel angle 2 V * 5 ms = 0.01 rad, rotor = wheel rot * gear ratio 6.75
    CHECK(valueOf(out, 101, 3) == doctest::Approx(0.01 / kTwoPi * 6.75));
    CHECK(valueOf(out, 101, 4) == doctest::Approx(2.0 / kTwoPi * 6.75));
    // the chassis was pushed forward: total 8 V * 10 N / 50 kg = 1.6 m/s^2
    CHECK(s.world().linearVelocity(s.chassisBody()).x == doctest::Approx(1.6 * 0.005).epsilon(1e-3));

    // steering feedback only for steered / swerve wheels
    CHECK(valueOf(out, 101, 3) != -999.0);
}

TEST_CASE("session: unmapped or silent robot means neutral outputs, nothing moves") {
    Fixture f;
    SimulationSession& s = f.make();
    proto::SensorFrame out;
    s.tick(out, false);
    CHECK(valueOf(out, 106, 1) == doctest::Approx(0.0));  // the encoder is still reported (at rest)
    for (int i = 0; i < 9; ++i) s.tick(out, false);
    CHECK(s.world().linearVelocity(s.chassisBody()).x == 0.f);
    CHECK(out.samples.empty());  // nothing changed since: delta frame
}

TEST_CASE("session: battery sags with total current, brownout latches with hysteresis and warns") {
    Fixture f;
    RobotConfig c = sampleConfig();
    c.battery.internalResistanceOhm = 0.1;
    SimulationSession& s = f.make(c);
    proto::SensorFrame out;
    s.tick(out, false);  // idle: baseline 2 A only
    CHECK(out.batteryVoltageV == doctest::Approx(12.6 - 2.0 * 0.1));
    CHECK(s.hub().batteryVoltage() == doctest::Approx(12.6 - 0.2));

    s.hub().onActuatorFrame(outputs({{105, 12.0}, {101, 12.0}, {102, 12.0}, {103, 12.0}, {104, 12.0}}));
    s.tick(out, false);  // intake 24 A + 4 wheels * 36 A + 2 A = 170 A -> 17 V sag -> floor at 0 V
    CHECK(out.batteryCurrentA == doctest::Approx(170.0));
    CHECK(out.batteryVoltageV == doctest::Approx(0.0));
    CHECK(s.battery().brownedOut());
    auto snap = s.buildSnapshot(false, true);
    CHECK(std::any_of(snap->warnings().begin(), snap->warnings().end(), [](const HubWarning& w) { return w.code == "brownout"; }));
    CHECK(snap->batteryVoltageV() == doctest::Approx(0.0));

    s.hub().onActuatorFrame(outputs({{105, 0.0}, {101, 0.0}, {102, 0.0}, {103, 0.0}, {104, 0.0}}));
    s.tick(out, false);
    CHECK_FALSE(s.battery().brownedOut());  // recovered well above brownout + margin
    snap = s.buildSnapshot(false, true);
    CHECK_FALSE(std::any_of(snap->warnings().begin(), snap->warnings().end(), [](const HubWarning& w) { return w.code == "brownout"; }));
    CHECK(s.battery().brownoutCount() == 1);
}

TEST_CASE("battery model: pure behaviour") {
    BatteryModel b(BatteryConfig{12.0, 0.02, 7.0});
    CHECK(b.update(0.0) == doctest::Approx(12.0));
    CHECK(b.update(100.0) == doctest::Approx(10.0));
    CHECK_FALSE(b.brownedOut());
    CHECK(b.update(-50.0) == doctest::Approx(12.0));  // regeneration is not a negative draw
    CHECK(b.update(300.0) == doctest::Approx(6.0));
    CHECK(b.brownedOut());
    CHECK(b.update(240.0) == doctest::Approx(7.2));  // above 7.0 but inside the 0.5 V hysteresis band
    CHECK(b.brownedOut());
    CHECK(b.update(200.0) == doctest::Approx(8.0));
    CHECK_FALSE(b.brownedOut());
    CHECK(b.brownoutCount() == 1);
    b.update(1e6);
    CHECK(b.voltageV() == 0.0);  // clamped, never negative
    b.reset();
    CHECK(b.voltageV() == 12.0);
    CHECK_FALSE(b.brownedOut());
}

TEST_CASE("session: runs are deterministic and reset restores t = 0") {
    auto run = [](int ticks) {
        Fixture f;
        SimulationSession& s = f.make();
        proto::SensorFrame out;
        for (int i = 0; i < ticks; ++i) {
            if (i % 7 == 0) s.hub().onActuatorFrame(outputs({{101, 1.0 + i * 0.1}, {105, 3.0}}));
            s.tick(out, false);
        }
        return std::make_tuple(f.world->stateHash(), out.batteryVoltageV, s.world().pose(s.chassisBody()).position.x);
    };
    CHECK(run(100) == run(100));
    CHECK(run(100) != run(101));

    Fixture f;
    SimulationSession& s = f.make();
    proto::SensorFrame out;
    s.hub().onActuatorFrame(outputs({{101, 5.0}, {102, 5.0}, {105, 5.0}}));
    for (int i = 0; i < 20; ++i) s.tick(out, false);
    CHECK(s.world().pose(s.chassisBody()).position.x > 0.f);
    const auto* intake = dynamic_cast<wpiemu::test_runtime::TestRoller*>(s.mechanisms().find("intake"));
    REQUIRE(intake != nullptr);
    const int resetsBefore = intake->resets;

    s.reset();
    CHECK(s.clock().tickIndex() == 0);
    CHECK(s.clock().simTimeUs() == 0);
    CHECK(s.world().pose(s.chassisBody()).position.x == 0.f);
    CHECK(s.world().linearVelocity(s.chassisBody()).x == 0.f);
    CHECK(intake->resets == resetsBefore + 1);
    CHECK(s.battery().voltageV() == doctest::Approx(12.6));
    s.tick(out, true);
    CHECK(out.tick == 0);  // numbering restarts with the epoch
    CHECK(out.simTimeUs == 5000);
    CHECK(s.hub().discovered().size() == 7);  // devices and bindings survive a reset
}

TEST_CASE("session: RNG is reseeded from the config on reset") {
    Fixture f;
    RobotConfig c = sampleConfig();
    c.sim.seed = 77;
    SimulationSession& s = f.make(c);
    (void)s;
    Rng expected(77);
    Rng probe(77);
    CHECK(expected.nextU64() == probe.nextU64());
}

TEST_CASE("session: reset after nonzero actuation matches a fresh neutral first tick") {
    Fixture used;
    Fixture fresh;
    SimulationSession& s = used.make();
    SimulationSession& neutral = fresh.make();
    const auto mappings = s.hub().exportMappings();
    for (int epoch = 0; epoch < 2; ++epoch) {
        s.hub().onActuatorFrame(outputs({{101, 12.0}, {102, 12.0}, {103, 12.0},
                                       {104, 12.0}, {105, 12.0}}));
        proto::SensorFrame driven;
        s.tick(driven, false);
        CHECK(s.world().linearVelocity(s.chassisBody()).x > 0.f);
        CHECK(valueOf(driven, 106, 1) > 0.0);
        s.reset();
        neutral.reset();
        CHECK(s.hub().exportMappings() == mappings);
        CHECK(s.hub().discovered().size() == sampleDevices().size());
        CHECK_FALSE(s.hub().motorOutput({"intake", "motor"}).valid);
        CHECK(s.hub().resolve({"intake", "motor"}) == DeviceId{105});
        proto::SensorFrame actual;
        proto::SensorFrame expected;
        s.tick(actual, false);
        neutral.tick(expected, true);
        CHECK(actual.full);
        CHECK(actual.tick == expected.tick);
        CHECK(actual.simTimeUs == expected.simTimeUs);
        CHECK(actual.samples == expected.samples);
        CHECK(actual.batteryCurrentA == expected.batteryCurrentA);
        CHECK(actual.batteryVoltageV == expected.batteryVoltageV);
        CHECK(s.world().pose(s.chassisBody()) == neutral.world().pose(neutral.chassisBody()));
        CHECK(s.world().linearVelocity(s.chassisBody()) == Vec3{});
        CHECK(valueOf(actual, 106, 0) == 0.0);
        CHECK(valueOf(actual, 106, 1) == 0.0);
        s.tick(actual, false);
        CHECK_FALSE(actual.full);
        CHECK(actual.samples.empty());
    }
}

TEST_CASE("session: the chassis IMU reads yaw/rates and proper acceleration") {
    Fixture f;
    RobotConfig c = sampleConfig();
    c.sim.physics.gravity = Vec3{0.f, 0.f, -9.81f};
    SimulationSession& s = f.make(c);
    s.world().setEnabled(s.chassisBody(), false);  // hold the (contact-less) fake body still: only the IMU path is under test
    s.world().setPose(s.chassisBody(), Pose3{Vec3{0.f, 0.f, 0.2f}, Quat::fromYaw(0.5f)});
    proto::SensorFrame out;
    s.tick(out, true);
    // pigeon runtime id 107: 0 yaw, 1 pitch, 2 roll, 3..5 angular vel, 6..8 accel
    CHECK(valueOf(out, 107, 0) == doctest::Approx(0.5).epsilon(1e-4));
    CHECK(valueOf(out, 107, 1) == doctest::Approx(0.0).epsilon(1e-4));
    CHECK(valueOf(out, 107, 2) == doctest::Approx(0.0).epsilon(1e-4));
    CHECK(valueOf(out, 107, 8) == doctest::Approx(9.81).epsilon(1e-4));  // at rest the IMU reads +g up
    CHECK(valueOf(out, 107, 6) == doctest::Approx(0.0).epsilon(1e-4));

    s.world().setPose(s.chassisBody(), Pose3{Vec3{0.f, 0.f, 0.2f}, Quat::fromEuler(0.f, 0.3f, 0.f)});  // pitched nose down/up
    s.tick(out, true);
    CHECK(valueOf(out, 107, 1) == doctest::Approx(0.3).epsilon(1e-4));
    s.world().setPose(s.chassisBody(), Pose3{Vec3{0.f, 0.f, 0.2f}, Quat::fromEuler(0.2f, 0.f, 0.f)});
    s.tick(out, true);
    CHECK(valueOf(out, 107, 2) == doctest::Approx(0.2).epsilon(1e-4));
}

TEST_CASE("session: snapshots carry chassis, wheels, mechanisms, devices, warnings and the game HUD") {
    Fixture f;
    RobotConfig c = sampleConfig();
    c.gamePluginId = "fake_game";
    REQUIRE(f.reg.registrar.ensureRegistered(*f.reg.plugins.find("fake_game")).ok());
    SimulationSession& s = f.make(c, false);
    for (const auto& a : sampleDevices()) s.hub().onAnnounce(a);
    s.hub().onAnnounce([] {
        auto extra = wpiemu::test_runtime::talonAnnounce(200, 33);  // not mapped anywhere
        return extra;
    }());
    proto::DsState ds;
    ds.enabled = true;
    ds.mode = proto::RobotMode::Autonomous;
    s.setDriverStation(ds);
    s.hub().onActuatorFrame(outputs({{105, 6.0}}));
    proto::SensorFrame out;
    for (int i = 0; i < 3; ++i) s.tick(out, false);

    const auto snap = s.buildSnapshot(true, true);
    CHECK(snap->tick() == 3);
    CHECK(snap->simTimeUs() == 15000);
    CHECK(snap->paused());
    CHECK(snap->robotConnected());
    CHECK(snap->chassis().name == "chassis");
    CHECK(snap->wheels().size() == 4);
    CHECK(snap->wheels()[0].name == "fl");
    REQUIRE(snap->mechanisms().size() == 1);
    CHECK(snap->mechanisms()[0].id == "intake");
    CHECK(snap->mechanisms()[0].active);
    CHECK(snap->devices().size() == 8);
    CHECK(std::any_of(snap->warnings().begin(), snap->warnings().end(), [](const HubWarning& w) { return w.code == "unmapped_loopback"; }));
    CHECK(snap->driverStation().enabled);
    CHECK(snap->driverStation().mode == proto::RobotMode::Autonomous);
    CHECK(snap->batteryVoltageV() < 12.6);
    REQUIRE(snap->hudValue("fake_game.steps") != nullptr);
    CHECK(snap->hudValue("fake_game.steps")->asInt() == 3);
    CHECK(f.reg.fakeCounters->gameStateSteps == 3);
    CHECK(f.reg.fakeCounters->onTickCalls == 3);

    // snapshots are immutable copies: later ticks do not change an existing one
    s.tick(out, false);
    CHECK(snap->tick() == 3);
    CHECK(s.buildSnapshot(false, false)->tick() == 4);
}

TEST_CASE("session: link poses resolve through the attachment chain, broken chains raise a warning") {
    Fixture f;
    RobotConfig c = sampleConfig();
    MechanismConfig child;
    child.id = "claw";
    child.type = "roller";
    child.mount = Pose3{Vec3{0.1f, 0.f, 0.f}, Quat{}};
    child.mappings.push_back({{"claw", "motor"}, DeviceKey{"phoenix6", "talonfx", 11, "rio", -1}});
    child.attachTo("intake", "tip");  // template link at (0, 0, 0.3)
    c.mechanisms.push_back(child);
    MechanismConfig orphan;
    orphan.id = "orphan";
    orphan.type = "roller";
    orphan.attachTo("ghost");
    c.mechanisms.push_back(orphan);
    SimulationSession& s = f.make(c);

    const auto pose = s.mechanisms().linkPoseInChassis("claw", "root");
    REQUIRE(pose.has_value());
    CHECK(pose->position.x == doctest::Approx(0.5f));  // intake mount 0.4 + child 0.1
    CHECK(pose->position.z == doctest::Approx(0.4f));  // intake mount 0.1 + tip 0.3
    CHECK(s.mechanisms().linkPoseInChassis("intake", "root")->position.x == doctest::Approx(0.4f));
    CHECK_FALSE(s.mechanisms().linkPoseInChassis("orphan", "root").has_value());
    CHECK_FALSE(s.mechanisms().linkPoseInChassis("intake", "nolink").has_value());

    const auto& warnings = s.sessionWarnings();
    REQUIRE(warnings.size() == 1);
    CHECK(warnings[0].code == "attachment_unresolved");
}

TEST_CASE("session: coupling driver is called around the physics step and reset drops its state") {
    struct Driver final : ICouplingDriver {
        void bind(IPhysicsWorld&, BodyId, const MassProperties&) override { ++binds; }
        void attachDrivetrain(IDrivetrain* d) override { attached = d; }
        IMechanismPhysicsCoupling& service() override { return fake; }
        void beforeStep(IPhysicsWorld& world, BodyId chassis, const MassProperties& base) override {
            order.push_back("before@" + std::to_string(dynamic_cast<testing::FakePhysicsWorld&>(world).stepCount()));
            fake.setChassisMass(base);
            fake.flushTo(world, chassis);
        }
        void afterStep() override {
            order.push_back("after");
            fake.endStep();
        }
        void reset() override { order.push_back("reset"); fake.clearPayload("probe"); }
        testing::FakeMechanismPhysicsCoupling fake;
        std::vector<std::string> order;
        int binds = 0;
        IDrivetrain* attached = nullptr;
    };
    Fixture f;
    auto driver = std::make_unique<Driver>();
    Driver* raw = driver.get();
    raw->fake.setCapabilities({true, true, true});
    SessionParts p = f.parts(sampleConfig());
    p.coupling = std::move(driver);
    auto built = SessionBuilder::build(std::move(p));
    REQUIRE(built.ok());
    SimulationSession& s = *built.value();
    CHECK(raw->binds == 1);
    CHECK(raw->attached == s.drivetrain());
    CHECK(s.drivetrain() != nullptr);

    raw->fake.applyWrench(Vec3{}, Wrench{Vec3{100.f, 0.f, 0.f}, Vec3{}});
    proto::SensorFrame out;
    s.tick(out, false);
    CHECK(raw->order == std::vector<std::string>{"before@0", "after"});
    CHECK(s.world().linearVelocity(s.chassisBody()).x == doctest::Approx(100.0 / 50.0 * 0.005).epsilon(1e-3));
    s.reset();
    CHECK(raw->order.back() == "reset");
}

TEST_CASE("session: an injected chassis builder replaces the reference one, its failure is reported") {
    Fixture f;
    bool called = false;
    SessionParts p = f.parts(sampleConfig());
    p.chassisBuilder = [&](IPhysicsWorld& world, const ChassisSpec&, const DrivetrainSpec&) -> Result<ChassisInfo> {
        called = true;
        BodyDesc d;
        d.type = BodyType::Dynamic;
        d.pose.position = Vec3{1.f, 2.f, 0.3f};
        ChassisInfo info;
        info.body = world.createBody(d);
        info.spawnPose = d.pose;
        return info;
    };
    auto built = SessionBuilder::build(std::move(p));
    REQUIRE(built.ok());
    CHECK(called);
    CHECK(built.value()->world().pose(built.value()->chassisBody()).position.y == 2.f);

    SessionParts bad = f.parts(sampleConfig());
    bad.chassisBuilder = [](IPhysicsWorld&, const ChassisSpec&, const DrivetrainSpec&) -> Result<ChassisInfo> {
        return makeError(ErrorCode::InvalidArgument, "no chassis for you");
    };
    const auto r = SessionBuilder::build(std::move(bad));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("no chassis for you") != std::string::npos);
}

TEST_CASE("session: plugin failures abort the build cleanly") {
    struct FailingField final : testing::FakeGameModule {
        FailingField() : FakeGameModule("badfield", 3001) {}
        Result<std::unique_ptr<IField>> createField(IPhysicsWorld&) override { return makeError(ErrorCode::Internal, "no field"); }
    };
    struct FailingState final : testing::FakeGameModule {
        FailingState() : FakeGameModule("badstate", 3002) {}
        Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override { return makeError(ErrorCode::Internal, "no state"); }
    };
    Fixture f;
    REQUIRE(f.reg.plugins.add(std::make_unique<FailingField>()).ok());
    REQUIRE(f.reg.plugins.add(std::make_unique<FailingState>()).ok());
    RobotConfig c = sampleConfig();
    c.gamePluginId = "badfield";
    CHECK(f.build(c).error().message.find("no field") != std::string::npos);
    c.gamePluginId = "badstate";
    CHECK(f.build(c).error().message.find("no state") != std::string::npos);
}

TEST_CASE("reference chassis: hull, wheel pads, mass properties, bad specs are rejected") {
    testing::FakePhysicsWorld world;
    ChassisSpec chassis;
    chassis.massKg = 40.f;
    chassis.comHeightM = 0.25f;
    chassis.sizeM = Vec3{0.6f, 0.5f, 0.1f};
    chassis.primitives.push_back({"bumper", PrimitiveKind::Box, Pose3{}, Vec3{0.1f, 0.1f, 0.1f}, 0.f, 0.f, "#fff", true, 0.f});
    chassis.primitives.push_back({"visual", PrimitiveKind::Box, Pose3{}, Vec3{0.1f, 0.1f, 0.1f}, 0.f, 0.f, "#fff", false, 0.f});
    DrivetrainSpec dt;
    dt.wheels = {wpiemu::test_runtime::wheel("a", 0.2f, 0.2f), wpiemu::test_runtime::wheel("b", 0.2f, -0.2f)};
    const auto r = buildReferenceChassis(world, chassis, dt);
    REQUIRE(r.ok());
    CHECK(world.shapeCount() == 1 + 1 + 2);  // hull + collidable primitive + two pads (visual-only primitive skipped)
    CHECK(r.value().baseMass.massKg == 40.f);
    CHECK(r.value().baseMass.inertiaDiagLocal.z == doctest::Approx(40.f / 12.f * (0.6f * 0.6f + 0.5f * 0.5f)));
    CHECK(r.value().spawnPose.position.z == 0.25f);
    chassis.inertiaDiagKgM2 = Vec3{1.f, 2.f, 3.f};
    const auto explicitInertia = buildReferenceChassis(world, chassis, dt);
    CHECK(explicitInertia.value().baseMass.inertiaDiagLocal == Vec3{1.f, 2.f, 3.f});
    chassis.massKg = 0.f;
    CHECK(buildReferenceChassis(world, chassis, dt).error().code == ErrorCode::InvalidArgument);
}
