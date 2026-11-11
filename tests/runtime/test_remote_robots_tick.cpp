// The tick pipeline of remote robots, proven with a fake chassis and the fake world: every robot prepares before the ONE
// world step, completes after it, in participant order, and the game steps last. No remote robot ever steps the world.
#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "remote_robot_rig.h"
#include "runtime_fixtures.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace remote_test;
using proto::AllianceStation;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;

namespace {

struct Log {
    std::vector<std::string> entries;
    std::vector<std::uint64_t> tags;
};

std::uint64_t stepsOf(const IPhysicsWorld& world) { return dynamic_cast<const testing::FakePhysicsWorld&>(world).stepCount(); }

class FakeChassis final : public IRemoteChassis {
public:
    FakeChassis(IPhysicsWorld& world, Log& log, ParticipantId id, std::size_t wheels)
        : world_(world), log_(log), id_(id), states_(wheels) {
        body_ = world.createBody(BodyDesc{});
    }
    ~FakeChassis() override { world_.destroyBody(body_); }
    std::size_t wheelCount() const override { return states_.size(); }
    void setSetpoints(std::span<const WheelSetpoint>) override {}
    void brake() override {}
    void prepareStep(bool, float) override { log_.entries.push_back("prepare:" + std::to_string(id_) + "@" + std::to_string(stepsOf(world_))); }
    void completeStep(float) override { log_.entries.push_back("complete:" + std::to_string(id_) + "@" + std::to_string(stepsOf(world_))); }
    void reset(const RobotStartPose&) override {}
    BodyId body() const override { return body_; }
    std::span<const WheelState> wheelStates() const override { return states_; }

private:
    IPhysicsWorld& world_;
    Log& log_;
    ParticipantId id_;
    std::vector<WheelState> states_;
    BodyId body_;
};

struct LogDrivetrain final : IDrivetrain {
    LogDrivetrain(const DrivetrainBuildArgs& a, Log& log) : spec_(&a.spec), world_(&a.world), log_(log) {}
    std::string_view kind() const override { return "logdt"; }
    const DrivetrainSpec& spec() const override { return *spec_; }
    std::vector<SlotSpec> slotSpecs() const override { return {}; }
    void step(std::span<const WheelCommand>, float) override { log_.entries.push_back("host@" + std::to_string(stepsOf(*world_))); }
    std::span<const WheelState> wheelStates() const override { return states_; }
    void reset() override {}
    const DrivetrainSpec* spec_;
    IPhysicsWorld* world_;
    Log& log_;
    std::vector<WheelState> states_ = std::vector<WheelState>(1);
};

struct LogState final : IGameState {
    explicit LogState(Log& log) : log_(log) {}
    std::string_view pluginId() const override { return "ordermod"; }
    void reset(SimContext&) override {}
    void step(SimContext& ctx, double) override { log_.entries.push_back("game@" + std::to_string(stepsOf(ctx.physics()))); }
    void publish(SnapshotBuilder&) const override {}
    Log& log_;
};

struct LogModule final : testing::FakeGameModule {
    explicit LogModule(Log& log) : FakeGameModule("ordermod", 3001), log_(log) {}
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override { return std::unique_ptr<IGameState>(std::make_unique<LogState>(log_)); }
    Log& log_;
};

}  // namespace

TEST_CASE("remote robots: one world step per tick with every robot prepared before it and completed after it") {
    Log log;
    Registries reg(false);
    REQUIRE(reg.plugins.add(std::make_unique<LogModule>(log)).ok());
    REQUIRE(reg.drivetrains.registerDrivetrain("logdt", {}, [&log](const DrivetrainBuildArgs& a) { return std::make_unique<LogDrivetrain>(a, log); }).ok());
    REQUIRE(reg.drivetrains.registerDrivetrain(drivetrain_kind::kSwerve, {}, [&log](const DrivetrainBuildArgs& a) { return std::make_unique<LogDrivetrain>(a, log); }).ok());

    RobotConfig host = sampleConfig();
    host.drivetrain.spec.kind = "logdt";
    host.drivetrain.mappings.clear();
    host.mechanisms.clear();
    host.gamePluginId = "ordermod";
    SessionParts parts;
    parts.world = std::make_unique<testing::FakePhysicsWorld>(host.sim.physics);
    parts.config = host;
    parts.mechanisms = &reg.mechanisms;
    parts.drivetrains = &reg.drivetrains;
    parts.tireModels = &reg.tires;
    parts.plugins = &reg.plugins;
    parts.remoteChassisFactory = [&log](IPhysicsWorld& world, const RobotConfig& spec, std::uint64_t tag, const RobotStartPose&) -> Result<std::unique_ptr<IRemoteChassis>> {
        log.tags.push_back(tag);
        return std::unique_ptr<IRemoteChassis>(std::make_unique<FakeChassis>(world, log, static_cast<ParticipantId>(robotTagParticipant(tag)),
                                                                           spec.drivetrain.spec.wheels.size()));
    };
    auto built = SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    SimulationSession& session = *built.value();

    // Added out of order on purpose: the tick runs them by participant id.
    RobotConfig spec = swerveRobot();
    spec.drivetrain.spec.tireModel = "test";
    REQUIRE_MESSAGE(session.addRemoteRobot(requestFor(5, AllianceStation::Red1, spec)).ok(), "robot 5");
    REQUIRE_MESSAGE(session.addRemoteRobot(requestFor(2, AllianceStation::Blue1, spec)).ok(), "robot 2");

    // Each robot's shapes carry its own tag, defined once in robot_tags.h.
    REQUIRE(log.tags.size() == 2);
    CHECK(log.tags[0] == makeRobotTag(5));
    CHECK(log.tags[1] == makeRobotTag(2));
    CHECK(log.tags[0] != log.tags[1]);

    log.entries.clear();
    proto::SensorFrame out;
    session.tick(out, false);
    session.tick(out, false);
    const std::vector<std::string> expected = {"host@0", "prepare:2@0", "prepare:5@0", "complete:2@1", "complete:5@1", "game@1",
                                               "host@1", "prepare:2@1", "prepare:5@1", "complete:2@2", "complete:5@2", "game@2"};
    CHECK(log.entries == expected);
    CHECK(stepsOf(session.world()) == 2);
}
