#include <doctest/doctest.h>

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/per_robot_state.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/multi_robot_contract.h"

using namespace wpiemu;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::testing;

namespace {

// A game state that reacts to remote robots the supported way: PerRobotState, zone flags, publishRobot.
class ReactingState final : public IGameState {
public:
    explicit ReactingState(bool orderDependent) : orderDependent_(orderDependent) {}

    std::string_view pluginId() const override { return "game_reacting"; }
    void reset(SimContext&) override { robots_.forEach([](ParticipantId, int& ticks) { ticks = 0; }); }
    void step(SimContext& ctx, double) override {
        robots_.forEachRemote([&](ParticipantId id, int& ticks) {
            const RobotView* view = ctx.robots().find(id);
            if (view != nullptr && view->enabled && view->zoneFlagActive(0)) ++ticks;
        });
    }
    void publish(SnapshotBuilder& out) const override {
        // The defect under test: what is published depends on who was announced first, not on the ids.
        if (orderDependent_ && !announced_.empty()) out.setHud("game_reacting.first", static_cast<int>(announced_.front()));
    }
    void onRobotAdded(SimContext&, const RobotView& robot) override {
        if (robot.isHost) return;
        robots_.add(robot.participantId, 0);
        announced_.push_back(robot.participantId);
    }
    void onRobotRemoved(SimContext&, ParticipantId participant) override { robots_.remove(participant); }
    void publishRobot(SnapshotBuilder& out, const RobotView& robot) const override {
        if (const int* ticks = robots_.find(robot.participantId)) out.setRobotHud(robot.participantId, "game_reacting.ticks", *ticks);
    }

private:
    bool orderDependent_;
    PerRobotState<int> robots_;
    std::vector<ParticipantId> announced_;
};

class ReactingModule final : public FakeGameModule {
public:
    ReactingModule(bool declareSupport, bool orderDependent)
        : FakeGameModule("game_reacting", 2998), declareSupport_(declareSupport), orderDependent_(orderDependent) {}
    GameMetadata metadata() const override {
        GameMetadata m = FakeGameModule::metadata();
        m.supportsMultiRobot = declareSupport_;
        return m;
    }
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        return std::unique_ptr<IGameState>(std::make_unique<ReactingState>(orderDependent_));
    }

private:
    bool declareSupport_;
    bool orderDependent_;
};

// No zone kinds: nothing for a remote robot to act through.
class ZonelessModule final : public FakeGameModule {
public:
    ZonelessModule() : FakeGameModule("game_zoneless", 2997) {}
    std::vector<ZoneKindInfo> contributeZoneKinds() const override { return {}; }
};

}  // namespace

TEST_CASE("multi robot contract fails a plugin that declares zone kinds but ignores remote robots") {
    FakeGameModule ignoring("game_ignoring", 2996);  // declares game_ignoring.fake_zone, its state never reads the roster
    const std::string violation = remoteRobotContractViolation(ignoring);
    CHECK(violation.find("ignores remote robots") != std::string::npos);
    CHECK(violation.find("game_ignoring") != std::string::npos);
}

TEST_CASE("multi robot contract passes a plugin that reacts to remote robots") {
    ReactingModule reacting(true, false);
    const std::string violation = remoteRobotContractViolation(reacting);
    CHECK_MESSAGE(violation.empty(), violation);
}

TEST_CASE("multi robot contract requires a reacting plugin to declare supportsMultiRobot") {
    ReactingModule silent(false, false);
    CHECK(remoteRobotContractViolation(silent).find("supportsMultiRobot") != std::string::npos);
}

TEST_CASE("multi robot contract fails an outcome that depends on the announcement order") {
    ReactingModule orderDependent(true, true);
    CHECK(remoteRobotContractViolation(orderDependent).find("order") != std::string::npos);
}

TEST_CASE("multi robot contract has nothing to check in a plugin without zone kinds") {
    ZonelessModule zoneless;
    CHECK(remoteRobotContractViolation(zoneless).empty());
}

TEST_CASE("every registered built-in game plugin meets the multi robot contract") {
    GamePluginRegistry registry;
    REQUIRE(registerBuiltinGamePlugins(registry).ok());
    REQUIRE(registry.size() > 0);
    for (IGameModule* module : registry.list()) {
        const std::string violation = remoteRobotContractViolation(*module);
        CHECK_MESSAGE(violation.empty(), violation);
    }
}
