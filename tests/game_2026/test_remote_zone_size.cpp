// A remote client's claimed pickup volume end to end: the spec goes through the match host's intake (acceptRemoteSpec) into the
// real SimulationSession with the 2026 plugin. A client claiming a huge volume must not collect fuel beyond the zone size
// limit; the same claim sent around the intake (the control) does, so the test can tell the limit from a missing intake.
#include <doctest/doctest.h>

#include <string>

#include "../match/match_test_support.h"
#include "multirobot_session_rig.h"
#include "test_support.h"
#include "wpiemu/core/match/spec_intake.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26session::Rig;
using sdk::proto::AllianceStation;

namespace {

constexpr float kMidX = dim::kFieldLengthM * 0.5f;

sdk::RobotConfig hugePickupRobot(float sideM) {
    sdk::RobotConfig spec = Rig::fuelRobot();
    spec.gamePluginId = std::string(pluginId());
    spec.zones[0].sizeM = {sideM, sideM, sideM};
    return spec;
}

sdk::RobotConfig acceptedByTheHost(const sdk::RobotConfig& claimed, std::vector<std::string>* clamped = nullptr) {
    match_test::PipelineKit kit;
    REQUIRE(registerPlugin(kit.plugins).ok());
    const sdk::Result<core::match::AcceptedSpec> accepted = core::match::acceptRemoteSpec(match_test::specJson(claimed), kit.pipeline);
    REQUIRE_MESSAGE(accepted.ok(), (accepted.ok() ? "" : accepted.error().message));
    if (clamped) *clamped = accepted.value().clamped;
    return accepted.value().config;
}

// Fuel held after a remote robot with `spec` sat 2 m beyond the neutral cluster's edge, facing away, with its pickup active.
// The 1.5 m volume 0.45 m ahead of the robot reaches 1.2 m from it, so it cannot touch the cluster; only a larger one can.
int fuelHeldBeyondTheCluster(const sdk::RobotConfig& spec) {
    Rig rig;
    rig.add(1, AllianceStation::Red2, spec);
    rig.run(100, {});
    sdk::Vec3 edge;
    for (const sdk::EntityGroupSample& g : rig.snapshot()->entityGroups())
        if (g.group == "fuel")
            for (const sdk::Vec3& p : g.positions)
                if (p.x > edge.x && p.x < kMidX + 2.5f) edge = p;
    rig.teleport(1, {edge.x + 2.0f, edge.y, 0.04f});
    rig.run(80, {{1, 0b01}});
    const auto snap = rig.snapshot();
    const sdk::RobotSample* sample = Rig::sampleOf(*snap, 1);
    REQUIRE(sample != nullptr);
    return Rig::hudInt(*sample, hud::kFuelCount);
}

}  // namespace

TEST_CASE("zone size e2e: a huge claimed pickup volume is clamped by the intake and collects nothing out of reach") {
    const sdk::RobotConfig claimed = hugePickupRobot(40.f);
    std::vector<std::string> clamped;
    const sdk::RobotConfig accepted = acceptedByTheHost(claimed, &clamped);

    REQUIRE(accepted.zones.size() == 2);
    CHECK(accepted.zones[0].sizeM.x == doctest::Approx(1.5f));
    CHECK(accepted.zones[0].sizeM.y == doctest::Approx(1.5f));
    CHECK(accepted.zones[0].sizeM.z == doctest::Approx(1.5f));
    bool reported = false;
    for (const std::string& line : clamped) reported = reported || line.find("zones[0].sizeM") == 0;
    CHECK(reported);

    // Control: the unclamped claim reaches the cluster from 2 m away, so the clamp is what keeps the volume out of reach.
    CHECK(fuelHeldBeyondTheCluster(claimed) > 8);
    CHECK(fuelHeldBeyondTheCluster(accepted) == 8);
}

TEST_CASE("zone size e2e: a pickup volume inside the limit is accepted unchanged") {
    const sdk::RobotConfig claimed = hugePickupRobot(0.6f);
    std::vector<std::string> clamped;
    const sdk::RobotConfig accepted = acceptedByTheHost(claimed, &clamped);
    CHECK(accepted.zones[0].sizeM.x == doctest::Approx(0.6f));
    for (const std::string& line : clamped) CHECK(line.find(".sizeM") == std::string::npos);
}
