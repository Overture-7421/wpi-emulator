// Limits on a remote robot's zone properties (kRemoteMax* in fuel_zones.h): a client chooses its shooter and pickup
// properties, so the plugin clamps them for remote robots only. The host robot's zones are never clamped.
#include <doctest/doctest.h>

#include <vector>

#include "multirobot_session_rig.h"
#include "multirobot_support.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::makeZone;
using g26test::remotePickup;
using g26test::remoteShooter;
using g26test::Roster;
using g26test::Session;
using sdk::proto::AllianceStation;

namespace {

const ShooterZoneRuntime& shooterOf(const Session& s, sdk::ParticipantId id) {
    const auto* zone = dynamic_cast<const ShooterZoneRuntime*>(s.game->robot(id)->zones.front().get());
    REQUIRE(zone != nullptr);
    return *zone;
}

const PickupZoneRuntime& pickupOf(const Session& s, sdk::ParticipantId id) {
    const auto* zone = dynamic_cast<const PickupZoneRuntime*>(s.game->robot(id)->zones.front().get());
    REQUIRE(zone != nullptr);
    return *zone;
}

sdk::Value overLimitShooter() {
    sdk::Value p = sdk::Value::makeObject();
    p["fireRateHz"] = 200.0;
    p["parallelBalls"] = 8;
    p["fixedExitSpeedMps"] = 100.0;
    p["inheritRobotVelocity"] = false;
    return p;
}

}  // namespace

TEST_CASE("remote caps: the limits are the documented numbers") {
    CHECK(kRemoteMaxFireRateHz == 25.0);
    CHECK(kRemoteMaxParallelBalls == 3);
    CHECK(kRemoteMaxIntakeRateHz == 25.0);
    CHECK(kRemoteMaxExitSpeedMps == 20.0);
    CHECK(kRemoteDefaultExitSpeedMps < kRemoteMaxExitSpeedMps);
}

TEST_CASE("remote caps: clampRemote params cap each limit and leave values at or below it alone") {
    ShooterParams over;
    over.fireRateHz = 200.0;
    over.parallelBalls = 8;
    over.fixedExitSpeedMps = 100.0;
    clampRemoteShooterParams(over);
    CHECK(over.fireRateHz == kRemoteMaxFireRateHz);
    CHECK(over.parallelBalls == kRemoteMaxParallelBalls);
    CHECK(over.fixedExitSpeedMps == kRemoteMaxExitSpeedMps);

    ShooterParams below;
    below.fireRateHz = 12.5;
    below.parallelBalls = 2;
    below.fixedExitSpeedMps = 9.0;
    clampRemoteShooterParams(below);
    CHECK(below.fireRateHz == 12.5);
    CHECK(below.parallelBalls == 2);
    CHECK(below.fixedExitSpeedMps == 9.0);

    ShooterParams atCap;
    atCap.fireRateHz = kRemoteMaxFireRateHz;
    atCap.parallelBalls = kRemoteMaxParallelBalls;
    atCap.fixedExitSpeedMps = kRemoteMaxExitSpeedMps;
    clampRemoteShooterParams(atCap);
    CHECK(atCap.fireRateHz == kRemoteMaxFireRateHz);
    CHECK(atCap.parallelBalls == kRemoteMaxParallelBalls);
    CHECK(atCap.fixedExitSpeedMps == kRemoteMaxExitSpeedMps);

    ShooterParams noFixed;   // 0 = use the default exit speed: stays 0
    clampRemoteShooterParams(noFixed);
    CHECK(noFixed.fixedExitSpeedMps == 0.0);

    PickupParams fast;
    fast.intakeRateHz = 500.0;
    clampRemotePickupParams(fast);
    CHECK(fast.intakeRateHz == kRemoteMaxIntakeRateHz);
    PickupParams unlimited;
    unlimited.intakeRateHz = 0.0;
    clampRemotePickupParams(unlimited);
    CHECK(unlimited.intakeRateHz == kRemoteMaxIntakeRateHz);
    PickupParams slow;
    slow.intakeRateHz = 7.0;
    clampRemotePickupParams(slow);
    CHECK(slow.intakeRateHz == 7.0);
}

TEST_CASE("remote caps: a remote robot's zones are clamped and the host's are not") {
    Session s;
    g26test::emptyField(s, 100, 50);
    s.addZone(makeZone("hostShooter", kShooterKind, overLimitShooter()));
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", overLimitShooter())}, {4.f, 3.f, 0.f});
    sdk::Value fastIntake = sdk::Value::makeObject();
    fastIntake["intakeRateHz"] = 500.0;
    roster.add(2, AllianceStation::Blue1, {makeZone("pickup", kPickupKind, fastIntake)}, {10.f, 3.f, 0.f});
    roster.add(3, AllianceStation::Blue2, {remotePickup("pickup", 0.0)}, {12.f, 3.f, 0.f});
    roster.setFlags(1, 1);
    s.tick(2);

    const ShooterZoneRuntime& remote = shooterOf(s, 1);
    CHECK(remote.params().fireRateHz == kRemoteMaxFireRateHz);
    CHECK(remote.params().parallelBalls == kRemoteMaxParallelBalls);
    CHECK(remote.params().fixedExitSpeedMps == kRemoteMaxExitSpeedMps);
    CHECK(remote.exitSpeedMps() == kRemoteMaxExitSpeedMps);
    CHECK(remote.solution().parallelBalls == kRemoteMaxParallelBalls);
    CHECK(remote.solution().exitSpeedMps == kRemoteMaxExitSpeedMps);

    const ShooterZoneRuntime& host = shooterOf(s, sdk::match::kHostParticipantId);
    CHECK(host.params().fireRateHz == 200.0);
    CHECK(host.params().parallelBalls == 8);
    CHECK(host.params().fixedExitSpeedMps == 100.0);
    CHECK(host.exitSpeedMps() == 100.0);

    CHECK(pickupOf(s, 2).params().intakeRateHz == kRemoteMaxIntakeRateHz);
    CHECK(pickupOf(s, 3).params().intakeRateHz == kRemoteMaxIntakeRateHz);   // "unlimited" is not allowed remotely
}

TEST_CASE("remote caps: values below the limits and an unlimited host pickup are untouched") {
    Session s;
    g26test::emptyField(s);
    sdk::Value unlimited = sdk::Value::makeObject();
    unlimited["intakeRateHz"] = 0.0;
    s.addZone(makeZone("hostPickup", kPickupKind, unlimited));
    s.start();
    Roster roster(s);
    sdk::Value modest = sdk::Value::makeObject();
    modest["fireRateHz"] = 12.5;
    modest["parallelBalls"] = 2;
    modest["fixedExitSpeedMps"] = 9.0;
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", modest)}, {4.f, 3.f, 0.f});
    roster.add(2, AllianceStation::Blue1, {remotePickup("pickup", 7.0)}, {10.f, 3.f, 0.f});
    roster.setFlags(1, 1);
    s.tick(2);

    const ShooterZoneRuntime& shooter = shooterOf(s, 1);
    CHECK(shooter.params().fireRateHz == 12.5);
    CHECK(shooter.params().parallelBalls == 2);
    CHECK(shooter.exitSpeedMps() == 9.0);
    CHECK(pickupOf(s, 2).params().intakeRateHz == 7.0);
    CHECK(pickupOf(s, sdk::match::kHostParticipantId).params().intakeRateHz == 0.0);
}

TEST_CASE("remote caps: a reset rebuilds the remote zones with the same limits") {
    Session s;
    g26test::emptyField(s);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", overLimitShooter())}, {4.f, 3.f, 0.f});
    s.stateOwner->reset(s.env.ctx);
    CHECK(shooterOf(s, 1).params().fireRateHz == kRemoteMaxFireRateHz);
    CHECK(shooterOf(s, 1).params().parallelBalls == kRemoteMaxParallelBalls);
    CHECK(shooterOf(s, 1).params().fixedExitSpeedMps == kRemoteMaxExitSpeedMps);
}

TEST_CASE("remote caps: a remote shooter at its maximum rate launches no more than the cap per second") {
    Session s;
    g26test::emptyField(s, 500, 500);
    s.addZone(makeZone("hostShooter", kShooterKind, overLimitShooter()));
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", overLimitShooter())}, {4.f, 3.f, 0.f});
    roster.setFlags(1, 1);

    const int ticksPerSecond = static_cast<int>(1.0 / s.env.clock.fixedStepSeconds() + 0.5);
    REQUIRE(ticksPerSecond > 0);
    // A volley is paid for before it leaves, so a window can hold at most one primed volley more than rate x time.
    const int burst = kRemoteMaxParallelBalls;
    int previous = 0;
    for (int second = 1; second <= 4; ++second) {
        s.tick(ticksPerSecond);
        const int shots = s.game->robot(1)->shots;
        const int inWindow = shots - previous;
        previous = shots;
        CHECK(inWindow <= static_cast<int>(kRemoteMaxFireRateHz) + burst);
        CHECK(inWindow >= static_cast<int>(kRemoteMaxFireRateHz) - burst);   // and it does fire at the cap
        CHECK(shots <= static_cast<int>(kRemoteMaxFireRateHz) * second + burst);
    }
    CHECK(s.game->robot(1)->meter == 500 - s.game->robot(1)->shots);   // spends only what it fires

    // The host, with the same properties, is not limited.
    CHECK(s.game->robot(sdk::match::kHostParticipantId)->shots > 4 * static_cast<int>(kRemoteMaxFireRateHz) + burst);
}

namespace {

using g26session::Options;
using g26session::Rig;

struct CapOutcome {
    std::uint64_t hash = 0;
    int fuelOnField = 0;
    int held = 0;
    int shots = 0;
    bool operator==(const CapOutcome& o) const {
        return hash == o.hash && fuelOnField == o.fuelOnField && held == o.held && shots == o.shots;
    }
};

sdk::RobotConfig overLimitRobot() {
    sdk::RobotConfig spec = Rig::fuelRobot();
    sdk::Value& props = spec.zones[1].properties;
    props["fireRateHz"] = 200.0;
    props["parallelBalls"] = 8;
    props["fixedExitSpeedMps"] = 100.0;
    return spec;
}

CapOutcome overLimitScenario(int workerCount, int seconds) {
    Options options;
    options.workerCount = workerCount;
    options.neutralFuel = 0;
    options.depotFuel = 0;
    options.pluginInts = {{"capacity", 500}, {"preloadFuel", 500}};
    Rig rig(options);
    rig.add(1, AllianceStation::Red2, overLimitRobot());
    rig.teleport(1, {dim::kFieldLengthM * 0.5f, dim::kFieldWidthM * 0.5f, 0.04f});
    const int ticks = static_cast<int>(static_cast<double>(seconds) / rig.session->clock().fixedStepSeconds() + 0.5);
    rig.run(ticks, {{1, 0b10}});
    const auto snap = rig.snapshot();
    const sdk::RobotSample* sample = Rig::sampleOf(*snap, 1);
    REQUIRE(sample != nullptr);
    CapOutcome out;
    out.hash = rig.session->world().stateHash();
    out.fuelOnField = Rig::fuelOnField(*snap);
    out.held = Rig::hudInt(*sample, hud::kFuelCount);
    out.shots = Rig::hudInt(*sample, hud::kShots);
    return out;
}

}  // namespace

TEST_CASE("remote caps e2e: an over-limit remote shooter is limited and identical for 1, 2 and 4 physics workers") {
    const int seconds = 3;
    const CapOutcome reference = overLimitScenario(1, seconds);
    CHECK(reference.shots > 0);
    CHECK(reference.shots <= static_cast<int>(kRemoteMaxFireRateHz) * seconds + kRemoteMaxParallelBalls);
    CHECK(reference.shots >= static_cast<int>(kRemoteMaxFireRateHz) * seconds - 2 * kRemoteMaxParallelBalls);
    CHECK(reference.held == 500 - reference.shots);
    CHECK(overLimitScenario(1, seconds) == reference);
    CHECK(overLimitScenario(2, seconds) == reference);
    CHECK(overLimitScenario(4, seconds) == reference);
}
