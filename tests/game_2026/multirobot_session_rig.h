#pragma once

// The real SimulationSession (Box3D world, drivetrain and tire registries, 2026 field and game state) with remote robots,
// for the end-to-end multi-robot tests of the 2026 plugin. tests/runtime/remote_robot_rig.h builds the same session around
// a recording fake game, so its specs, inputs and tick helpers are reused and only the game selection is new here.

#include <doctest/doctest.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../runtime/remote_robot_rig.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"
#include "wpiemu/plugins/game_2026/game_state.h"
#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"

namespace g26session {

using namespace wpiemu;
namespace rt = remote_test;

struct Options {
    int workerCount = 1;
    int neutralFuel = 360;
    int depotFuel = 24;
    std::uint64_t seed = 1;
    std::vector<sdk::ZoneConfig> hostZones;           // zones of the host robot (it has none by default)
    std::map<std::string, int> pluginInts;            // extra integer keys of the plugin section (capacity, preloadFuel...)
};

class Rig {
public:
    explicit Rig(const Options& options = {}) {
        REQUIRE(core::physics::registerPhysicsComponents(drivetrains, tires).ok());
        REQUIRE(plugins::game_2026::registerPlugin(plugins).ok());
        sdk::RobotConfig host;
        host.name = "Host";
        host.gamePluginId = std::string(plugins::game_2026::pluginId());
        host.sim.seed = options.seed;
        host.sim.physics.workerCount = options.workerCount;
        sdk::Value& section = host.pluginSection(std::string(plugins::game_2026::pluginId())).data;
        section["neutralFuel"] = options.neutralFuel;
        section["depotFuel"] = options.depotFuel;
        for (const auto& [key, value] : options.pluginInts) section[key] = value;
        host.zones = options.hostZones;
        core::runtime::SessionParts parts;
        parts.world = core::physics::createBox3dWorld(host.sim.physics);
        REQUIRE(parts.world != nullptr);
        parts.config = host;
        parts.plugins = &plugins;
        parts.mechanisms = &mechanisms;
        parts.drivetrains = &drivetrains;
        parts.tireModels = &tires;
        parts.fallbackField = sdk::createDefaultField;
        parts.chassisBuilder = core::runtime::physicsChassisBuilder({rt::kHostStartX, rt::kHostStartY, 0.f},
                                                                    sdk::makeRobotTag(sdk::match::kHostParticipantId));
        parts.coupling = std::make_unique<core::runtime::PhysicsCouplingDriver>();
        parts.remoteChassisFactory = core::runtime::physicsRemoteChassisFactory(drivetrains, tires);
        auto built = core::runtime::SessionBuilder::build(std::move(parts));
        REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
        session = std::move(built).value();
        setEnabled(true);
    }

    void setEnabled(bool enabled) {
        sdk::proto::DsState ds = session->driverStation();
        ds.enabled = enabled;
        session->setDriverStation(ds);
    }
    void setStation(sdk::proto::AllianceStation station) {
        sdk::proto::DsState ds = session->driverStation();
        ds.station = station;
        session->setDriverStation(ds);
    }

    // A swerve robot with a pickup (flag bit 0) and a shooter (flag bit 1) in the plugin's zone kinds.
    static sdk::RobotConfig fuelRobot(double fixedExitSpeedMps = 0.0) {
        sdk::RobotConfig spec = rt::swerveRobot("Fuel");
        sdk::ZoneConfig pickup;
        pickup.id = "intake";
        pickup.kind = plugins::game_2026::kPickupKind;
        pickup.local.position = {0.45f, 0.f, 0.2f};
        pickup.sizeM = {0.30f, 0.60f, 0.40f};
        pickup.properties = sdk::Value::makeObject();
        pickup.properties["intakeRateHz"] = 0.0;
        sdk::ZoneConfig shooter;
        shooter.id = "shooter";
        shooter.kind = plugins::game_2026::kShooterKind;
        shooter.local.position = {0.25f, 0.f, 0.55f};
        shooter.properties = sdk::Value::makeObject();
        shooter.properties["fixedExitSpeedMps"] = fixedExitSpeedMps;
        spec.zones = {pickup, shooter};
        return spec;
    }

    void add(sdk::ParticipantId id, sdk::proto::AllianceStation station, sdk::RobotConfig spec) {
        const sdk::Status s = session->addRemoteRobot(rt::requestFor(id, station, std::move(spec)));
        REQUIRE_MESSAGE(s.ok(), (s.ok() ? "" : s.error().message));
        sent_[id] = 0;
    }

    // Puts a remote robot at rest on the field (the session places robots near their alliance wall).
    void teleport(sdk::ParticipantId id, const sdk::Vec3& position, float yaw = 0.f) {
        const sdk::RobotView* view = session->robots().find(id);
        REQUIRE(view != nullptr);
        session->world().setPose(view->body, sdk::Pose3{position, sdk::Quat::fromYaw(yaw)});
        session->world().setLinearVelocity(view->body, sdk::Vec3{});
        session->world().setAngularVelocity(view->body, sdk::Vec3{});
    }

    // Runs `ticks` ticks; each robot in `flags` keeps sending a stationary input with its zone flags every 4 ticks.
    void run(int ticks, const std::vector<std::pair<sdk::ParticipantId, std::uint32_t>>& flags) {
        for (int i = 0; i < ticks; ++i) {
            if (i % 4 == 0)
                for (const auto& [id, bits] : flags) {
                    const auto in = rt::inputOf(++sent_[id], session->clock().tickIndex(),
                                                std::vector<sdk::WheelSetpoint>(4), bits);
                    REQUIRE(session->submitRemoteInput(id, in).ok());
                }
            session->tick(frame, false);
        }
    }

    std::shared_ptr<const sdk::ISimSnapshot> snapshot() const { return session->buildSnapshot(false, true, {}); }
    static const sdk::RobotSample* sampleOf(const sdk::ISimSnapshot& snap, sdk::ParticipantId id) {
        for (const sdk::RobotSample& r : snap.remoteRobots())
            if (r.participantId == id) return &r;
        return nullptr;
    }
    static int hudInt(const sdk::RobotSample& sample, const char* key) {
        for (const sdk::HudValue& h : sample.hud)
            if (h.key == key) return static_cast<int>(h.value.asInt());
        return -1;
    }
    static int fuelOnField(const sdk::ISimSnapshot& snap) {
        for (const sdk::EntityGroupSample& g : snap.entityGroups())
            if (g.group == "fuel") return static_cast<int>(g.positions.size());
        return -1;
    }

    sdk::DrivetrainRegistry drivetrains;
    sdk::TireModelRegistry tires;
    sdk::MechanismFactory mechanisms;
    sdk::GamePluginRegistry plugins;
    std::unique_ptr<core::runtime::SimulationSession> session;
    sdk::proto::SensorFrame frame;

private:
    std::map<sdk::ParticipantId, std::uint32_t> sent_;
};

}  // namespace g26session
