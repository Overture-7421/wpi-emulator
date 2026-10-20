#include "wpiemu/plugins/game_2026/game_state.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/fuel_layout.h"
#include "wpiemu/plugins/game_2026/game_values.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"
#include "wpiemu/sdk/game/zone_activation.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr float kBoundsMarginM = 2.0f;  // fuel further than this outside the field is lost

Alliance allianceOf(sdk::proto::AllianceStation station) {
    return station <= sdk::proto::AllianceStation::Red3 ? Alliance::Red : Alliance::Blue;
}

sdk::Vec3 perturbDirection(const sdk::Vec3& dir, double yawJitter, double pitchJitter) {
    sdk::Vec3 right = sdk::cross(dir, sdk::Vec3{0.f, 0.f, 1.f});
    if (sdk::length(right) < 1e-4f) right = sdk::Vec3{0.f, 1.f, 0.f};
    right = sdk::normalized(right);
    const sdk::Vec3 up = sdk::cross(right, dir);
    return sdk::normalized(dir + right * static_cast<float>(std::tan(yawJitter)) + up * static_cast<float>(std::tan(pitchJitter)));
}

// Seconds the hub stays active from now: the rest of the current phase plus every directly following phase in which
// it is still active (a hub that scores in AUTO also does through TRANSITION).
double activeRemainingS(Alliance a, const MatchStatus& st, Alliance firstInactive, bool enforceShifts) {
    if (!enforceShifts || st.phase == MatchPhase::Auto) return st.periodRemainingS;
    static constexpr MatchPhase kOrder[] = {MatchPhase::Transition, MatchPhase::Shift1, MatchPhase::Shift2,
                                            MatchPhase::Shift3,     MatchPhase::Shift4, MatchPhase::EndGame};
    static constexpr double kLen[] = {kTransitionDurationS, kShiftDurationS, kShiftDurationS,
                                      kShiftDurationS,      kShiftDurationS, kEndGameDurationS};
    double total = st.phaseRemainingS;
    bool seen = false;
    for (std::size_t i = 0; i < 6; ++i) {
        if (!seen) { seen = kOrder[i] == st.phase; continue; }
        if (!hubActiveInPhase(a, kOrder[i], firstInactive)) break;
        total += kLen[i];
    }
    return total;
}

std::string hubStatusText(bool active, double remainingS) {
    if (!active) return "Inactive";
    const int s = std::max(0, static_cast<int>(std::ceil(remainingS)));
    char buf[32];
    std::snprintf(buf, sizeof buf, "Active %d:%02d", s / 60, s % 60);
    return buf;
}

}  // namespace

Game2026State::Game2026State(sdk::SimContext& ctx, GameConfig config, FieldLayout layout)
    : config_(std::move(config)), layout_(std::move(layout)),
      pool_(ctx.physics(), config_.fuel, config_.maxFuelOnField),
      hub_(config_.enforceHubShifts, config_.firstInactive) {
    buildZones(ctx);
}

std::string_view Game2026State::pluginId() const { return game_2026::pluginId(); }

void Game2026State::buildZones(sdk::SimContext& ctx) {
    zones_.clear();
    intakes_.clear();
    shooters_.clear();
    for (const sdk::ZoneConfig& zone : ctx.config().zones) {
        if (zone.kind == kPickupKind) {
            auto pickup = std::make_unique<PickupZoneRuntime>(zone);
            intakes_.push_back(pickup.get());
            zones_.push_back(std::move(pickup));
        } else if (zone.kind == kShooterKind) {
            auto shooter = std::make_unique<ShooterZoneRuntime>(zone);
            shooters_.push_back(shooter.get());
            zones_.push_back(std::move(shooter));
        }
    }
}

void Game2026State::stepZones(sdk::SimContext& ctx, double dt) {
    sdk::IMechanismDirectory* dir = ctx.mechanisms();
    std::vector<std::string> missing;
    for (const std::unique_ptr<FuelZoneRuntime>& zone : zones_) {
        const sdk::ZoneConfig& cfg = zone->config();
        sdk::Pose3 attachment;  // the chassis frame unless attached to a mechanism link that can be resolved
        if (!cfg.attachment.onChassis() && dir != nullptr)
            if (const auto p = dir->linkPoseInChassis(cfg.attachment.parentMechanismId, cfg.attachment.parentLink))
                attachment = *p;
        missing.clear();
        const bool held = sdk::evaluateActivation(cfg.activation, dir, &missing);
        for (const std::string& mechanism : missing) zone->warnMissing(ctx, mechanism);
        zone->update(ctx, attachment * cfg.local, held, dt);
    }
}

void Game2026State::reset(sdk::SimContext& ctx) {
    buildZones(ctx);
    pool_.clear();
    pending_.clear();
    hits_.clear();
    clock_.reset();
    status_ = MatchStatus{};
    hub_.reset();
    matchTimed_ = false;
    meter_ = std::min(config_.preloadFuel, config_.capacity);
    shots_ = 0;
    lost_ = 0;
    for (OutpostState& o : outposts_) o = OutpostState{config_.outpostChuteFuel, 0.0};

    for (const sdk::Vec3& p : initialFuelPositions(config_, layout_)) pool_.spawn(p, {});
    refreshPositions();
}

sdk::Pose3 Game2026State::chassisPose(sdk::SimContext& ctx) const {
    return ctx.chassisBody().valid() ? ctx.physics().pose(ctx.chassisBody()) : sdk::Pose3{};
}

void Game2026State::step(sdk::SimContext& ctx, double dt) {
    const sdk::proto::DsState& ds = ctx.driverStation();
    ownAlliance_ = allianceOf(ds.station);
    // A DS match time (Practice) means a timed match: phases, shifts and scores apply. Without one the robot is in
    // free play: when enabled it behaves like test mode (both hubs scoring, no phases) and the match HUD is hidden.
    const bool timed = ds.matchTimeS >= 0.0;
    if (timed && !matchTimed_) {
        clock_.reset();
        hub_.reset();
    }
    matchTimed_ = timed;
    if (timed || ds.mode == sdk::proto::RobotMode::Test || !ds.enabled || ds.eStop) {
        status_ = clock_.update(ds, dt);
    } else {
        sdk::proto::DsState freePlay = ds;
        freePlay.mode = sdk::proto::RobotMode::Test;
        status_ = clock_.update(freePlay, dt);
    }
    hub_.update(status_, ds, dt, ctx.rng());
    pool_.updateGhosts(ctx.clock().simTimeUs());

    handleSensorEvents(ctx);
    despawnOutOfPlay(ctx);
    stepZones(ctx, dt);
    stepShooters(ctx);
    stepIntakes(ctx);
    stepRecycling(ctx, dt);
    stepHumanPlayers(ctx, dt);
    refreshPositions();
}

void Game2026State::handleSensorEvents(sdk::SimContext& ctx) {
    // Copy first: despawning below must not happen while iterating the world's event span.
    hits_.clear();
    for (const sdk::SensorEvent& e : ctx.physics().sensorEvents()) {
        if (!e.begin || tagKind(e.visitorTag) != EntityKind::Fuel) continue;
        const EntityKind kind = tagKind(e.sensorTag);
        if (kind != EntityKind::HubSensor && kind != EntityKind::CorralSensor) continue;
        hits_.push_back({kind, tagAlliance(e.sensorTag), tagId(e.visitorTag)});
    }
    for (const SensorHit& hit : hits_) {
        if (!pool_.alive(hit.fuel)) continue;  // already consumed by an earlier event of this step
        pool_.despawn(hit.fuel);
        if (hit.kind == EntityKind::HubSensor) {
            hub_.recordFuel(hit.alliance, status_);
            if (config_.recycleScoredFuel) pending_.push_back({hit.alliance, config_.hubProcessingDelayS});
        } else {
            ++outposts_[index(hit.alliance)].stock;
        }
    }
}

void Game2026State::despawnOutOfPlay(sdk::SimContext&) {
    const float xMin = -kBoundsMarginM, xMax = dim::kFieldLengthM + kBoundsMarginM;
    const float yMin = -kBoundsMarginM, yMax = dim::kFieldWidthM + kBoundsMarginM;
    scratchIds_.clear();
    pool_.forEach([&](std::uint32_t id, const sdk::Vec3& p) {
        if (p.x < xMin || p.x > xMax || p.y < yMin || p.y > yMax || p.z < -kBoundsMarginM || !std::isfinite(p.x + p.y + p.z))
            scratchIds_.push_back(id);
    });
    for (const std::uint32_t id : scratchIds_) {
        pool_.despawn(id);
        ++lost_;
    }
}

void Game2026State::stepShooters(sdk::SimContext& ctx) {
    if (shooters_.empty()) return;
    const sdk::Pose3 chassis = chassisPose(ctx);
    const sdk::SimTimeUs now = ctx.clock().simTimeUs();
    const sdk::SimTimeUs ghostUntil = config_.shotGhostS > 0.f ? now + sdk::secondsToUs(config_.shotGhostS) : 0;
    const float spacing = 2.f * dim::kFuelRadiusM + 0.01f;

    for (IFuelShooterPort* shooter : shooters_) {
        if (meter_ <= 0) break;
        if (!shooter->feeding()) continue;
        const int shots = std::min(shooter->shotBudget(), meter_);
        if (shots <= 0) continue;
        const ShotSolution sol = shooter->solution();
        const sdk::Vec3 muzzle = chassis.transformPoint(sol.muzzleInChassis);
        const sdk::Vec3 baseDir = chassis.transformVector(sol.directionInChassis);
        sdk::Vec3 robotVelocity;
        if (sol.inheritRobotVelocity && ctx.chassisBody().valid())
            robotVelocity = ctx.physics().pointVelocity(ctx.chassisBody(), muzzle);

        int fired = 0;
        for (int k = 0; k < shots; ++k) {
            sdk::Vec3 dir = baseDir;
            if (sol.aimJitterRad > 0.0) {
                const double yawJitter = ctx.rng().uniform(-sol.aimJitterRad, sol.aimJitterRad);
                const double pitchJitter = ctx.rng().uniform(-sol.aimJitterRad, sol.aimJitterRad);
                dir = perturbDirection(dir, yawJitter, pitchJitter);
            }
            double speed = sol.exitSpeedMps;
            if (sol.speedJitter > 0.0) speed *= 1.0 + ctx.rng().uniform(-sol.speedJitter, sol.speedJitter);

            FuelPool::SpawnOptions options;
            options.velocity = dir * static_cast<float>(speed) + robotVelocity;
            options.fast = true;
            options.ghostUntilUs = ghostUntil;
            // several shots in one tick leave one ball diameter apart along the barrel
            if (pool_.spawn(muzzle - baseDir * (spacing * static_cast<float>(k)), options) == 0) break;
            ++fired;
        }
        meter_ -= fired;
        shots_ += fired;
        shooter->consumeShots(fired);
    }
}

void Game2026State::stepIntakes(sdk::SimContext& ctx) {
    if (intakes_.empty()) return;
    const sdk::Pose3 chassis = chassisPose(ctx);
    struct Candidate {
        float distSq;
        std::uint32_t id;
    };
    std::vector<Candidate> candidates;

    for (IFuelIntakePort* intake : intakes_) {
        const int room = config_.capacity - meter_;
        if (room <= 0) break;
        if (!intake->pickupActive()) continue;
        const int budget = std::min(intake->pickupBudget(), room);
        if (budget <= 0) continue;

        const sdk::Box3 zone = intake->zoneInChassis();
        const sdk::Pose3 inv = (chassis * zone.pose).inverse();
        const sdk::Vec3 half = zone.sizeM * 0.5f;
        candidates.clear();
        pool_.forEach([&](std::uint32_t id, const sdk::Vec3& p) {
            const sdk::Vec3 local = inv.transformPoint(p);
            if (std::fabs(local.x) <= half.x && std::fabs(local.y) <= half.y && std::fabs(local.z) <= half.z)
                candidates.push_back({sdk::dot(local, local), id});
        });
        if (candidates.empty()) continue;
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.distSq != b.distSq ? a.distSq < b.distSq : a.id < b.id;
        });
        const int taken = std::min(budget, static_cast<int>(candidates.size()));
        for (int i = 0; i < taken; ++i) pool_.despawn(candidates[static_cast<std::size_t>(i)].id);
        meter_ += taken;
        intake->consumePickups(taken);
    }
}

void Game2026State::stepRecycling(sdk::SimContext& ctx, double dt) {
    if (pending_.empty()) return;
    std::size_t keep = 0;
    bool spawnFailed = false;
    for (std::size_t i = 0; i < pending_.size(); ++i) {
        PendingRelease r = pending_[i];
        r.timeLeftS -= dt;
        if (r.timeLeftS > 0.0 || spawnFailed) {
            pending_[keep++] = r;
            continue;
        }
        const HubSpec& hub = layout_.hub(r.hub);
        const HubExit& exit = hub.exits[ctx.rng().nextU64() % hub.exits.size()];
        FuelPool::SpawnOptions options;
        options.velocity = exit.direction * exit.speedMps;
        if (pool_.spawn(exit.position, options) == 0) {
            r.timeLeftS = 0.0;
            pending_[keep++] = r;
            spawnFailed = true;  // At most one refused spawn per tick, even with a large backlog.
        }
    }
    pending_.resize(keep);
}

void Game2026State::stepHumanPlayers(sdk::SimContext& ctx, double dt) {
    if (config_.humanPlayer != HumanPlayerMode::Timed || !status_.running) return;
    for (const Alliance a : kAlliances) {
        OutpostState& o = outposts_[index(a)];
        if (o.stock <= 0) continue;
        o.timerS += dt;
        if (o.timerS < config_.humanPlayerPeriodS) continue;
        o.timerS = 0.0;
        const OutpostSpec& spec = layout_.outpost(a);
        const float offset = static_cast<float>(ctx.rng().uniform(-spec.chuteHalfWidthM, spec.chuteHalfWidthM));
        FuelPool::SpawnOptions options;
        options.velocity = spec.chuteVelocity;
        if (pool_.spawn(spec.chuteExit + spec.inwardY * offset, options) != 0) --o.stock;
    }
}

void Game2026State::refreshPositions() {
    fuelPositions_.clear();
    pool_.forEach([&](std::uint32_t, const sdk::Vec3& p) { fuelPositions_.push_back(p); });
}

std::optional<std::string> Game2026State::gameSpecificMessage() const {
    if (!hub_.firstInactiveKnown()) return std::nullopt;
    return std::string(hub_.firstInactive() == Alliance::Red ? "R" : "B");
}

std::optional<double> Game2026State::gameValue(std::string_view id) const {
    if (id == game_value::kFuelCount) return static_cast<double>(meter_);
    if (id == game_value::kFuelCapacity) return static_cast<double>(config_.capacity);
    if (id == game_value::kFuelFraction)
        return config_.capacity > 0 ? static_cast<double>(meter_) / config_.capacity : 0.0;
    return std::nullopt;
}

void Game2026State::publish(sdk::SnapshotBuilder& out) const {
    sdk::EntityGroupSample fuel;
    fuel.group = "fuel";
    fuel.shape = "sphere";
    fuel.radiusM = dim::kFuelRadiusM;
    fuel.color = "#ffd000";
    fuel.positions = fuelPositions_;
    out.addEntityGroup(std::move(fuel));
    for (const std::unique_ptr<FuelZoneRuntime>& zone : zones_) out.addZone({zone->id(), zone->active()});

    out.setHud(hud::kFuelCount, meter_);
    out.setHud(hud::kFuelCapacity, config_.capacity);
    out.setHud(hud::kFuelOnField, static_cast<int>(fuelPositions_.size()));
    out.setHud(hud::kShots, shots_);
    if (matchTimed_) {
        out.setHud(hud::kScoreRed, hub_.score(Alliance::Red).points());
        out.setHud(hud::kScoreBlue, hub_.score(Alliance::Blue).points());
        out.setHud(hud::kHubRedActive, hub_.active(Alliance::Red));
        out.setHud(hud::kHubBlueActive, hub_.active(Alliance::Blue));
        out.setHud(hud::kHubOwnActive, hub_.active(ownAlliance_));
        const Alliance fi = hub_.firstInactive();
        out.setHud(hud::kHubRedStatus, hubStatusText(hub_.active(Alliance::Red),
                                                     activeRemainingS(Alliance::Red, status_, fi, config_.enforceHubShifts)));
        out.setHud(hud::kHubBlueStatus, hubStatusText(hub_.active(Alliance::Blue),
                                                      activeRemainingS(Alliance::Blue, status_, fi, config_.enforceHubShifts)));
        out.setHud(hud::kFirstInactive,
                   hub_.firstInactiveKnown() ? std::string(allianceName(hub_.firstInactive())) : std::string("unknown"));
        out.setHud(hud::kMatchPhase, std::string(phaseName(status_.phase)));
        out.setHud(hud::kMatchTime, status_.periodRemainingS);
        out.setHud(hud::kPhaseTime, status_.phaseRemainingS);
    }
    out.setHud(hud::kOutpostRed, outposts_[index(Alliance::Red)].stock);
    out.setHud(hud::kOutpostBlue, outposts_[index(Alliance::Blue)].stock);
}

}  // namespace wpiemu::plugins::game_2026
