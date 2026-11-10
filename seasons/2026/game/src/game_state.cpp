#include "wpiemu/plugins/game_2026/game_state.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/fuel_layout.h"
#include "wpiemu/plugins/game_2026/game_sounds.h"
#include "wpiemu/plugins/game_2026/game_values.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"
#include "wpiemu/sdk/game/zone_activation.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr float kBoundsMarginM = 2.0f;  // fuel further than this outside the field is lost

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
    buildHostZones(ctx);
    order_.push_back(&host_);
}

std::string_view Game2026State::pluginId() const { return game_2026::pluginId(); }

void Game2026State::buildRuntimes(RobotState& robot, const std::vector<sdk::ZoneConfig>& configs) {
    robot.zones.clear();
    robot.zoneBits.clear();
    robot.intakes.clear();
    robot.shooters.clear();
    for (std::size_t i = 0; i < configs.size(); ++i) {
        const sdk::ZoneConfig& zone = configs[i];
        if (zone.kind == kPickupKind) {
            auto pickup = std::make_unique<PickupZoneRuntime>(zone);
            robot.intakes.push_back(pickup.get());
            robot.zones.push_back(std::move(pickup));
        } else if (zone.kind == kShooterKind) {
            auto shooter = std::make_unique<ShooterZoneRuntime>(zone, robot.remote);
            robot.shooters.push_back(shooter.get());
            robot.zones.push_back(std::move(shooter));
        } else {
            continue;
        }
        robot.zoneBits.push_back(i);
    }
}

void Game2026State::buildHostZones(sdk::SimContext& ctx) { buildRuntimes(host_, ctx.config().zones); }

void Game2026State::resetRobot(RobotState& robot) {
    if (robot.remote) buildRuntimes(robot, robot.zoneConfigs);
    robot.meter = std::min(config_.preloadFuel, config_.capacity);
    robot.shots = 0;
}

void Game2026State::refreshOrder() {
    order_.clear();
    order_.push_back(&host_);
    for (auto& entry : remotes_) order_.push_back(&entry.second);
}

const Game2026State::RobotState* Game2026State::robot(sdk::ParticipantId id) const {
    if (id == sdk::match::kHostParticipantId) return &host_;
    const auto it = remotes_.find(id);
    return it != remotes_.end() ? &it->second : nullptr;
}

std::vector<sdk::ParticipantId> Game2026State::robotIds() const {
    std::vector<sdk::ParticipantId> ids;
    ids.reserve(order_.size());
    for (const RobotState* r : order_) ids.push_back(r->id);
    return ids;
}

void Game2026State::onRobotAdded(sdk::SimContext&, const sdk::RobotView& robot) {
    if (robot.isHost || robot.participantId == sdk::match::kHostParticipantId) return;  // the host is always present
    RobotState state;
    state.id = robot.participantId;
    state.remote = true;
    if (robot.zones != nullptr) state.zoneConfigs = *robot.zones;
    buildRuntimes(state, state.zoneConfigs);
    state.meter = std::min(config_.preloadFuel, config_.capacity);
    remotes_.insert_or_assign(robot.participantId, std::move(state));
    refreshOrder();
}

void Game2026State::onRobotRemoved(sdk::SimContext&, sdk::ParticipantId participant) {
    if (participant == sdk::match::kHostParticipantId) return;
    if (remotes_.erase(participant) != 0) refreshOrder();
}

// Resolves where every robot is this tick, in the order they act.
void Game2026State::collectFrames(sdk::SimContext& ctx) {
    frames_.clear();
    for (RobotState* state : order_) {
        RobotFrame frame;
        frame.state = state;
        if (!state->remote) {
            frame.body = ctx.chassisBody();
            frame.present = true;
        } else if (const sdk::RobotView* view = ctx.robots().find(state->id)) {
            frame.view = view;
            frame.body = view->body;
            frame.present = true;
        }
        frame.pose = frame.body.valid() ? ctx.physics().pose(frame.body) : sdk::Pose3{};
        frames_.push_back(frame);
    }
}

void Game2026State::stepZones(sdk::SimContext& ctx, double dt) {
    sdk::IMechanismDirectory* dir = ctx.mechanisms();
    std::vector<std::string> missing;
    for (const RobotFrame& frame : frames_) {
        RobotState& robot = *frame.state;
        for (std::size_t i = 0; i < robot.zones.size(); ++i) {
            FuelZoneRuntime& zone = *robot.zones[i];
            const sdk::ZoneConfig& cfg = zone.config();
            if (robot.remote) {
                // No mechanisms: active iff the client's flag is set and the robot may act; chassis-frame rest pose.
                const bool held = frame.view != nullptr && frame.view->enabled && frame.view->zoneFlagActive(robot.zoneBits[i]);
                zone.update(ctx, sdk::restPoseOfZone(cfg).value_or(cfg.local), held, dt);
                continue;
            }
            sdk::Pose3 attachment;  // the chassis frame unless attached to a mechanism link that can be resolved
            if (!cfg.attachment.onChassis() && dir != nullptr)
                if (const auto p = dir->linkPoseInChassis(cfg.attachment.parentMechanismId, cfg.attachment.parentLink))
                    attachment = *p;
            missing.clear();
            const bool held = sdk::evaluateActivation(cfg.activation, dir, &missing);
            for (const std::string& mechanism : missing) zone.warnMissing(ctx, mechanism);
            zone.update(ctx, attachment * cfg.local, held, dt);
        }
    }
}

void Game2026State::reset(sdk::SimContext& ctx) {
    buildHostZones(ctx);
    pool_.clear();
    pending_.clear();
    hits_.clear();
    clock_.reset();
    status_ = MatchStatus{};
    hub_.reset();
    matchTimed_ = false;
    matchResult_ = false;
    resultCuePending_ = false;
    for (RobotState* robot : order_) resetRobot(*robot);
    lost_ = 0;
    for (OutpostState& o : outposts_) o = OutpostState{config_.outpostChuteFuel, 0.0};

    for (const sdk::Vec3& p : initialFuelPositions(config_, layout_)) pool_.spawn(p, {});
    refreshPositions();
}
void Game2026State::step(sdk::SimContext& ctx, double dt) {
    const sdk::proto::DsState& ds = ctx.driverStation();
    ownAlliance_ = sdk::proto::allianceOf(ds.station);
    // A DS match time (Practice) means a timed match: phases, shifts and scores apply. Without one the robot is in
    // free play: when enabled it behaves like test mode (both hubs scoring, no phases) and the match HUD is hidden.
    const bool timed = ds.matchTimeS >= 0.0;
    MatchStatus previous = status_;
    const bool resultBefore = matchResult_;
    if (timed && !matchTimed_) {
        clock_.reset();
        hub_.reset();
        previous = MatchStatus{};   // a new match starts from nothing: no boundary can be crossed on its first tick
    }
    // When the timed match stops (it ended or was aborted) its final score stays on the HUD until a reset, the next
    // timed match, or free play is enabled.
    if (timed) matchResult_ = false;
    else if (matchTimed_) matchResult_ = true;
    else if (matchResult_ && ds.enabled && !ds.eStop) matchResult_ = false;
    matchTimed_ = timed;
    if (timed || ds.mode == sdk::proto::RobotMode::Test || !ds.enabled || ds.eStop) {
        status_ = clock_.update(ds, dt);
    } else {
        sdk::proto::DsState freePlay = ds;
        freePlay.mode = sdk::proto::RobotMode::Test;
        status_ = clock_.update(freePlay, dt);
    }
    hub_.update(status_, ds, dt, ctx.rng());
    requestSoundCues(ctx, previous, matchResult_ && !resultBefore, ds.eStop);
    pool_.updateGhosts(ctx.clock().simTimeUs());

    handleSensorEvents(ctx);
    despawnOutOfPlay(ctx);
    collectFrames(ctx);
    stepZones(ctx, dt);
    for (const RobotFrame& frame : frames_) stepShooters(ctx, frame);
    for (const RobotFrame& frame : frames_) stepIntakes(frame);
    stepRecycling(ctx, dt);
    stepHumanPlayers(ctx, dt);
    refreshPositions();
}

void Game2026State::requestSoundCues(sdk::SimContext& ctx, const MatchStatus& previous, bool resultEdge, bool eStop) {
    // Only a boundary crossed inside one TELEOP run sounds: the first TELEOP tick belongs to the DS ("3 Bells"), and a
    // pause or re-enable inside a phase changes nothing.
    if (previous.teleopStarted && status_.running && status_.phase != previous.phase) {
        if (isShift(status_.phase)) ctx.requestSound(sound::kShiftChange);
        else if (status_.phase == MatchPhase::EndGame) ctx.requestSound(sdk::sound_cue::kEndGame);
    }

    // The result is posted once fuel is no longer assessed (manual 6.5). An E-Stopped match gets the foghorn instead.
    if (resultEdge && !eStop) resultCuePending_ = true;
    if (!matchResult_) resultCuePending_ = false;
    if (resultCuePending_ && !hub_.scoringOpen(Alliance::Red) && !hub_.scoringOpen(Alliance::Blue)) {
        resultCuePending_ = false;
        ctx.requestSound(sdk::sound_cue::kMatchResult);
    }
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

void Game2026State::stepShooters(sdk::SimContext& ctx, const RobotFrame& frame) {
    RobotState& robot = *frame.state;
    if (robot.shooters.empty()) return;
    const sdk::Pose3& chassis = frame.pose;
    const sdk::SimTimeUs now = ctx.clock().simTimeUs();
    const sdk::SimTimeUs ghostUntil = config_.shotGhostS > 0.f ? now + sdk::secondsToUs(config_.shotGhostS) : 0;
    const float spacing = 2.f * dim::kFuelRadiusM + 0.01f;

    for (IFuelShooterPort* shooter : robot.shooters) {
        if (robot.meter <= 0) break;
        if (!shooter->feeding()) continue;
        const int shots = std::min(shooter->shotBudget(), robot.meter);
        if (shots <= 0) continue;
        const ShotSolution sol = shooter->solution();
        const sdk::Vec3 muzzle = chassis.transformPoint(sol.muzzleInChassis);
        const sdk::Vec3 baseDir = chassis.transformVector(sol.directionInChassis);
        const sdk::Vec3 lateral = chassis.transformVector(sol.lateralInChassis);
        const int lanes = std::max(1, sol.parallelBalls);

        int fired = 0;
        for (int k = 0; k < shots; ++k) {
            // volleys of `lanes` balls leave side by side; successive rows are one diameter apart along the barrel
            const int row = k / lanes;
            const int lane = k % lanes;
            const int rowSize = std::min(lanes, shots - row * lanes);
            const float offset = (static_cast<float>(lane) - 0.5f * static_cast<float>(rowSize - 1)) * spacing;
            const sdk::Vec3 spawnPos = muzzle - baseDir * (spacing * static_cast<float>(row)) + lateral * offset;

            sdk::Vec3 robotVelocity;
            if (sol.inheritRobotVelocity && frame.body.valid())
                robotVelocity = ctx.physics().pointVelocity(frame.body, spawnPos);

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
            if (pool_.spawn(spawnPos, options) == 0) break;
            ++fired;
        }
        robot.meter -= fired;
        robot.shots += fired;
        shooter->consumeShots(fired);
    }
}

void Game2026State::stepIntakes(const RobotFrame& frame) {
    RobotState& robot = *frame.state;
    if (robot.intakes.empty()) return;
    const sdk::Pose3& chassis = frame.pose;
    struct Candidate {
        float distSq;
        std::uint32_t id;
    };
    std::vector<Candidate> candidates;

    for (IFuelIntakePort* intake : robot.intakes) {
        const int room = config_.capacity - robot.meter;
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
        robot.meter += taken;
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
    if (id == game_value::kFuelCount) return static_cast<double>(host_.meter);
    if (id == game_value::kFuelCapacity) return static_cast<double>(config_.capacity);
    if (id == game_value::kFuelFraction)
        return config_.capacity > 0 ? static_cast<double>(host_.meter) / config_.capacity : 0.0;
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
    for (const std::unique_ptr<FuelZoneRuntime>& zone : host_.zones) out.addZone({zone->id(), zone->active()});

    out.setHud(hud::kFuelCount, host_.meter);
    out.setHud(hud::kFuelCapacity, config_.capacity);
    out.setHud(hud::kFuelOnField, static_cast<int>(fuelPositions_.size()));
    out.setHud(hud::kShots, host_.shots);
    if (matchTimed_ || matchResult_) {
        out.setHud(hud::kScoreRed, hub_.score(Alliance::Red).points());
        out.setHud(hud::kScoreBlue, hub_.score(Alliance::Blue).points());
    }
    if (matchResult_) out.setHud(hud::kMatchPhase, std::string("FINAL"));
    if (matchTimed_) {
        out.setHud(hud::kHubRedActive, hub_.active(Alliance::Red));
        out.setHud(hud::kHubBlueActive, hub_.active(Alliance::Blue));
        out.setHud(hud::kHubOwnActive, hub_.active(ownAlliance_));
        const Alliance fi = hub_.firstInactive();
        out.setHud(hud::kHubRedStatus, hubStatusText(hub_.active(Alliance::Red),
                                                     activeRemainingS(Alliance::Red, status_, fi, config_.enforceHubShifts)));
        out.setHud(hud::kHubBlueStatus, hubStatusText(hub_.active(Alliance::Blue),
                                                      activeRemainingS(Alliance::Blue, status_, fi, config_.enforceHubShifts)));
        out.setHud(hud::kFirstInactive,
                   hub_.firstInactiveKnown() ? std::string(sdk::allianceKey(hub_.firstInactive())) : std::string("unknown"));
        out.setHud(hud::kMatchPhase, std::string(phaseName(status_.phase)));
        out.setHud(hud::kMatchTime, status_.periodRemainingS);
        out.setHud(hud::kPhaseTime, status_.phaseRemainingS);
    }
    out.setHud(hud::kOutpostRed, outposts_[index(Alliance::Red)].stock);
    out.setHud(hud::kOutpostBlue, outposts_[index(Alliance::Blue)].stock);
}

void Game2026State::publishRobot(sdk::SnapshotBuilder& out, const sdk::RobotView& view) const {
    const RobotState* robot = this->robot(view.participantId);
    if (robot == nullptr || !robot->remote) return;
    out.setRobotHud(robot->id, hud::kFuelCount, robot->meter);
    out.setRobotHud(robot->id, hud::kFuelCapacity, config_.capacity);
    out.setRobotHud(robot->id, hud::kShots, robot->shots);
    if (matchTimed_) out.setRobotHud(robot->id, hud::kHubOwnActive, hub_.active(view.alliance()));
}

}  // namespace wpiemu::plugins::game_2026
