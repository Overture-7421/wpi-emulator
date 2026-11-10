#include "remote_robots.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "wheel_sample.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"
#include "wpiemu/sdk/physics/robot_tags.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

namespace {

constexpr std::size_t kMaxListedViolations = 6;

std::string describeViolations(const std::vector<SpecViolation>& violations) {
    std::string text = "robot spec rejected: ";
    for (std::size_t i = 0; i < violations.size() && i < kMaxListedViolations; ++i) {
        if (i > 0) text += ", ";
        text += violations[i].code + " (" + violations[i].path + ")";
    }
    if (violations.size() > kMaxListedViolations) text += " and " + std::to_string(violations.size() - kMaxListedViolations) + " more";
    return text;
}

}  // namespace

// ---- placement ----------------------------------------------------------------------------------------------------

RobotStartPose remoteStartPose(const FieldFrame& frame, proto::AllianceStation station) {
    const float setback = std::min(kRemoteStartSetbackM, frame.lengthM * kRemoteStartSetbackFraction);
    const float fromLeft = (2.f * static_cast<float>(proto::stationNumberOf(station)) - 1.f) / 6.f;
    RobotStartPose pose;
    if (proto::allianceOf(station) == Alliance::Blue) {
        pose.xM = setback;
        pose.yM = frame.widthM * (1.f - fromLeft);
        pose.headingRad = 0.f;
    } else {
        pose.xM = frame.lengthM - setback;
        pose.yM = frame.widthM * fromLeft;
        pose.headingRad = static_cast<float>(kPi);
    }
    return pose;
}

float chassisFootprintRadiusM(const ChassisSpec& chassis) {
    return 0.5f * std::sqrt(chassis.sizeM.x * chassis.sizeM.x + chassis.sizeM.y * chassis.sizeM.y);
}

RobotStartPose remoteStartPoseClear(const FieldFrame& frame, proto::AllianceStation station, float radiusM,
                                    std::span<const StartDisc> occupied) {
    RobotStartPose pose = remoteStartPose(frame, station);
    const float awayFromWall = proto::allianceOf(station) == Alliance::Blue ? 1.f : -1.f;
    const auto inside = [&](float v, float length) {
        return length > 2.f * radiusM ? std::clamp(v, radiusM, length - radiusM) : v;
    };
    pose.yM = inside(pose.yM, frame.widthM);
    const auto blocked = [&](const RobotStartPose& p) {
        return std::any_of(occupied.begin(), occupied.end(), [&](const StartDisc& d) {
            const float distance = std::hypot(p.xM - d.xM, p.yM - d.yM);
            return distance < radiusM + d.radiusM + kRemoteStartClearanceM;
        });
    };
    const int maxSteps = static_cast<int>(frame.lengthM / kRemoteStartStepM);
    for (int step = 0; step < maxSteps && blocked(pose); ++step) pose.xM += awayFromWall * kRemoteStartStepM;
    pose.xM = inside(pose.xM, frame.lengthM);
    return pose;
}

// ---- roster -------------------------------------------------------------------------------------------------------

RemoteRobots::RemoteRobots(IPhysicsWorld& world, RemoteChassisFactory factory, const DrivetrainRegistry* drivetrains,
                           const TireModelRegistry* tireModels)
    : world_(world), factory_(std::move(factory)), drivetrains_(drivetrains), tireModels_(tireModels) {
    views_.emplace_back();
}

RemoteRobots::~RemoteRobots() = default;

RemoteRobots::Robot* RemoteRobots::lookup(match::ParticipantId id) {
    for (const auto& r : robots_)
        if (r->id == id) return r.get();
    return nullptr;
}

const RemoteRobots::Robot* RemoteRobots::lookup(match::ParticipantId id) const {
    for (const auto& r : robots_)
        if (r->id == id) return r.get();
    return nullptr;
}

Status RemoteRobots::validate(const RemoteRobotRequest& request) const {
    if (!factory_) return makeError(ErrorCode::Unsupported, "this session has no remote robot support (no chassis factory)");
    if (request.participantId == match::kHostParticipantId || request.participantId == match::kNoParticipant)
        return makeError(ErrorCode::InvalidArgument, "participant id " + std::to_string(request.participantId) + " is reserved");
    if (lookup(request.participantId) != nullptr)
        return makeError(ErrorCode::AlreadyExists, "participant " + std::to_string(request.participantId) + " is already in the session");
    for (const auto& r : robots_)
        if (r->station == request.station)
            return makeError(ErrorCode::AlreadyExists, "station " + std::string(request.station <= proto::AllianceStation::Red3 ? "red " : "blue ") +
                                                           std::to_string(proto::stationNumberOf(request.station)) + " is taken");
    if (robots_.size() >= kMaxRemoteRobots)
        return makeError(ErrorCode::OutOfRange, "a match holds at most " + std::to_string(kMaxRemoteRobots) + " remote robots");
    if (request.name.size() > match::kMaxNameBytes) return makeError(ErrorCode::InvalidArgument, "robot name is too long");

    const RobotConfig& spec = request.spec;
    if (spec.zones.size() > match::kMaxZonesPerRobot)
        return makeError(ErrorCode::InvalidArgument, "a remote robot has at most " + std::to_string(match::kMaxZonesPerRobot) + " zones");
    if (drivetrains_ == nullptr || tireModels_ == nullptr)
        return makeError(ErrorCode::Unsupported, "remote robots need the drivetrain and tire model registries");
    if (!drivetrains_->contains(spec.drivetrain.spec.kind))
        return makeError(ErrorCode::InvalidArgument, "unknown drivetrain kind '" + spec.drivetrain.spec.kind + "'");
    if (!tireModels_->contains(spec.drivetrain.spec.tireModel))
        return makeError(ErrorCode::InvalidArgument, "unknown tire model '" + spec.drivetrain.spec.tireModel + "'");
    if (spec.drivetrain.spec.wheels.empty() || spec.drivetrain.spec.wheels.size() > match::kMaxWheelsPerRobot)
        return makeError(ErrorCode::InvalidArgument, "a remote robot has 1 to " + std::to_string(match::kMaxWheelsPerRobot) + " wheels");
    const std::vector<SpecViolation> violations = checkRemoteSpec(spec.chassis, spec.drivetrain.spec);
    if (!violations.empty()) return makeError(ErrorCode::InvalidArgument, describeViolations(violations));
    return okStatus();
}

Status RemoteRobots::add(const RemoteRobotRequest& request, const RobotStartPose& placement, SimTimeUs nowUs) {
    if (Status s = validate(request); !s) return s;
    const std::uint64_t tag = makeRobotTag(request.participantId);
    Result<std::unique_ptr<IRemoteChassis>> chassis = factory_(world_, request.spec, tag, placement);
    if (!chassis) return makeError(chassis.error().code, "remote chassis: " + chassis.error().message);
    if (chassis.value()->wheelCount() != request.spec.drivetrain.spec.wheels.size())
        return makeError(ErrorCode::Internal, "remote chassis has a different wheel count than its spec");

    auto robot = std::make_unique<Robot>();
    robot->id = request.participantId;
    robot->name = request.name;
    robot->station = request.station;
    robot->spec = request.spec;
    robot->placement = placement;
    robot->footprintRadiusM = chassisFootprintRadiusM(request.spec.chassis);
    robot->zoneMask = match::zoneFlagsMask(request.spec.zones.size());
    for (const WheelSpec& w : request.spec.drivetrain.spec.wheels) robot->maxWheelSpeed.push_back(maxWheelSpeedRadPerSec(w));
    robot->chassis = std::move(chassis).value();
    robot->lastInputUs = nowUs;

    const auto at = std::find_if(robots_.begin(), robots_.end(), [&](const auto& r) { return r->id > request.participantId; });
    robots_.insert(at, std::move(robot));
    rebuildViews();
    return okStatus();
}

bool RemoteRobots::remove(match::ParticipantId id) {
    const auto it = std::find_if(robots_.begin(), robots_.end(), [id](const auto& r) { return r->id == id; });
    if (it == robots_.end()) return false;
    robots_.erase(it);
    rebuildViews();
    return true;
}

std::vector<match::ParticipantId> RemoteRobots::ids() const {
    std::vector<match::ParticipantId> out;
    out.reserve(robots_.size());
    for (const auto& r : robots_) out.push_back(r->id);
    return out;
}

std::optional<RemoteRobotInfo> RemoteRobots::info(match::ParticipantId id) const {
    const Robot* r = lookup(id);
    if (r == nullptr) return std::nullopt;
    RemoteRobotInfo out;
    out.participantId = r->id;
    out.name = r->name;
    out.station = r->station;
    out.placement = r->placement;
    out.stats = r->stats;
    out.stats.queueDepth = r->queue.size();
    out.enabled = r->enabled;
    out.zoneFlags = r->activeZoneFlags;
    return out;
}

std::vector<StartDisc> RemoteRobots::footprints() const {
    std::vector<StartDisc> out;
    out.reserve(robots_.size());
    for (const auto& r : robots_) out.push_back({r->placement.xM, r->placement.yM, r->footprintRadiusM});
    return out;
}

// ---- inputs -------------------------------------------------------------------------------------------------------

Result<RemoteInputOutcome> RemoteRobots::submit(match::ParticipantId id, const match::MatchInput& input, std::uint64_t tickNow,
                                                SimTimeUs stepUs) {
    Robot* r = lookup(id);
    if (r == nullptr) return makeError(ErrorCode::NotFound, "no remote robot " + std::to_string(id));
    RemoteInputStats& stats = r->stats;
    ++stats.received;

    if (input.inputSeq == match::kNoInputSeq) {
        ++stats.rejected;
        return makeError(ErrorCode::InvalidArgument, "input sequence 0 is reserved");
    }
    if (input.wheels.size() != r->chassis->wheelCount()) {
        ++stats.rejected;
        return makeError(ErrorCode::InvalidArgument, "input has " + std::to_string(input.wheels.size()) + " wheels, the robot has " +
                                                         std::to_string(r->chassis->wheelCount()));
    }
    const std::int64_t leadTicks = static_cast<std::int64_t>(input.targetTick - tickNow);
    if (leadTicks > 0 && leadTicks > kMaxRemoteInputLeadUs / std::max<SimTimeUs>(stepUs, 1)) {
        ++stats.rejected;
        return makeError(ErrorCode::OutOfRange, "input targets a tick too far ahead");
    }

    if (stats.applied > 0 && input.inputSeq == stats.lastAppliedSeq) {
        ++stats.duplicate;
        return RemoteInputOutcome::Duplicate;
    }
    if (stats.applied > 0 && match::seqNewer(stats.lastAppliedSeq, input.inputSeq)) {
        ++stats.stale;
        return RemoteInputOutcome::Stale;
    }
    for (const Queued& q : r->queue)
        if (q.seq == input.inputSeq) {
            ++stats.duplicate;
            return RemoteInputOutcome::Duplicate;
        }

    if (!r->anyReceived || match::seqNewer(input.inputSeq, r->newestReceivedSeq)) {
        r->anyReceived = true;
        r->newestReceivedSeq = input.inputSeq;
        stats.inputLeadTicks = leadTicks;
    }

    auto pos = r->queue.begin();
    while (pos != r->queue.end() && !match::seqNewer(pos->seq, input.inputSeq)) ++pos;
    if (r->queue.size() >= kMaxQueuedRemoteInputs) {
        if (pos == r->queue.begin()) {   // older than everything we hold: it is the one to drop
            ++stats.evicted;
            return RemoteInputOutcome::Stale;
        }
        const auto index = pos - r->queue.begin() - 1;
        r->queue.erase(r->queue.begin());
        pos = r->queue.begin() + index;
        ++stats.evicted;
    }

    Queued q;
    q.seq = input.inputSeq;
    q.targetTick = input.targetTick;
    q.zoneFlags = input.zoneFlags & r->zoneMask;
    const auto& wheels = r->spec.drivetrain.spec.wheels;
    q.setpoints.reserve(wheels.size());
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        WheelSetpoint sp;
        sp.wheelSpeedRadPerSec = match::sanitizedWheelSpeed(input.wheels[i].wheelSpeedRadPerSec, r->maxWheelSpeed[i]);
        sp.steerAngleRad = isSteered(wheels[i]) ? match::dequantizeAngle(input.wheels[i].steerAngleQ) : 0.f;
        q.setpoints.push_back(sp);
    }
    r->queue.insert(pos, std::move(q));
    ++stats.queued;
    if (input.targetTick < tickNow) {
        ++stats.late;
        return RemoteInputOutcome::QueuedLate;
    }
    return RemoteInputOutcome::Queued;
}

// ---- tick ---------------------------------------------------------------------------------------------------------

void RemoteRobots::beforeStep(std::uint64_t tick, SimTimeUs nowUs, bool enabled, float dt) {
    for (const auto& robot : robots_) {
        Robot& r = *robot;
        // The newest sequence whose target tick has come; everything older is superseded by it.
        for (std::size_t i = r.queue.size(); i-- > 0;) {
            if (r.queue[i].targetTick > tick) continue;
            const Queued& q = r.queue[i];
            r.chassis->setSetpoints(q.setpoints);
            r.zoneFlags = q.zoneFlags;
            r.stats.lastAppliedSeq = q.seq;
            r.stats.ackTick = tick;
            ++r.stats.applied;
            r.stats.superseded += i;
            r.lastInputUs = nowUs;
            r.queue.erase(r.queue.begin(), r.queue.begin() + static_cast<std::ptrdiff_t>(i) + 1);
            break;
        }
        const bool timedOut = nowUs - r.lastInputUs >= match::kInputTimeoutUs;
        if (timedOut) r.chassis->brake();
        r.enabled = enabled;
        r.activeZoneFlags = enabled && !timedOut ? r.zoneFlags : 0u;
        r.chassis->prepareStep(enabled, dt);
    }
    refreshViews();
}

void RemoteRobots::afterStep(float dt) {
    for (const auto& r : robots_) r->chassis->completeStep(dt);
}

void RemoteRobots::reset() {
    for (const auto& robot : robots_) {
        Robot& r = *robot;
        r.chassis->reset(r.placement);
        r.queue.clear();
        r.stats = RemoteInputStats{};
        r.anyReceived = false;
        r.newestReceivedSeq = 0;
        r.zoneFlags = 0;
        r.activeZoneFlags = 0;
        r.lastInputUs = 0;
        r.enabled = false;
    }
    rebuildViews();
}

// ---- directory and snapshot -----------------------------------------------------------------------------------------

void RemoteRobots::setHost(Host host) {
    host_ = std::move(host);
    rebuildViews();
}

void RemoteRobots::setHostState(proto::AllianceStation station, bool enabled) {
    host_.station = station;
    host_.enabled = enabled;
    refreshViews();
}

// The per-tick path: same roster, so only the changing fields are written and nothing allocates.
void RemoteRobots::refreshViews() {
    if (views_.size() != robots_.size() + 1) return rebuildViews();
    RobotView& host = views_.front();
    host.station = host_.station;
    host.enabled = host_.enabled;
    for (std::size_t i = 0; i < robots_.size(); ++i) {
        views_[i + 1].zoneFlags = robots_[i]->activeZoneFlags;
        views_[i + 1].enabled = robots_[i]->enabled;
    }
}

RobotView RemoteRobots::viewOf(const Robot& r) {
    RobotView v;
    v.participantId = r.id;
    v.name = r.name;
    v.station = r.station;
    v.body = r.chassis->body();
    v.zones = &r.spec.zones;
    v.zoneFlags = r.activeZoneFlags;
    v.enabled = r.enabled;
    v.isHost = false;
    return v;
}

void RemoteRobots::rebuildViews() {
    views_.clear();
    RobotView host;
    host.participantId = match::kHostParticipantId;
    host.name = host_.name;
    host.station = host_.station;
    host.body = host_.body;
    host.enabled = host_.enabled;
    host.isHost = true;
    views_.push_back(std::move(host));
    for (const auto& r : robots_) views_.push_back(viewOf(*r));
}

void RemoteRobots::addSamples(SnapshotBuilder& out) const {
    for (const auto& robot : robots_) {
        const Robot& r = *robot;
        RobotSample s;
        s.participantId = r.id;
        s.name = r.name;
        s.station = r.station;
        const BodyId body = r.chassis->body();
        s.body.name = "chassis";
        s.body.pose = world_.pose(body);
        s.body.linearVelocity = world_.linearVelocity(body);
        s.body.angularVelocity = world_.angularVelocity(body);
        const auto& wheels = r.spec.drivetrain.spec.wheels;
        const auto states = r.chassis->wheelStates();
        for (std::size_t i = 0; i < wheels.size() && i < states.size(); ++i) s.wheels.push_back(makeWheelSample(wheels[i], states[i]));
        for (std::size_t i = 0; i < r.spec.zones.size(); ++i)
            s.zones.push_back({r.spec.zones[i].id, ((r.activeZoneFlags >> i) & 1u) != 0});
        out.addRobot(std::move(s));
    }
}

void RemoteRobots::publishHud(const IGameState& state, SnapshotBuilder& out) const {
    for (std::size_t i = 1; i < views_.size(); ++i) state.publishRobot(out, views_[i]);
}

}  // namespace wpiemu::core::runtime
