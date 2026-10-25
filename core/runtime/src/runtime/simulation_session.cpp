#include "wpiemu/core/runtime/simulation_session.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/runtime/motion_link_graph.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

// ---- directory ----------------------------------------------------------------------------------------------

class SimulationSession::Directory final : public IMechanismDirectory {
public:
    Directory(const RobotConfig& config, const MechanismFactory& factory, std::vector<std::unique_ptr<IMechanism>>& instances)
        : config_(config), factory_(factory), instances_(instances) {}

    IMechanism* find(std::string_view id) override {
        for (const auto& m : instances_)
            if (m->id() == id) return m.get();
        return nullptr;
    }
    std::vector<IMechanism*> all() override {
        std::vector<IMechanism*> out;
        out.reserve(instances_.size());
        for (const auto& m : instances_) out.push_back(m.get());
        return out;
    }
    std::optional<Pose3> linkPoseInChassis(std::string_view mechanismId, std::string_view link) const override {
        return resolveLinkPoseInChassis(config_, mechanismId, link, [this](std::string_view id, std::string_view l) { return rootRelative(id, l); });
    }

private:
    std::optional<Pose3> rootRelative(std::string_view id, std::string_view link) const {
        for (const auto& m : instances_)
            if (m->id() == id) {
                if (const std::optional<Pose3> live = m->linkPose(link)) return live;
                break;  // the mechanism does not publish this link: use the template's rest pose
            }
        if (link == kRootLink) return Pose3{};
        if (const MechanismConfig* cfg = config_.findMechanism(id))
            if (const MechanismTemplateInfo* info = factory_.templateInfo(cfg->type))
                for (const LinkSpec& spec : info->links)
                    if (spec.name == link) return spec.local;
        return std::nullopt;
    }

    const RobotConfig& config_;
    const MechanismFactory& factory_;
    std::vector<std::unique_ptr<IMechanism>>& instances_;
};

// ---- reference chassis ----------------------------------------------------------------------------------------

Result<ChassisInfo> buildReferenceChassis(IPhysicsWorld& world, const ChassisSpec& chassis, const DrivetrainSpec& drivetrain) {
    if (!(chassis.massKg > 0.f) || !(chassis.comHeightM > 0.f) || !(chassis.sizeM.x > 0.f) || !(chassis.sizeM.y > 0.f) || !(chassis.sizeM.z > 0.f))
        return makeError(ErrorCode::InvalidArgument, "chassis needs positive mass, size and COM height");

    BodyDesc body;
    body.type = BodyType::Dynamic;
    body.allowSleep = false;
    body.pose = Pose3{Vec3{0.f, 0.f, chassis.comHeightM}, Quat{}};
    const BodyId id = world.createBody(body);
    if (!id.valid()) return makeError(ErrorCode::Internal, "physics world refused to create the chassis body");

    constexpr std::uint64_t kHullMask = layer::Field | layer::Fuel | layer::Robot | layer::Projectile | layer::Sensor;
    constexpr std::uint64_t kPadMask = layer::Field | layer::Fuel | layer::Robot | layer::Projectile;

    ShapeDesc hull;
    hull.geometry = BoxShape{chassis.sizeM * 0.5f};
    hull.local.position = Vec3{-chassis.comOffsetM.x, -chassis.comOffsetM.y, chassis.groundClearanceM + chassis.sizeM.z * 0.5f - chassis.comHeightM};
    hull.material = chassis.material;
    hull.category = layer::Robot;
    hull.mask = kHullMask;
    hull.contactEvents = true;
    if (!world.addShape(id, hull).valid()) {
        world.destroyBody(id);
        return makeError(ErrorCode::Internal, "physics world refused the chassis hull");
    }
    for (const PrimitiveSpec& p : chassis.primitives) {
        if (!p.collides) continue;
        ShapeDesc s;
        if (p.kind == PrimitiveKind::Box) s.geometry = BoxShape{p.sizeM * 0.5f};
        else s.geometry = CylinderShape{p.radiusM, p.heightM * 0.5f};
        s.local = p.local;
        s.material = chassis.material;
        s.category = layer::Robot;
        s.mask = kHullMask;
        (void)world.addShape(id, s);
    }
    for (const WheelSpec& w : drivetrain.wheels) {
        ShapeDesc pad;
        pad.geometry = SphereShape{w.radiusM};
        const float hubZ = w.positionM.z > 0.f ? w.positionM.z : w.radiusM;
        pad.local.position = Vec3{w.positionM.x, w.positionM.y, hubZ - chassis.comHeightM};
        pad.material = Material{0.f, 0.f, 0.f};  // zero friction: traction comes from the tire model (docs/reference/physics.md)
        pad.category = layer::Robot;
        pad.mask = kPadMask;
        (void)world.addShape(id, pad);
    }

    MassProperties mp;
    mp.massKg = chassis.massKg;
    mp.comLocal = Vec3{};
    const float m = chassis.massKg;
    const Vec3 s = chassis.sizeM;
    mp.inertiaDiagLocal = Vec3{m / 12.f * (s.y * s.y + s.z * s.z), m / 12.f * (s.x * s.x + s.z * s.z), m / 12.f * (s.x * s.x + s.y * s.y)};
    if (chassis.inertiaDiagKgM2.x > 0.f && chassis.inertiaDiagKgM2.y > 0.f && chassis.inertiaDiagKgM2.z > 0.f) mp.inertiaDiagLocal = chassis.inertiaDiagKgM2;
    world.setMassProperties(id, mp);

    ChassisInfo info;
    info.body = id;
    info.baseMass = mp;
    info.spawnPose = body.pose;
    return info;
}

// ---- session ---------------------------------------------------------------------------------------------------

SimulationSession::SimulationSession(SessionParts& parts, SimTimeUs stepUs)
    : config_(std::move(parts.config)),
      options_(parts.options),
      logger_(parts.logger != nullptr ? parts.logger : &nullLogger_),
      clock_(stepUs),
      rng_(config_.sim.seed),
      hub_(logger_),
      battery_(config_.battery) {
    hub_.setBatteryVoltage(battery_.voltageV());
}

SimulationSession::~SimulationSession() = default;

IMechanismDirectory& SimulationSession::mechanisms() { return *directory_; }

Status SimulationSession::bindSlot(const SlotAddress& slot, const DeviceKey& device) {
    if (MotionLinkGraph::isDrivenByMotionLink(motionAxes_, motionSources_, slot))
        return makeError(
            ErrorCode::InvalidArgument,
            "This axis follows another axis. Remove its motion link before mapping a motor.");
    return hub_.bind(slot, device);
}

Result<std::unique_ptr<SimulationSession>> SessionBuilder::build(SessionParts parts) {
    if (!parts.world) return makeError(ErrorCode::InvalidArgument, "session needs a physics world");
    if (parts.mechanisms == nullptr) return makeError(ErrorCode::InvalidArgument, "session needs a mechanism factory");
    if (parts.config.sim.stepUs <= 0) return makeError(ErrorCode::InvalidArgument, "sim.stepUs must be positive");
    const SimTimeUs stepUs = parts.config.sim.stepUs;
    std::unique_ptr<SimulationSession> session(new SimulationSession(parts, stepUs));
    if (Status s = session->build(parts); !s) return s.error();
    return session;
}

Status SimulationSession::build(SessionParts& parts) {
    factory_ = parts.mechanisms;
    world_ = std::move(parts.world);
    coupling_ = std::move(parts.coupling);

    // 1. field of the selected game plugin (static world content)
    if (parts.plugins != nullptr) {
        module_ = config::selectPlugin(config_, *parts.plugins);
        if (module_ == nullptr && !config_.gamePluginId.empty())
            logger_->warn("session", "game plugin '" + config_.gamePluginId + "' is not available: running core features only");
    }
    if (module_ != nullptr) {
        Result<std::unique_ptr<IField>> field = module_->createField(*world_);
        if (!field) return makeError(field.error().code, "plugin field: " + field.error().message);
        field_ = std::move(field).value();
    } else if (parts.fallbackField) {
        Result<std::unique_ptr<IField>> field = parts.fallbackField(*world_);
        if (!field) return makeError(field.error().code, "default field: " + field.error().message);
        field_ = std::move(field).value();
    }

    // 2. chassis + drivetrain
    const ChassisBuilder builder = parts.chassisBuilder ? parts.chassisBuilder : ChassisBuilder(buildReferenceChassis);
    Result<ChassisInfo> chassis = builder(*world_, config_.chassis, config_.drivetrain.spec);
    if (!chassis) return makeError(chassis.error().code, "chassis: " + chassis.error().message);
    chassis_ = chassis.value();
    if (coupling_) coupling_->bind(*world_, chassis_.body, chassis_.baseMass);
    ctx_ = std::make_unique<SimContext>(SimContextServices{*world_, clock_, hub_, *logger_, config_, rng_, ds_, chassis_.body, nullptr,
                                                           coupling_ ? &coupling_->service() : nullptr, parts.sounds});

    if (!config_.drivetrain.spec.wheels.empty()) {
        if (parts.drivetrains == nullptr || parts.tireModels == nullptr)
            return makeError(ErrorCode::InvalidArgument, "the config has wheels: drivetrain and tire model registries are required");
        Result<std::unique_ptr<ITireModel>> tire = parts.tireModels->create(config_.drivetrain.spec.tireModel);
        if (!tire) return makeError(tire.error().code, "tire model: " + tire.error().message);
        tire_ = std::move(tire).value();
        const DrivetrainBuildArgs args{config_.drivetrain.spec, config_.chassis, *world_, chassis_.body, *tire_};
        Result<std::unique_ptr<IDrivetrain>> dt = parts.drivetrains->create(config_.drivetrain.spec.kind, args);
        if (!dt) return makeError(dt.error().code, "drivetrain: " + dt.error().message);
        drivetrain_ = std::move(dt).value();
        wheelCommands_.assign(config_.drivetrain.spec.wheels.size(), WheelCommand{});
    }
    if (coupling_) coupling_->attachDrivetrain(drivetrain_.get());

    // 3. slots and persisted mappings
    if (drivetrain_) hub_.declareSlots(slot_owner::kDrivetrain, drivetrain_->slotSpecs());
    hub_.declareSlots(slot_owner::kRobot, robotSlotSpecs());
    for (const SensorConfig& s : config_.sensors)
        if (!s.id.empty()) hub_.declareSlots(s.id, {sensorSlotSpec(s.kind)});
    for (MechanismConfig& m : config_.mechanisms)
        if (const MechanismTemplateInfo* info = factory_->templateInfo(m.type)) {
            hub_.declareSlots(m.id, info->slotSpecs);
            applyDefaults(info->properties, m.properties);
        }
    if (Status s = hub_.loadMappings(config_.allMappings()); !s)
        logger_->warn("session", "some slot mappings could not be applied: " + s.error().message);

    // 4. mechanisms in config order (parents before children for a consistent link view)
    directory_ = std::make_unique<Directory>(config_, *factory_, mechanismInstances_);
    ctx_->bindMechanisms(directory_.get());
    for (const MechanismConfig& m : config_.mechanisms) {
        Result<std::unique_ptr<IMechanism>> mech = factory_->create(m.type, MechanismBuildContext{m, *ctx_});
        if (!mech) return makeError(mech.error().code, "mechanism '" + m.id + "': " + mech.error().message);
        mechanismInstances_.push_back(std::move(mech).value());
    }
    for (const MechanismConfig& m : config_.mechanisms)
        if (!directory_->linkPoseInChassis(m.id, kRootLink))
            sessionWarnings_.push_back({Severity::Warning, DeviceId{}, "attachment_unresolved",
                                        "mechanism '" + m.id + "' has a broken attachment chain; it is mounted on the chassis root"});

    if (!config_.motionLinks.empty()) {
        Result<MotionLinkGraph> graph =
            MotionLinkGraph::build(config_, drivetrain_.get(), mechanismInstances_);
        if (!graph) return graph.error();
        motionAxes_ = std::move(graph.value().axes);
        motionStarts_ = std::move(graph.value().starts);
        motionSources_ = std::move(graph.value().sources);
        motionRatios_ = std::move(graph.value().ratios);
        motionOrder_ = std::move(graph.value().order);
        motionMechanisms_ = std::move(graph.value().mechanismHasAxes);
        motionDrivetrain_ = graph.value().drivetrainHasAxes;
    }

    // 5. game state
    if (module_ != nullptr) {
        Result<std::unique_ptr<IGameState>> state = module_->createGameState(*ctx_);
        if (!state) return makeError(state.error().code, "plugin game state: " + state.error().message);
        gameState_ = std::move(state).value();
        ctx_->bindGameState(gameState_.get());
        gameState_->reset(*ctx_);
    }

    vision_ = std::make_unique<VisionSystem>(config_, module_ != nullptr ? module_->aprilTagLayout() : AprilTagLayout{});
    sensors_ = std::make_unique<SensorSystem>(config_);

    world_->setPose(chassis_.body, chassis_.spawnPose);
    return okStatus();
}

void SimulationSession::reset() {
    hub_.resetSignalHistory();
    Pose3 spawn = chassis_.spawnPose;
    if (startOverride_) spawn = Pose3{{startOverride_->xM, startOverride_->yM, spawn.position.z}, Quat::fromYaw(startOverride_->headingRad)};
    world_->setPose(chassis_.body, spawn);
    world_->setLinearVelocity(chassis_.body, Vec3{});
    world_->setAngularVelocity(chassis_.body, Vec3{});
    world_->setMassProperties(chassis_.body, chassis_.baseMass);
    if (coupling_) coupling_->reset();
    if (drivetrain_) drivetrain_->reset();
    for (const auto& m : mechanismInstances_) m->reset();
    for (std::size_t i = 0; i < motionAxes_.size(); ++i) motionStarts_[i] = motionAxes_[i].state();
    clock_.reset();
    rng_.seed(config_.sim.seed);
    battery_.reset();
    hub_.setBatteryVoltage(battery_.voltageV());
    lastVelocity_ = Vec3{};
    brownoutWarned_ = false;
    std::erase_if(sessionWarnings_, [](const HubWarning& w) { return w.code == "brownout"; });
    if (gameState_) gameState_->reset(*ctx_);
    vision_->reset();
}

void SimulationSession::stepDrivetrain(double dt, double& currentA) {
    currentA = 0.0;
    if (!drivetrain_) return;
    const auto& wheels = drivetrain_->spec().wheels;
    for (std::size_t i = 0; i < wheels.size(); ++i) wheelCommands_[i] = wheelCommandFromHub(hub_, wheels[i]);
    if (!motionDrivetrain_) drivetrain_->step(wheelCommands_, static_cast<float>(dt));

    const auto states = drivetrain_->wheelStates();
    for (std::size_t i = 0; i < wheels.size() && i < states.size(); ++i) {
        const WheelSpec& w = wheels[i];
        const WheelState& st = states[i];
        currentA += std::abs(st.driveMotorCurrentA) + std::abs(st.steerMotorCurrentA);

        MotorFeedback drive;
        drive.rotorPositionRot = radToRot(st.driveWheelAngleRad) * w.driveGearRatio;
        drive.rotorVelocityRps = radToRot(st.driveWheelOmegaRadPerSec) * w.driveGearRatio;
        drive.motorFamily = motorFamilyOf(w.driveMotor);
        for (int m = 0; m < std::clamp(w.driveMotorCount, 1, kMaxMotorsPerAxis); ++m)
            hub_.writeMotorFeedback({slot_owner::kDrivetrain, driveSlotName(w.name, m)}, drive);
        const SlotAddress steerSlot{slot_owner::kDrivetrain, steerSlotName(w.name)};
        if (hub_.isDeclared(steerSlot)) {   // only what the drivetrain declared: tank and mecanum have no steer slots
            MotorFeedback steer;
            steer.rotorPositionRot = radToRot(st.steerAngleRad) * w.steerGearRatio;
            steer.rotorVelocityRps = radToRot(st.steerOmegaRadPerSec) * w.steerGearRatio;
            steer.motorFamily = motorFamilyOf(w.steerMotor);
            hub_.writeMotorFeedback(steerSlot, steer);
            hub_.writeEncoder({slot_owner::kDrivetrain, steerEncoderSlotName(w.name)},
                              {radToRot(st.steerAngleRad), radToRot(st.steerOmegaRadPerSec)});
        }
    }
}

void SimulationSession::writeImu(double dt) {
    const Pose3 pose = world_->pose(chassis_.body);
    const Quat q = pose.rotation;
    ImuFeedback imu;
    imu.yawRad = q.yaw();
    imu.pitchRad = std::asin(std::clamp(2.f * (q.w * q.y - q.z * q.x), -1.f, 1.f));
    imu.rollRad = std::atan2(2.f * (q.w * q.x + q.y * q.z), 1.f - 2.f * (q.x * q.x + q.y * q.y));
    const Quat toBody = q.conjugate();
    imu.angularVelocityRadPerSec = toBody.rotate(world_->angularVelocity(chassis_.body));
    const Vec3 v = world_->linearVelocity(chassis_.body);
    const Vec3 accelWorld = dt > 0.0 ? (v - lastVelocity_) / static_cast<float>(dt) : Vec3{};
    lastVelocity_ = v;
    imu.accelerationMps2 = toBody.rotate(accelWorld - config_.sim.physics.gravity);  // proper acceleration (reads +g at rest)
    hub_.writeImu({slot_owner::kRobot, slot_owner::kRobotImuSlot}, imu);
}

void SimulationSession::updateBrownoutWarning() {
    const bool now = battery_.brownedOut();
    if (now == brownoutWarned_) return;
    brownoutWarned_ = now;
    if (now) {
        sessionWarnings_.push_back({Severity::Error, DeviceId{}, "brownout",
                                    "battery sagged below the brownout voltage (" + std::to_string(battery_.config().brownoutVoltageV) + " V)"});
        logger_->warn("session", "brownout at t=" + std::to_string(clock_.simTimeSeconds()) + " s");
    } else {
        std::erase_if(sessionWarnings_, [](const HubWarning& w) { return w.code == "brownout"; });
    }
}

void SimulationSession::tick(proto::SensorFrame& out, bool fullFrame) {
    const double dt = clock_.fixedStepSeconds();

    double mechanismCurrentA = 0.0;
    if (!motionOrder_.empty()) {
        if (motionDrivetrain_) drivetrain_->prepareMotionStep(static_cast<float>(dt));
        for (const auto i : motionOrder_) {
            std::optional<PlantState> motion;
            if (const auto source = motionSources_[i]) {
                motion = linkedMotion(motionAxes_[*source].state(), motionStarts_[*source],
                                      motionStarts_[i], motionRatios_[i]);
                if (!std::isfinite(motion->position) || !std::isfinite(motion->velocity)) {
                    logger_->error("motion_links", "Non-finite linked motion at " +
                                                       motionAxes_[i].info.address.toString());
                    motion = PlantState{motionAxes_[i].state().position, 0.0};
                }
            }
            motionAxes_[i].advance(*ctx_, dt, motion);
        }
    }
    for (std::size_t i = 0; i < mechanismInstances_.size(); ++i) {
        const auto& m = mechanismInstances_[i];
        if (!motionMechanisms_.empty() && motionMechanisms_[i])
            m->finishMotionTick(*ctx_, dt);
        else
            m->tick(*ctx_, dt);
        mechanismCurrentA += std::max(0.0, m->batteryCurrentA());
    }
    double drivetrainCurrentA = 0.0;
    stepDrivetrain(dt, drivetrainCurrentA);

    if (coupling_) coupling_->beforeStep(*world_, chassis_.body, chassis_.baseMass);
    world_->step(static_cast<float>(dt));
    if (coupling_) coupling_->afterStep();

    if (gameState_) gameState_->step(*ctx_, dt);
    if (module_ != nullptr) module_->onTick(*ctx_, dt);
    sensors_->step(gameState_.get(), hub_);

    writeImu(dt);
    hub_.setBatteryVoltage(battery_.update(options_.baselineCurrentA + mechanismCurrentA + drivetrainCurrentA));
    updateBrownoutWarning();

    out.tick = clock_.tickIndex();
    clock_.advance();
    out.simTimeUs = clock_.simTimeUs();
    out.stepUs = static_cast<std::uint32_t>(clock_.fixedStepUs());
    out.batteryCurrentA = battery_.currentA();
    hub_.buildSensorFrame(out, fullFrame);  // also sets batteryVoltageV

    vision_->step(clock_.simTimeUs(), world_->pose(chassis_.body), directory_.get(), rng_);
    out.vision = vision_->collectDue(clock_.simTimeUs());
}

std::shared_ptr<const ISimSnapshot> SimulationSession::buildSnapshot(bool paused, bool robotConnected,
                                                                     std::span<const HubWarning> extraWarnings) const {
    SnapshotBuilder builder;
    SimSnapshotData& d = builder.data();
    d.tick = clock_.tickIndex();
    d.simTimeUs = clock_.simTimeUs();
    d.paused = paused;
    d.robotConnected = robotConnected;

    d.chassis.name = "chassis";
    d.chassis.pose = world_->pose(chassis_.body);
    d.chassis.linearVelocity = world_->linearVelocity(chassis_.body);
    d.chassis.angularVelocity = world_->angularVelocity(chassis_.body);

    if (drivetrain_) {
        const auto& wheels = drivetrain_->spec().wheels;
        const auto states = drivetrain_->wheelStates();
        for (std::size_t i = 0; i < wheels.size() && i < states.size(); ++i) {
            WheelSample w;
            w.name = wheels[i].name;
            w.contactPoint = states[i].contactPoint;
            w.steerAngleRad = states[i].steerAngleRad;
            w.spinAngleRad = states[i].driveWheelAngleRad;
            w.normalLoadN = states[i].normalLoadN;
            w.slipRatio = states[i].slipRatio;
            w.slipAngleTan = states[i].slipAngleTan;
            w.airborne = states[i].airborne;
            d.wheels.push_back(std::move(w));
        }
    }
    for (const auto& m : mechanismInstances_) d.mechanisms.push_back(m->state());
    const auto devices = hub_.discovered();
    d.devices.assign(devices.begin(), devices.end());
    const auto hubWarnings = hub_.warnings();
    d.warnings.assign(hubWarnings.begin(), hubWarnings.end());
    d.warnings.insert(d.warnings.end(), sessionWarnings_.begin(), sessionWarnings_.end());
    const auto sensorWarnings = sensors_->warnings();
    d.warnings.insert(d.warnings.end(), sensorWarnings.begin(), sensorWarnings.end());
    d.warnings.insert(d.warnings.end(), extraWarnings.begin(), extraWarnings.end());
    d.batteryVoltageV = battery_.voltageV();
    d.driverStation = ds_;
    d.physicsStats = world_->stats();
    if (gameState_) gameState_->publish(builder);
    vision_->publish(builder);
    sensors_->publish(builder);
    return builder.build();
}


}  // namespace wpiemu::core::runtime
