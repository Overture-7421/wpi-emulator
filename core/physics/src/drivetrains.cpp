#include "wpiemu/core/physics/drivetrains.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/core/physics/chassis_builder.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/devices/device_hub.h"

namespace wpiemu::core::physics {
namespace {

using namespace sdk;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinHeadingLength = 1e-4f;

double finiteOrZero(double v) { return std::isfinite(v) ? v : 0.0; }

int driveMotorCount(const WheelSpec& w) { return std::clamp(w.driveMotorCount, 1, kMaxMotorsPerAxis); }

// Voltage of drive motor `i`: its own entry when the command lists motors (extras repeat motor 0).
double driveVoltage(const WheelCommand& cmd, int i) {
    if (cmd.driveMotors <= 0) return cmd.driveVoltageV;
    const int n = std::min(cmd.driveMotors, kMaxMotorsPerAxis);
    return cmd.driveMotorVoltagesV[static_cast<std::size_t>(i < n ? i : 0)];
}

bool isRoller(WheelKind kind) { return kind == WheelKind::Omni || kind == WheelKind::Mecanum; }

}  // namespace

TractionDrivetrain::WheelPolicy TractionDrivetrain::policyFor(Layout layout, const WheelSpec& wheel) {
    WheelPolicy p;
    p.tractionAngleRad = wheel.tire.tractionAngleRad;
    switch (layout) {
        case Layout::Swerve:
            p.steered = wheel.steered || wheel.kind == WheelKind::Swerve;
            break;
        case Layout::Tank:
            break;
        case Layout::Mecanum: {
            // WPILib X layout (+y left): FL and RR -45 deg, FR and RL +45 deg.
            if (p.tractionAngleRad == 0.f) {
                const float handedness = wheel.positionM.x * wheel.positionM.y;
                if (handedness > 0.f) p.tractionAngleRad = -0.25f * kPi;
                else if (handedness < 0.f) p.tractionAngleRad = 0.25f * kPi;
            }
            break;
        }
    }
    return p;
}

TractionDrivetrain::TractionDrivetrain(Layout layout, std::string kind, const DrivetrainBuildArgs& args,
                                       TractionOptions options)
    : kind_(std::move(kind)),
      spec_(&args.spec),
      chassis_(args.chassis),
      world_(&args.world),
      body_(args.chassisBody),
      tire_(&args.tire),
      options_(std::move(options)),
      mass_(chassisMassProperties(args.chassis)) {
    if (!options_.motorModels)
        options_.motorModels = [](const MotorSpec& s) { return std::make_unique<DcMotorModel>(s); };

    wheels_.reserve(spec_->wheels.size());
    for (const WheelSpec& ws : spec_->wheels) {
        Wheel w;
        w.spec = &ws;
        w.hubLocal = wheelHubLocal(chassis_, ws);
        w.policy = policyFor(layout, ws);
        w.tire = ws.tire;
        w.tire.tractionAngleRad = w.policy.tractionAngleRad;
        if (isRoller(ws.kind)) w.tire.lateralScale = std::min(w.tire.lateralScale, kRollerLateralScaleMax);
        w.driveMotor = options_.motorModels(ws.driveMotor);
        if (w.policy.steered) w.steerMotor = options_.motorModels(ws.steerMotor);
        wheels_.push_back(std::move(w));
    }
    states_.assign(wheels_.size(), WheelState{});
    ground_.assign(wheels_.size(), Ground{});
    groundedXY_.reserve(wheels_.size());
    groundedLoads_.reserve(wheels_.size());
    groundedIndex_.reserve(wheels_.size());
}

TractionDrivetrain::~TractionDrivetrain() = default;

std::vector<SlotSpec> TractionDrivetrain::slotSpecs() const {
    std::vector<SlotSpec> out;
    for (const Wheel& w : wheels_) {
        for (int i = 0; i < driveMotorCount(*w.spec); ++i) {
            SlotSpec drive;
            drive.name = driveSlotName(w.spec->name, i);
            drive.label = w.spec->name + " drive motor" + (i > 0 ? " " + std::to_string(i + 1) : std::string());
            drive.type = SlotType::Motor;
            drive.required = i == 0;
            drive.description = i == 0 ? "Drive motor of wheel '" + w.spec->name + "'"
                                       : "Additional drive motor on the gearbox of wheel '" + w.spec->name + "'";
            out.push_back(std::move(drive));
        }
        if (w.policy.steered) {
            SlotSpec steer;
            steer.name = w.spec->name + ".steer";
            steer.label = w.spec->name + " steer motor";
            steer.type = SlotType::Motor;
            steer.required = true;
            steer.description = "Azimuth motor of module '" + w.spec->name + "'";
            out.push_back(std::move(steer));
        }
    }
    return out;
}

void TractionDrivetrain::setChassisMass(const MassProperties& composed) {
    if (composed.massKg > 0.f && std::isfinite(composed.massKg)) mass_ = composed;
}

void TractionDrivetrain::reset() {
    for (Wheel& w : wheels_) {
        w.omega = 0.0;
        w.steerOmega = 0.0;
    }
    states_.assign(wheels_.size(), WheelState{});
    ground_.assign(wheels_.size(), Ground{});
    hasLastVelocity_ = false;
    lastVelocity_ = {};
    accelX_ = accelY_ = 0.f;
}

WheelDrive TractionDrivetrain::driveFor(const Wheel& w, const WheelCommand& cmd, double& supplyCurrentA) const {
    const WheelSpec& s = *w.spec;
    const double gear = std::max(s.driveGearRatio, 1e-6f);
    const int count = driveMotorCount(s);
    const double eta = std::clamp(static_cast<double>(s.driveEfficiency), 0.01, 1.0);
    const double motorOmega = gear * w.omega;
    const double nominal = std::max(1e-3, w.driveMotor->spec().nominalVoltageV);

    // Each motor has its own voltage (a reversed one fights the others); torque, damping and current add up.
    double torqueSum = 0.0, dampingSum = 0.0;
    bool limited = false;
    supplyCurrentA = 0.0;
    for (int i = 0; i < count; ++i) {
        const double voltageV = finiteOrZero(driveVoltage(cmd, i));
        double current = w.driveMotor->currentA(voltageV, motorOmega);
        double torque = w.driveMotor->torqueNm(voltageV, motorOmega);
        const double slope = w.driveMotor->torqueNm(voltageV, motorOmega + 1.0) - torque;  // d torque / d omega, < 0
        const double limit = s.driveCurrentLimitA;
        if (limit > 0.0 && std::fabs(current) > limit) {
            torque *= limit / std::fabs(current);
            current = std::copysign(limit, current);
            limited = true;
        } else {
            dampingSum += std::max(0.0, -slope);
        }
        torqueSum += torque;
        supplyCurrentA += std::max(0.0, voltageV * current / nominal);
    }

    WheelDrive d;
    d.omegaRadPerSec = static_cast<float>(w.omega);
    d.torqueNm = static_cast<float>(eta * gear * torqueSum);
    d.dampingNmS = static_cast<float>(dampingSum * eta * gear * gear);
    d.inertiaKgM2 = static_cast<float>(static_cast<double>(count) * options_.rotorInertiaKgM2 * gear * gear + s.inertiaKgM2);
    return d;
}

void TractionDrivetrain::stepSteer(Wheel& w, WheelState& st, double voltageV, float dt) const {
    const WheelSpec& s = *w.spec;
    const double gear = std::max(s.steerGearRatio, 1e-6f);
    const double eta = std::clamp(static_cast<double>(options_.steerEfficiency), 0.01, 1.0);
    const double motorOmega = gear * w.steerOmega;

    const double current = w.steerMotor->currentA(voltageV, motorOmega);
    const double torque = w.steerMotor->torqueNm(voltageV, motorOmega);
    const double slope = w.steerMotor->torqueNm(voltageV, motorOmega + 1.0) - torque;
    const double azimuthTorque = eta * gear * torque;
    const double damping = std::max(0.0, -slope) * eta * gear * gear;
    const double inertia = std::max(1e-9, s.steerInertiaKgM2 + options_.rotorInertiaKgM2 * gear * gear);

    const double ie = inertia + dt * damping;
    const double steerOmegaNext = w.steerOmega + dt * azimuthTorque / ie;
    if (std::isfinite(steerOmegaNext)) w.steerOmega = steerOmegaNext;
    st.steerAngleRad += w.steerOmega * dt;
    st.steerOmegaRadPerSec = w.steerOmega;
    const double nominal = std::max(1e-3, w.steerMotor->spec().nominalVoltageV);
    st.steerMotorCurrentA = std::max(0.0, voltageV * current / nominal);
}

void TractionDrivetrain::step(std::span<const WheelCommand> commands, float dt) {
    if (!(dt > 0.f) || !std::isfinite(dt) || wheels_.empty()) return;
    prepareMotionStep(dt);
    for (std::size_t i = 0; i < wheels_.size(); ++i)
        advanceWheel(i, i < commands.size() ? commands[i] : WheelCommand{}, dt);
}

std::vector<MotionAxisBinding> TractionDrivetrain::motionAxes() {
    std::vector<MotionAxisBinding> out;
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        const SlotAddress address{slot_owner::kDrivetrain, driveSlotName(wheels_[i].spec->name, 0)};
        MotionAxisInfo info{address, PlantKind::Angular, {}};
        for (int m = 0; m < driveMotorCount(*wheels_[i].spec); ++m)
            info.motorSlots.push_back({slot_owner::kDrivetrain, driveSlotName(wheels_[i].spec->name, m)});
        out.push_back(
            {std::move(info),
             [this, i] {
                 return PlantState{states_[i].driveWheelAngleRad,
                                   states_[i].driveWheelOmegaRadPerSec};
             },
             [this, i](SimContext& ctx, double dt, std::optional<PlantState> motion) {
                 advanceWheel(i, wheelCommandFromHub(ctx.devices(), *wheels_[i].spec),
                              static_cast<float>(dt), motion);
             }});
    }
    return out;
}

void TractionDrivetrain::prepareMotionStep(float dt) {
    const std::size_t n = wheels_.size();
    const bool bodyOk = world_->isValid(body_);

    const Pose3 pose = bodyOk ? world_->pose(body_) : Pose3{};
    const Quat rot = pose.rotation;
    const Vec3 up = rot.rotate({0.f, 0.f, 1.f});
    const Vec3 down = -up;

    // Acceleration produced by the tire forces (previous step), expressed in the chassis frame, low-passed.
    if (bodyOk) {
        const Vec3 v = world_->linearVelocity(body_);
        const Vec3 a = hasLastVelocity_ ? (v - lastVelocity_) / dt : Vec3{};
        lastVelocity_ = v;
        hasLastVelocity_ = true;
        const Vec3 aBody = rot.conjugate().rotate(a + Vec3{0.f, 0.f, options_.gravityMps2});
        const float tau = spec_->loadTransferTauS;
        const float alpha = tau > 0.f ? dt / (tau + dt) : 1.f;
        accelX_ += alpha * (aBody.x - accelX_);
        accelY_ += alpha * (aBody.y - accelY_);
    }

    // Pass 1: ground contact per wheel (rays only find the contact point and normal; no springs).
    groundedXY_.clear();
    groundedIndex_.clear();
    for (std::size_t i = 0; i < n; ++i) {
        Ground g;
        if (bodyOk) {
            const Wheel& w = wheels_[i];
            const Vec3 hub = pose.transformPoint(w.hubLocal);
            const RayHit hit = world_->rayCastClosest(hub, down, w.spec->radiusM + options_.groundRayExtraM, layer::Field);
            if (hit.hit) {
                g.contact = true;
                g.point = hit.point;
                g.normal = length(hit.normal) > 0.5f ? normalized(hit.normal) : up;
            }
        }
        ground_[i] = g;
        if (g.contact) {
            groundedIndex_.push_back(i);
            groundedXY_.push_back({wheels_[i].hubLocal.x - mass_.comLocal.x, wheels_[i].hubLocal.y - mass_.comLocal.y});
        }
    }

    // Quasi-static normal loads over the wheels that touch the ground.
    groundedLoads_.assign(groundedXY_.size(), 0.f);
    const float gNormal = options_.gravityMps2 * std::max(0.f, up.z);
    quasiStaticLoads(groundedXY_, mass_.massKg, chassis_.comHeightM + mass_.comLocal.z, accelX_, accelY_, gNormal,
                     groundedLoads_);
    wheelLoads_.assign(n, 0.f);
    for (std::size_t i = 0; i < groundedIndex_.size(); ++i)
        wheelLoads_[groundedIndex_[i]] = groundedLoads_[i];
}

void TractionDrivetrain::advanceWheel(std::size_t i, WheelCommand cmd, float dt,
                                      std::optional<PlantState> motion) {
    const std::size_t n = wheels_.size();
    const Quat rot = world_->isValid(body_) ? world_->pose(body_).rotation : Quat{};
    Wheel& w = wheels_[i];
    WheelState& st = states_[i];
    const double steerV = finiteOrZero(cmd.steerVoltageV);

    double supply = 0.0;
    WheelDrive drive;
    if (motion) {
        w.omega = motion->velocity;
        drive.omegaRadPerSec = static_cast<float>(w.omega);
        drive.inertiaKgM2 = w.spec->inertiaKgM2;
    } else {
        drive = driveFor(w, cmd, supply);
    }

    TireInput in;
    in.chassisShareMassKg = mass_.massKg / static_cast<float>(n);
    in.wheelRadiusM = w.spec->radiusM;
    in.dt = dt;
    in.prescribedMotion = motion.has_value();

    Vec3 axisLong, axisLat;
    bool contact = ground_[i].contact;
    const float load = wheelLoads_[i];
    if (contact) {
        const Vec3& nrm = ground_[i].normal;
        const float delta = static_cast<float>(st.steerAngleRad);
        const Vec3 fwd = rot.rotate({std::cos(delta), std::sin(delta), 0.f});
        Vec3 h = fwd - nrm * dot(fwd, nrm);
        if (length(h) < kMinHeadingLength) {
            contact = false;
        } else {
            h = normalized(h);
            const float phi = w.policy.tractionAngleRad;
            axisLong = h * std::cos(phi) + cross(nrm, h) * std::sin(phi);
            axisLat = cross(nrm, axisLong);
            const Vec3 vc = world_->pointVelocity(body_, ground_[i].point);
            in.vLongMps = dot(vc, axisLong);
            in.vLatMps = dot(vc, axisLat);
            in.normalLoadN = load;
        }
    }

    TireOutput out = tire_->solve(w.tire, in, drive);
    // A (third-party) tire model must never poison the wheel or the body: no force, keep the
    // previous spin.
    if (!std::isfinite(out.fLongN) || !std::isfinite(out.fLatN)) out.fLongN = out.fLatN = 0.f;
    if (!std::isfinite(out.omegaNext)) out.omegaNext = static_cast<float>(w.omega);
    if (!std::isfinite(out.slipRatio)) out.slipRatio = 0.f;
    if (!std::isfinite(out.slipAngleTan)) out.slipAngleTan = 0.f;

    st.airborne = !contact || !(load > 0.f);
    st.normalLoadN = contact ? load : 0.f;
    st.slipRatio = out.slipRatio;
    st.slipAngleTan = out.slipAngleTan;
    if (contact) {
        const Vec3 force = axisLong * out.fLongN + axisLat * out.fLatN;
        world_->applyForce(body_, force, ground_[i].point);
        st.contactPoint = ground_[i].point;
        st.force = force;
    } else {
        st.contactPoint = {};
        st.force = {};
    }

    w.omega = motion ? motion->velocity : out.omegaNext;
    if (motion)
        st.driveWheelAngleRad = motion->position;
    else
        st.driveWheelAngleRad += w.omega * dt;
    st.driveWheelOmegaRadPerSec = w.omega;
    st.driveMotorCurrentA = finiteOrZero(supply);

    if (w.policy.steered) stepSteer(w, st, steerV, dt);
}

Status registerDrivetrains(DrivetrainRegistry& registry) {
    if (auto s = registry.registerDrivetrain(
            drivetrain_kind::kSwerve, {"Swerve", "Four or more independently steered and driven modules"},
            [](const DrivetrainBuildArgs& a) { return std::make_unique<SwerveDrivetrain>(a); });
        !s.ok())
        return s;
    if (auto s = registry.registerDrivetrain(
            drivetrain_kind::kTank, {"Tank / differential", "Fixed-heading wheels, any number per side"},
            [](const DrivetrainBuildArgs& a) { return std::make_unique<TankDrivetrain>(a); });
        !s.ok())
        return s;
    return registry.registerDrivetrain(
        drivetrain_kind::kMecanum, {"Mecanum", "Four roller wheels with +-45 degree traction axes"},
        [](const DrivetrainBuildArgs& a) { return std::make_unique<MecanumDrivetrain>(a); });
}

}  // namespace wpiemu::core::physics
