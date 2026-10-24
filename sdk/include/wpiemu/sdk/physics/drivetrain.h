#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/devices/device_hub.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/mechanisms/motion_link.h"
#include "wpiemu/sdk/physics/chassis_spec.h"
#include "wpiemu/sdk/physics/physics_world.h"
#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::sdk {

// Per-wheel actuation for ONE step, taken from the mapped motor slots (MotorOutput, already in the wheel frame:
// positive spins the wheel forward). A motor whose controller regulates current (MotorOutput::hasTorqueCurrent)
// carries its torque current instead; the drivetrain resolves it against the motor it simulates at that motor's
// present speed, since the controller's own voltage report only fits the controller's motor model.
struct WheelCommand {
    double driveVoltageV = 0.0;   // every drive motor, unless `driveMotorVoltagesV` is filled
    double steerVoltageV = 0.0;   // swerve only
    // Individual drive motors (index = driveSlotName order). Used when `driveMotors` > 0; motors beyond it
    // repeat motor 0.
    std::array<double, kMaxMotorsPerAxis> driveMotorVoltagesV{};
    int driveMotors = 0;
    // Torque-current commands, per drive motor and for the steer motor; `has...` false means use the voltage.
    std::array<double, kMaxMotorsPerAxis> driveTorqueCurrentsA{};
    std::array<bool, kMaxMotorsPerAxis> driveHasTorqueCurrent{};
    double steerTorqueCurrentA = 0.0;
    bool steerHasTorqueCurrent = false;
};

// Reads the wheel's drive/steer motor slots (owner = slot_owner::kDrivetrain). Every drive slot up to
// WheelSpec::driveMotorCount contributes its own command; unmapped ones mirror drive motor 0.
inline WheelCommand wheelCommandFromHub(const IActuatorReader& hub, const WheelSpec& wheel) {
    WheelCommand cmd;
    const int count = std::clamp(wheel.driveMotorCount, 1, kMaxMotorsPerAxis);
    for (int i = 0; i < count; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const MotorOutput out = hub.motorOutput({slot_owner::kDrivetrain, driveSlotName(wheel.name, i)});
        if (out.valid) {
            cmd.driveMotorVoltagesV[k] = out.voltageV;
            cmd.driveTorqueCurrentsA[k] = out.torqueCurrentA;
            cmd.driveHasTorqueCurrent[k] = out.hasTorqueCurrent;
        } else if (i > 0) {
            cmd.driveMotorVoltagesV[k] = cmd.driveMotorVoltagesV[0];
            cmd.driveTorqueCurrentsA[k] = cmd.driveTorqueCurrentsA[0];
            cmd.driveHasTorqueCurrent[k] = cmd.driveHasTorqueCurrent[0];
        }
    }
    cmd.driveMotors = count;
    cmd.driveVoltageV = cmd.driveMotorVoltagesV[0];
    const MotorOutput steer = hub.motorOutput({slot_owner::kDrivetrain, steerSlotName(wheel.name)});
    cmd.steerVoltageV = steer.valid ? steer.voltageV : 0.0;
    cmd.steerTorqueCurrentA = steer.valid ? steer.torqueCurrentA : 0.0;
    cmd.steerHasTorqueCurrent = steer.valid && steer.hasTorqueCurrent;
    return cmd;
}

// Per-wheel result after IDrivetrain::step, used to feed motor encoders / CANcoders / snapshots.
struct WheelState {
    double driveWheelAngleRad = 0.0;      // accumulated wheel rotation (rolling distance / radius)
    double driveWheelOmegaRadPerSec = 0.0;
    double driveMotorCurrentA = 0.0;      // total over driveMotorCount (battery model input)
    double steerAngleRad = 0.0;           // module azimuth relative to the chassis; 0 for fixed wheels
    double steerOmegaRadPerSec = 0.0;
    double steerMotorCurrentA = 0.0;
    float normalLoadN = 0.f;
    float slipRatio = 0.f;
    float slipAngleTan = 0.f;
    bool airborne = false;
    Vec3 contactPoint;                    // world frame (valid when !airborne)
    Vec3 force;                           // world-frame traction force applied this step
};

// Everything a drivetrain needs at construction (constructor injection; non-owning refs must
// outlive the drivetrain, which the core runtime guarantees by owning them in that order).
struct DrivetrainBuildArgs {
    const DrivetrainSpec& spec;
    const ChassisSpec& chassis;
    IPhysicsWorld& world;
    BodyId chassisBody;
    const ITireModel& tire;
};

// Turns wheel commands into traction forces on the chassis body.
//
// step() contract (called on the SIM THREAD once per tick, BEFORE IPhysicsWorld::step):
//   1. per wheel: ray-cast the ground contact point, compute quasi-static normal load,
//   2. motor model(s) -> WheelDrive, ITireModel::solve -> forces,
//   3. world.applyForce(chassisBody, force, contactPoint),
//   4. advance wheel spin state.
// It never calls world.step(). Kind-specific kinematics (swerve module azimuth control, tank
// skid, mecanum rollers) live entirely inside the implementation.
class IDrivetrain {
    WPIEMU_INTERFACE(IDrivetrain)
public:
    virtual std::string_view kind() const = 0;
    virtual const DrivetrainSpec& spec() const = 0;
    // Motors ("<wheel>.drive[N]" / steerSlotName) and optional azimuth position sensor (steerEncoderSlotName),
    // owner = slot_owner::kDrivetrain. Must equal DrivetrainInfo::slotSpecs for this spec.
    virtual std::vector<SlotSpec> slotSpecs() const = 0;

    // commands.size() == spec().wheels.size(), same order.
    virtual void step(std::span<const WheelCommand> commands, float dtSeconds) = 0;
    virtual std::span<const WheelState> wheelStates() const = 0;
    // Back to rest state (zero wheel spin); the chassis body pose is reset by the caller.
    virtual void reset() = 0;
    virtual std::vector<MotionAxisBinding> motionAxes() { return {}; }
    // Contact preparation only. Axis callbacks then advance each wheel once before world.step().
    virtual void prepareMotionStep(float /*dtSeconds*/) {}
};

struct DrivetrainInfo {
    std::string displayName;
    std::string description;
    // The slots a drivetrain of this kind declares for `spec`. Pure (no world or body needed), so the validator and the
    // editor can ask the registry; it must match IDrivetrain::slotSpecs() of the instance built from the same spec. Null =
    // the traction convention (tractionSlotSpecs with steering).
    std::function<std::vector<SlotSpec>(const DrivetrainSpec&)> slotSpecs;
};

// The slots of a drivetrain whose wheels are driven by one or more motors each: "<wheel>.drive[N]" (the first required)
// and, when `steerable`, "<wheel>.steer" (required) and "<wheel>.steerEncoder" for every steered wheel.
inline std::vector<SlotSpec> tractionSlotSpecs(const DrivetrainSpec& spec, bool steerable) {
    std::vector<SlotSpec> out;
    for (const WheelSpec& w : spec.wheels) {
        for (int i = 0; i < std::clamp(w.driveMotorCount, 1, kMaxMotorsPerAxis); ++i) {
            SlotSpec drive;
            drive.name = driveSlotName(w.name, i);
            drive.label = w.name + " drive" + (i > 0 ? " " + std::to_string(i + 1) : std::string());
            drive.type = SlotType::Motor;
            drive.required = i == 0;
            drive.description = i == 0 ? "Motor controller driving wheel '" + w.name + "'."
                                       : "Additional motor controller on the gearbox of wheel '" + w.name + "'.";
            drive.countKey = w.name + ".driveMotorCount";
            drive.ordinal = i + 1;
            out.push_back(std::move(drive));
        }
        if (steerable && isSteered(w)) {
            SlotSpec steer;
            steer.name = steerSlotName(w.name);
            steer.label = w.name + " steer";
            steer.type = SlotType::Motor;
            steer.description = "Motor controller steering module '" + w.name + "'.";
            out.push_back(std::move(steer));
            SlotSpec encoder;
            encoder.name = steerEncoderSlotName(w.name);
            encoder.label = w.name + " steer encoder";
            encoder.type = SlotType::PositionSensor;
            encoder.required = false;
            encoder.description = "Azimuth encoder of module '" + w.name + "'.";
            out.push_back(std::move(encoder));
        }
    }
    return out;
}

// The slots `spec` declares for its kind: the registered callback, else the traction convention (also for an unknown kind,
// which the validator reports separately).
inline std::vector<SlotSpec> drivetrainSlotSpecs(const DrivetrainInfo* info, const DrivetrainSpec& spec) {
    return info != nullptr && info->slotSpecs ? info->slotSpecs(spec) : tractionSlotSpecs(spec, true);
}

struct DrivetrainEntry {
    std::string kind;
    DrivetrainInfo info;
};

// Metadata and slot conventions of the built-in drivetrain kinds ("swerve", "tank", "mecanum"), shared by
// core registration, the editor's catalog and schema fallbacks.
inline std::vector<DrivetrainEntry> builtinDrivetrainInfos() {
    const auto traction = [](bool steerable) {
        return [steerable](const DrivetrainSpec& spec) { return tractionSlotSpecs(spec, steerable); };
    };
    return {
        {drivetrain_kind::kSwerve,
         {"Swerve", "Four or more independently steered and driven modules", traction(true)}},
        {drivetrain_kind::kTank,
         {"Tank / differential", "Fixed-heading wheels, any number per side", traction(false)}},
        {drivetrain_kind::kMecanum,
         {"Mecanum", "Four roller wheels with +-45 degree traction axes", traction(false)}},
    };
}

// "swerve", "tank", "mecanum", ... Add a drivetrain type = register a factory.
class DrivetrainRegistry : public Registry<DrivetrainInfo, IDrivetrain, const DrivetrainBuildArgs&> {
public:
    Status registerDrivetrain(std::string kind, DrivetrainInfo info, Factory factory) {
        return add(std::move(kind), std::move(info), std::move(factory));
    }
};

}  // namespace wpiemu::sdk
