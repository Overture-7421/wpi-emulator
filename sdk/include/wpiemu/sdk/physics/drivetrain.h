#pragma once

#include <algorithm>
#include <array>
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

// Per-wheel actuation for ONE step, taken from the mapped motor slots (MotorOutput::voltageV, already in the
// wheel frame: positive spins the wheel forward).
struct WheelCommand {
    double driveVoltageV = 0.0;   // every drive motor, unless `driveMotorVoltagesV` is filled
    double steerVoltageV = 0.0;   // swerve only
    // Individual drive motors (index = driveSlotName order). Used when `driveMotors` > 0; motors beyond it
    // repeat motor 0.
    std::array<double, kMaxMotorsPerAxis> driveMotorVoltagesV{};
    int driveMotors = 0;
};

// Reads the wheel's drive/steer motor slots (owner = slot_owner::kDrivetrain). Every drive slot up to
// WheelSpec::driveMotorCount contributes its own voltage; unmapped ones mirror drive motor 0.
inline WheelCommand wheelCommandFromHub(const IActuatorReader& hub, const WheelSpec& wheel) {
    WheelCommand cmd;
    const int count = std::clamp(wheel.driveMotorCount, 1, kMaxMotorsPerAxis);
    for (int i = 0; i < count; ++i) {
        const MotorOutput out = hub.motorOutput({slot_owner::kDrivetrain, driveSlotName(wheel.name, i)});
        if (out.valid)
            cmd.driveMotorVoltagesV[static_cast<std::size_t>(i)] = out.voltageV;
        else if (i > 0)
            cmd.driveMotorVoltagesV[static_cast<std::size_t>(i)] = cmd.driveMotorVoltagesV[0];
    }
    cmd.driveMotors = count;
    cmd.driveVoltageV = cmd.driveMotorVoltagesV[0];
    const MotorOutput steer = hub.motorOutput({slot_owner::kDrivetrain, wheel.name + ".steer"});
    cmd.steerVoltageV = steer.valid ? steer.voltageV : 0.0;
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
    // Motors ("<wheel>.drive[N]" / "<wheel>.steer") and optional azimuth position sensor
    // ("<wheel>.steerEncoder"), owner = slot_owner::kDrivetrain.
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
};

// "swerve", "tank", "mecanum", ... Add a drivetrain type = register a factory.
class DrivetrainRegistry : public Registry<DrivetrainInfo, IDrivetrain, const DrivetrainBuildArgs&> {
public:
    Status registerDrivetrain(std::string kind, DrivetrainInfo info, Factory factory) {
        return add(std::move(kind), std::move(info), std::move(factory));
    }
};

}  // namespace wpiemu::sdk
