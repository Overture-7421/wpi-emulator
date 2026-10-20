#pragma once

// PRIVATE to core/mechanisms: the shared building blocks of the generic mechanism templates.
//
//   AxisOptions   how a template configures one actuated degree of freedom (defaults, which
//                 properties/slots exist)
//   axisProperties / axisSlots / readAxisSettings
//                 property schema, slot specs and typed settings of one axis
//   MechanismAxis one DOF at runtime: motors -> voltage -> IPlant -> encoders / limit switches

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/core/mechanisms/motor_catalog.h"
#include "wpiemu/core/mechanisms/motor_drive.h"
#include "wpiemu/core/mechanisms/plants.h"
#include "wpiemu/core/mechanisms/sensor_models.h"
#include "wpiemu/sdk/config/property_builders.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/devices/device_hub.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/mechanisms/motion_link.h"

namespace wpiemu::core::mechanisms::detail {

enum class Motion { Rotary, Arm, Linear };

const char* motionKey(Motion m);

struct AxisOptions {
    Motion motion = Motion::Rotary;
    bool selectableMotion = false;   // expose a `motion` enum property
    const char* rotaryModel = plant_model::kFlywheel;  // plant key used when motion == Rotary
    bool limits = false;             // min/max/start properties
    bool limitSwitchSlots = false;   // forward/reverse limit switch slots (need limits)
    bool encoderSlots = true;        // position + velocity sensor slots
    bool motorRequired = true;       // primary motor slot is `required`

    // defaults
    std::string motorModel = "kraken_x60";
    int motorCount = 1;
    double gearRatio = 1.0;
    double efficiency = 1.0;
    bool brake = false;
    double inertiaKgM2 = 0.002;      // Rotary; Arm: 0 = uniform rod about the pivot
    double massKg = 2.0;             // Arm / Linear
    double lengthM = 0.3;            // Arm: pivot to tip
    double comFraction = 0.5;        // Arm: centre of mass as a fraction of lengthM
    double drumRadiusM = 0.02;       // Linear
    bool gravity = false;
    bool armGravityEnabled = true;
    bool linearGravityEnabled = true;
    double gravityAngleOffsetRad = 0.0;
    double viscousFrictionNmS = 0.0; // output-side damping default (N*m*s/rad or N*s/m)
    double minPosition = 0.0;
    double maxPosition = 0.0;
    double startPosition = 0.0;
};

struct AxisSlotNames {
    std::vector<std::string> motors;   // primary first
    std::string encoder;
    std::string velocity;
    std::string reverseLimit;          // lower stop
    std::string forwardLimit;          // upper stop
};
AxisSlotNames axisSlotNames();
std::vector<sdk::SlotSpec> axisSlots(const AxisOptions& options);
sdk::MotionAxisSpec axisMotionSpec(const AxisOptions& options);
std::vector<sdk::PropertySpec> axisProperties(const AxisOptions& options, const MotorCatalog& motors);

struct AxisSettings {
    Motion motion = Motion::Rotary;
    sdk::PlantSpec plant;
    double statorCurrentLimitA = 0.0;
    NeutralMode neutral = NeutralMode::Coast;
    bool reportMotorLimits = false;
    EncoderModel encoder;
    bool limited = false;
    double limitMargin = 1e-6;
    bool limitPressedValue = true;
    double startPosition = 0.0;
};

// Reads (and defaults) the properties of one axis; InvalidArgument names the offending property.
sdk::Result<AxisSettings> readAxisSettings(const AxisOptions& options, const sdk::Value& props,
                                           const MotorCatalog& motors);

// One DOF at runtime. Owns its plant; reads the motor slots of `owner`, writes the sensor slots.
class MechanismAxis {
public:
    MechanismAxis(std::string owner, AxisOptions options, AxisSettings settings, std::unique_ptr<sdk::IPlant> plant,
                  MotorDrive drive);

    void tick(sdk::SimContext& ctx, double dtSeconds);
    void reset();
    sdk::MotionAxisBinding motionBinding();

    double position() const { return plant_->state().position; }   // rad or m
    double velocity() const { return plant_->state().velocity; }
    bool atLowerLimit() const { return plant_->atLowerLimit(); }
    bool atUpperLimit() const { return plant_->atUpperLimit(); }

    // Mean voltage across the motors and total |motor current| of the last tick; battery draw >= 0.
    double appliedVoltageV() const { return appliedVoltageV_; }
    double motorCurrentA() const { return motorCurrentA_; }
    double batteryCurrentA() const { return batteryCurrentA_; }

    const AxisSettings& settings() const { return settings_; }
    const AxisSlotNames& slotNames() const { return slots_; }
    sdk::SlotAddress address(const std::string& slot) const { return {owner_, slot}; }

private:
    double outputRotations(double position) const;
    void writeFeedback(sdk::ISensorWriter& hub, double previousVelocity, double dt);

    std::string owner_;
    AxisOptions options_;
    AxisSettings settings_;
    AxisSlotNames slots_;
    std::unique_ptr<sdk::IPlant> plant_;
    MotorDrive drive_;
    RotorSensorModel rotor_;
    LimitSwitchModel reverseSwitch_;
    LimitSwitchModel forwardSwitch_;
    double appliedVoltageV_ = 0.0;
    double motorCurrentA_ = 0.0;
    double batteryCurrentA_ = 0.0;
};

}  // namespace wpiemu::core::mechanisms::detail
