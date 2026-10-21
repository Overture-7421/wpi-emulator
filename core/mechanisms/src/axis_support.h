#pragma once

// PRIVATE to core/mechanisms: the building blocks of the generic mechanism template.
//
//   AxisOptions   the defaults of the one actuated degree of freedom
//   axisProperties / axisSlots / readAxisSettings
//                 property schema, slot specs and typed settings of the axis
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

// The defaults of the one core template's axis, plus the one per-instance adjustment, `linearGravityEnabled`.
struct AxisOptions {
    Motion motion = Motion::Rotary;  // default of the `motion` property

    std::string motorModel = "kraken_x60";
    int motorCount = 1;
    double gearRatio = 10.0;
    double efficiency = 1.0;
    bool brake = true;
    double inertiaKgM2 = 0.01;       // Rotary; Arm: 0 = uniform rod about the pivot
    double massKg = 2.0;             // Arm / Linear
    double lengthM = 0.3;            // Arm: pivot to tip
    double comFraction = 0.5;        // Arm: centre of mass as a fraction of lengthM
    double drumRadiusM = sdk::kDefaultDrumRadiusM;       // Linear
    bool gravity = true;             // acts on Arm and Linear only
    bool linearGravityEnabled = true;
    double viscousFrictionNmS = 0.001;   // bearing drag, so that a coasting rotary mechanism spins down
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
std::vector<sdk::SlotSpec> axisSlots();
sdk::MotionAxisSpec axisMotionSpec();
std::vector<sdk::PropertySpec> axisProperties(const AxisOptions& options, const MotorCatalog& motors);

struct AxisSettings {
    Motion motion = Motion::Rotary;
    sdk::PlantSpec plant;
    double statorCurrentLimitA = 0.0;
    NeutralMode neutral = NeutralMode::Coast;
    bool reportMotorLimits = false;
    EncoderModel encoder;
    bool limited = false;
    double limitMargin = kDefaultLimitMargin;
    bool limitPressedValue = true;
    double startPosition = 0.0;
};

// Reads (and defaults) the properties of one axis; InvalidArgument names the offending property.
sdk::Result<AxisSettings> readAxisSettings(const AxisOptions& options, const sdk::Value& props,
                                           const MotorCatalog& motors);

// One DOF at runtime. Owns its plant; reads the motor slots of `owner`, writes the sensor slots.
class MechanismAxis {
public:
    MechanismAxis(std::string owner, AxisSettings settings, std::unique_ptr<sdk::IPlant> plant, MotorDrive drive);

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
