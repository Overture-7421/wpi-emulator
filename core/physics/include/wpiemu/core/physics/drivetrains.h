#pragma once

// Traction drivetrains (docs/reference/physics.md): ground contact by ray, quasi-static normal loads, one ITireModel solve per
// wheel, forces applied to the chassis at the ground contact points. No suspension.
//
//   SwerveDrivetrain   per-wheel azimuth (steer motor dynamics) + drive
//   TankDrivetrain     fixed-heading wheels, any number per side (grouping is the caller's wiring)
//   MecanumDrivetrain  roller wheels, traction axis +-45 deg derived from the wheel position (X layout)
//
// Registered with the SDK registries by registerDrivetrains(); the tire model comes in through
// DrivetrainBuildArgs::tire.

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/core/physics/quasi_static_loads.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::core::physics {

// Implemented by objects that depend on the chassis mass properties composed with payloads
// (ChassisPhysicsCoupling pushes updates to it).
class IChassisMassAware {
    WPIEMU_INTERFACE(IChassisMassAware)
public:
    virtual void setChassisMass(const sdk::MassProperties& composed) = 0;
};

using MotorModelFactory = std::function<std::unique_ptr<sdk::IMotorModel>(const sdk::MotorSpec&)>;

struct TractionOptions {
    // Default: ideal DC motor (sdk::DcMotorModel). Inject another model (thermal, data table, a fake) here.
    MotorModelFactory motorModels;
    float gravityMps2 = sdk::kDefaultGravityMps2;   // gravity used to estimate the tire-force acceleration
    float rotorInertiaKgM2 = 5e-5f;     // per drive / steer motor rotor, reflected through the gearbox
    float steerEfficiency = 0.9f;
    float groundRayExtraM = 0.02f;      // ray length = wheel radius + this
};

class TractionDrivetrain : public sdk::IDrivetrain, public IChassisMassAware {
public:
    ~TractionDrivetrain() override;

    std::string_view kind() const override { return kind_; }
    const sdk::DrivetrainSpec& spec() const override { return *spec_; }
    std::vector<sdk::SlotSpec> slotSpecs() const override;
    void step(std::span<const sdk::WheelCommand> commands, float dtSeconds) override;
    std::span<const sdk::WheelState> wheelStates() const override { return states_; }
    void reset() override;
    std::vector<sdk::MotionAxisBinding> motionAxes() override;
    void prepareMotionStep(float dtSeconds) override;

    // Composed chassis mass (chassis + payloads); defaults to ChassisSpec mass at the body origin.
    void setChassisMass(const sdk::MassProperties& composed) override;
    const sdk::MassProperties& chassisMass() const { return mass_; }

protected:
    enum class Layout { Swerve, Tank, Mecanum };
    TractionDrivetrain(Layout layout, std::string kind, const sdk::DrivetrainBuildArgs& args, TractionOptions options);

private:
    struct WheelPolicy {
        bool steered = false;
        float tractionAngleRad = 0.f;
    };
    static WheelPolicy policyFor(Layout layout, const sdk::WheelSpec& wheel);

    struct Wheel {
        const sdk::WheelSpec* spec = nullptr;
        sdk::Vec3 hubLocal;
        WheelPolicy policy;
        sdk::TireParams tire;
        std::unique_ptr<sdk::IMotorModel> driveMotor;
        std::unique_ptr<sdk::IMotorModel> steerMotor;
        double omega = 0.0;
        double steerOmega = 0.0;
    };

    struct Ground {
        bool contact = false;
        sdk::Vec3 point;
        sdk::Vec3 normal;
    };

    sdk::WheelDrive driveFor(const Wheel& w, const sdk::WheelCommand& command, double& supplyCurrentA) const;
    void stepSteer(Wheel& w, sdk::WheelState& st, double voltageV, float dt) const;
    void advanceWheel(std::size_t index, sdk::WheelCommand command, float dt,
                      std::optional<sdk::PlantState> motion = std::nullopt);

    std::string kind_;
    const sdk::DrivetrainSpec* spec_;
    sdk::ChassisSpec chassis_;
    sdk::IPhysicsWorld* world_;
    sdk::BodyId body_;
    const sdk::ITireModel* tire_;
    TractionOptions options_;
    sdk::MassProperties mass_;

    std::vector<Wheel> wheels_;
    std::vector<sdk::WheelState> states_;
    std::vector<Ground> ground_;
    std::vector<WheelXY> groundedXY_;
    std::vector<float> groundedLoads_;
    std::vector<float> wheelLoads_;
    std::vector<std::size_t> groundedIndex_;

    sdk::Vec3 lastVelocity_;
    bool hasLastVelocity_ = false;
    float accelX_ = 0.f;  // low-passed tire-force acceleration, chassis frame
    float accelY_ = 0.f;
};

class SwerveDrivetrain final : public TractionDrivetrain {
public:
    explicit SwerveDrivetrain(const sdk::DrivetrainBuildArgs& args, TractionOptions options = {})
        : TractionDrivetrain(Layout::Swerve, sdk::drivetrain_kind::kSwerve, args, std::move(options)) {}
};

class TankDrivetrain final : public TractionDrivetrain {
public:
    explicit TankDrivetrain(const sdk::DrivetrainBuildArgs& args, TractionOptions options = {})
        : TractionDrivetrain(Layout::Tank, sdk::drivetrain_kind::kTank, args, std::move(options)) {}
};

class MecanumDrivetrain final : public TractionDrivetrain {
public:
    explicit MecanumDrivetrain(const sdk::DrivetrainBuildArgs& args, TractionOptions options = {})
        : TractionDrivetrain(Layout::Mecanum, sdk::drivetrain_kind::kMecanum, args, std::move(options)) {}
};

// Registers "swerve", "tank", "mecanum" (default options). Fails with AlreadyExists if a key is taken.
sdk::Status registerDrivetrains(sdk::DrivetrainRegistry& registry);

}  // namespace wpiemu::core::physics
