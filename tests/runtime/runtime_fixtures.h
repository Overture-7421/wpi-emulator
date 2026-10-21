#pragma once

// Shared fixtures of the tests/runtime suite: a sample robot config, tiny mechanism / drivetrain / tire fakes, and a
// registry bundle, all built on the sdk/testing fakes so nothing depends on the physics or mechanisms modules.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_hub.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/tire_model.h"
#include "wpiemu/sdk/testing/device_fixtures.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

namespace wpiemu::test_runtime {

using namespace wpiemu::sdk;

// roller: motor (required) + encoder (optional). omega = 10 rad/s per volt, draws 2 A per volt.
class TestRoller final : public IMechanism {
public:
    explicit TestRoller(const MechanismBuildContext& ctx) : id_(ctx.config.id), type_(ctx.config.type) {}
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return type_; }
    void tick(SimContext& ctx, double dt) override {
        const MotorOutput out = ctx.devices().motorOutput({id_, "motor"});
        omega_ = out.valid ? out.voltageV * 10.0 : 0.0;
        current_ = std::abs(out.voltageV) * 2.0;
        position_ += omega_ * dt;
        ctx.devices().writeEncoder({id_, "encoder"}, EncoderFeedback{position_ / kTwoPi, omega_ / kTwoPi});
        ++ticks;
    }
    void reset() override {
        position_ = 0.0;
        omega_ = 0.0;
        current_ = 0.0;
        ++resets;
    }
    MechanismState state() const override {
        MechanismState s;
        s.id = id_;
        s.type = type_;
        s.active = std::abs(omega_) > 1.0;
        s.values.push_back({"position", position_, "rad"});
        return s;
    }
    double batteryCurrentA() const override { return current_; }
    int ticks = 0;
    int resets = 0;

private:
    std::string id_, type_;
    double position_ = 0.0, omega_ = 0.0, current_ = 0.0;
};

inline MechanismTemplateInfo rollerInfo() {
    MechanismTemplateInfo info;
    info.type = "roller";
    info.displayName = "Roller";
    info.slotSpecs = {{"motor", "Motor", SlotType::Motor, true, "", "motorCount", 1},
                      {"motor2", "Motor 2", SlotType::Motor, false, "", "motorCount", 2},
                      {"encoder", "Encoder", SlotType::PositionSensor, false, ""}};
    PropertySpec motors;
    motors.key = "motorCount";
    motors.type = PropertyType::Int;
    motors.defaultValue = 1;
    motors.min = 1;
    motors.max = 2;
    info.properties.properties.push_back(motors);
    PropertySpec gear;
    gear.key = "gearRatio";
    gear.type = PropertyType::Double;
    gear.defaultValue = 1.0;
    gear.min = 0.1;
    gear.max = 100.0;
    info.properties.properties.push_back(gear);
    info.links.push_back({"tip", "Tip", Pose3{Vec3{0.f, 0.f, 0.3f}, Quat{}}, ""});
    return info;
}

// Drivetrain fake: pushes the chassis forward with the summed drive voltage and reports 3 A per volt per wheel.
class TestDrivetrain final : public IDrivetrain {
public:
    explicit TestDrivetrain(const DrivetrainBuildArgs& a)
        : spec_(&a.spec), world_(&a.world), body_(a.chassisBody), states_(a.spec.wheels.size()) {}
    std::string_view kind() const override { return "test"; }
    const DrivetrainSpec& spec() const override { return *spec_; }
    std::vector<SlotSpec> slotSpecs() const override { return tractionSlotSpecs(*spec_, true); }
    void step(std::span<const WheelCommand> commands, float dt) override {
        float total = 0.f;
        for (std::size_t i = 0; i < commands.size() && i < states_.size(); ++i) {
            const double v = commands[i].driveVoltageV;
            total += static_cast<float>(v);
            states_[i].driveWheelOmegaRadPerSec = v;
            states_[i].driveWheelAngleRad += v * dt;
            states_[i].driveMotorCurrentA = std::abs(v) * 3.0;
            states_[i].steerAngleRad += commands[i].steerVoltageV * dt;
            states_[i].steerOmegaRadPerSec = commands[i].steerVoltageV;
        }
        const Pose3 pose = world_->pose(body_);
        world_->applyForceToCenter(body_, pose.transformVector(Vec3{total * 10.f, 0.f, 0.f}));
        ++steps;
    }
    std::span<const WheelState> wheelStates() const override { return states_; }
    void reset() override {
        std::fill(states_.begin(), states_.end(), WheelState{});
        ++resets;
    }
    int steps = 0;
    int resets = 0;

private:
    const DrivetrainSpec* spec_;
    IPhysicsWorld* world_;
    BodyId body_;
    std::vector<WheelState> states_;
};

class TestTire final : public ITireModel {
public:
    std::string_view id() const override { return "test"; }
    TireOutput solve(const TireParams&, const TireInput&, const WheelDrive&) const override { return {}; }
};

// Registries + plugin set shared by the tests.
struct Registries {
    MechanismFactory mechanisms;
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    GamePluginRegistry plugins;
    core::config::PluginRegistrar registrar{mechanisms};
    DrivetrainRegistry* drivetrainsPtr() { return &drivetrains; }
    std::shared_ptr<testing::FakeGameModule::Counters> fakeCounters;

    explicit Registries(bool withFakePlugin = true) {
        (void)mechanisms.registerTemplate(rollerInfo(), [](const MechanismBuildContext& c) { return std::make_unique<TestRoller>(c); });
        (void)drivetrains.registerDrivetrain("test", {"Test", "", nullptr}, [](const DrivetrainBuildArgs& a) { return std::make_unique<TestDrivetrain>(a); });
        (void)tires.registerModel("test", [] { return std::make_unique<TestTire>(); });
        if (withFakePlugin) {
            auto module = std::make_unique<testing::FakeGameModule>("fake_game", 2999);
            fakeCounters = module->counters();
            (void)plugins.add(std::move(module));
        }
    }
};

// A game module with one AprilTag (id 3) 3 m ahead of the spawn point, 1 m high, facing back at the robot, on a 10 m x 6 m
// field. Select it with RobotConfig::gamePluginId = "tag_game".
inline constexpr float kTagFieldLengthM = 10.f;
inline constexpr float kTagFieldWidthM = 6.f;

class TagGameModule final : public testing::FakeGameModule {
public:
    TagGameModule() : FakeGameModule("tag_game", 2998) {}
    AprilTagLayout aprilTagLayout() const override {
        AprilTagLayout layout;
        layout.name = "tag-layout";
        layout.fieldLengthM = kTagFieldLengthM;
        layout.fieldWidthM = kTagFieldWidthM;
        AprilTag tag;
        tag.id = 3;
        tag.pose = Pose3{Vec3{3.f, 0.f, 1.f}, Quat::fromYaw(static_cast<float>(kPi))};
        layout.tags.push_back(tag);
        return layout;
    }
};

// A camera on the chassis looking along +X from 1 m above the floor.
inline CameraConfig frontCamera(const std::string& id = "front") {
    CameraConfig cam;
    cam.id = id;
    cam.name = id;
    cam.local = Pose3{Vec3{0.f, 0.f, 0.8f}, Quat{}};
    cam.fps = 200.0;
    cam.latencyMs = 20.0;
    return cam;
}

inline WheelSpec wheel(const std::string& name, float x, float y) {
    WheelSpec w;
    w.name = name;
    w.positionM = Vec3{x, y, 0.f};
    return w;
}

// 4-wheel "test" drivetrain, one roller "intake" with its motor mapped, an IMU mapped on robot.
inline RobotConfig sampleConfig() {
    RobotConfig c;
    c.name = "Sample";
    c.project.path = "C:/robot";
    c.project.language = "java";
    c.project.wpilibYear = 2999;
    c.project.gradleRioVersion = "2999.1.1";
    c.chassis.massKg = 50.f;
    c.sim.physics.gravity = Vec3{};  // weightless: the fake world has no real contact
    c.drivetrain.spec.kind = "test";
    c.drivetrain.spec.tireModel = "test";
    c.drivetrain.spec.wheels = {wheel("fl", 0.3f, 0.3f), wheel("fr", 0.3f, -0.3f), wheel("bl", -0.3f, 0.3f), wheel("br", -0.3f, -0.3f)};
    int id = 1;
    for (const WheelSpec& w : c.drivetrain.spec.wheels)
        c.drivetrain.mappings.push_back({{slot_owner::kDrivetrain, w.name + ".drive"}, DeviceKey{"phoenix6", "talonfx", id++, "rio", -1}});

    MechanismConfig intake;
    intake.id = "intake";
    intake.type = "roller";
    intake.label = "Intake";
    intake.mount = Pose3{Vec3{0.4f, 0.f, 0.1f}, Quat{}};
    intake.mappings.push_back({{"intake", "motor"}, DeviceKey{"phoenix6", "talonfx", 10, "rio", -1}});
    intake.mappings.push_back({{"intake", "encoder"}, DeviceKey{"phoenix6", "cancoder", 12, "rio", -1}});
    c.mechanisms.push_back(intake);
    c.robotMappings.push_back({{slot_owner::kRobot, "imu"}, DeviceKey{"phoenix6", "pigeon2", 20, "rio", -1}});
    return c;
}

// A tank drivetrain with three wheels a side (left0..2, right0..2); only the middle wheels' drive slots are mapped (talons 1, 2).
inline RobotConfig sixWheelTank() {
    RobotConfig c;
    c.drivetrain.spec.kind = drivetrain_kind::kTank;
    for (const int side : {1, -1})
        for (int i = 0; i < 3; ++i) {
            WheelSpec w;
            w.name = std::string(side > 0 ? "left" : "right") + std::to_string(i);
            w.positionM = {0.2f * static_cast<float>(i - 1), 0.3f * static_cast<float>(side), w.radiusM};
            c.drivetrain.spec.wheels.push_back(w);
        }
    c.drivetrain.mappings = {
        {{slot_owner::kDrivetrain, "left1.drive"}, {"phoenix6", "talonfx", 1, "", -1}},
        {{slot_owner::kDrivetrain, "right1.drive"}, {"phoenix6", "talonfx", 2, "", -1}}};
    return c;
}

// A scratch directory under the current working directory (never the system temp dir), removed on destruction.
class ScratchDir {
public:
    ScratchDir() {
        static std::mt19937_64 gen{std::random_device{}()};
        path_ = std::filesystem::current_path() / ("wpiemu-test-scratch-" + std::to_string(gen() & 0xFFFFFFull));
        std::filesystem::create_directories(path_);
    }
    ~ScratchDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    ScratchDir(const ScratchDir&) = delete;
    ScratchDir& operator=(const ScratchDir&) = delete;
    const std::filesystem::path& path() const { return path_; }

    // Writes `content` to `relative` below the scratch dir (creating directories); returns the full path.
    std::filesystem::path write(const std::string& relative, const std::string& content) const {
        const std::filesystem::path p = path_ / relative;
        std::filesystem::create_directories(p.parent_path());
        std::ofstream out(p, std::ios::binary);
        out << content;
        return p;
    }
    static std::string read(const std::filesystem::path& p) {
        std::ifstream in(p, std::ios::binary);
        std::ostringstream s;
        s << in.rdbuf();
        return s.str();
    }

private:
    std::filesystem::path path_;
};

inline proto::DeviceAnnounce cancoderAnnounce(std::uint32_t runtimeId, int canId) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{runtimeId};
    a.descriptor.family = "phoenix6";
    a.descriptor.type = "cancoder";
    a.descriptor.id = canId;
    a.descriptor.bus = kBusUnknown;
    a.descriptor.capabilities = Capability::PositionSensor | Capability::VelocitySensor;
    a.signalSpecs = {SignalSpec{"position", SignalDirection::Input, SignalType::Double, "rot"},
                     SignalSpec{"velocity", SignalDirection::Input, SignalType::Double, "rps"}};
    return a;
}

// Announces the devices of sampleConfig(): talonfx 1..4 (drivetrain, runtime ids 101..104), talonfx 10 (intake, 105),
// cancoder 12 (intake encoder, 106: signals 0 position, 1 velocity), pigeon2 20 (107).
inline std::vector<proto::DeviceAnnounce> sampleDevices() {
    std::vector<proto::DeviceAnnounce> out;
    std::uint32_t rid = 101;
    for (int can : {1, 2, 3, 4, 10}) out.push_back(sdk::testing::talonFxAnnounce(rid++, can));
    out.push_back(cancoderAnnounce(rid++, 12));
    out.push_back(sdk::testing::pigeon2Announce(rid++, 20, "rad"));
    return out;
}

}  // namespace wpiemu::test_runtime
