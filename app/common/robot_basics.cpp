#include "common/robot_basics.h"

#include <algorithm>
#include <cmath>

#include "ui/qt_text.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"
#include "wpiemu/sdk/physics/wheel_layout.h"

namespace wpiemu::app {

namespace {

constexpr ui::Tr tr{"RobotBasics"};

constexpr float kFrameHeightM = 0.10f;
constexpr float kComHeightM = 0.20f;
constexpr float kGroundClearanceM = 0.04f;
constexpr float kBumperHeightM = 0.1397f;      // 5.5 in tall bumper zone
constexpr float kBumperCentreAboveFloorM = 0.1207f;
constexpr float kModuleInsetM = 0.0635f;       // swerve module centre from the frame edge
constexpr float kTankWheelInsetM = 0.04f;
constexpr double kMaxLegalMassKg = 56.7;        // 125 lb robot weight limit
constexpr const char* kRedBumper = "#c0392b";
constexpr const char* kBlueBumper = "#2b5fd0";

sdk::WheelSpec baseWheel(const RobotBasics& b) {
    sdk::WheelSpec w;
    w.radiusM = static_cast<float>(b.wheelDiameterM / 2.0);
    const MotorChoice* drive = findMotor(b.driveMotor);
    w.driveMotor = drive != nullptr ? drive->spec : sdk::motors::krakenX60();
    w.driveMotorCount = std::max(1, b.motorsPerWheel);
    w.driveGearRatio = static_cast<float>(b.driveGearRatio);
    return w;
}

std::vector<sdk::WheelSpec> drivetrainWheels(const RobotBasics& b) {
    sdk::WheelSpec base = baseWheel(b);
    const float halfLength = static_cast<float>(b.frameLengthM) / 2.f - kModuleInsetM;
    const float halfWidth = static_cast<float>(b.frameWidthM) / 2.f;
    if (b.isSwerve()) {
        const MotorChoice* steer = findMotor(b.steerMotor);
        base.steerMotor = steer != nullptr ? steer->spec : sdk::motors::neo550();
        base.steerGearRatio = static_cast<float>(b.steerGearRatio);
        return sdk::swerveWheels(base, halfLength, halfWidth - kModuleInsetM);
    }
    if (b.isMecanum()) return sdk::mecanumWheels(base, halfLength, halfWidth - kTankWheelInsetM);
    const int perSide = std::max(basics_limits::kWheelsPerSide.min, b.wheelsPerSide);
    return sdk::tankWheels(base, perSide, halfLength, halfWidth - kTankWheelInsetM);
}

sdk::PrimitiveSpec bumper(const char* name, sdk::Vec3 centre, sdk::Vec3 size, const char* color) {
    sdk::PrimitiveSpec p;
    p.name = name;
    p.kind = sdk::PrimitiveKind::Box;
    p.local = sdk::Pose3(centre, sdk::Quat{});
    p.sizeM = size;
    p.color = color;
    p.collides = true;
    return p;
}

}  // namespace

const std::vector<MotorChoice>& motorCatalog() {
    static const std::vector<MotorChoice> catalog = [] {
        std::vector<MotorChoice> choices;
        for (const sdk::MotorPreset& p : sdk::motors::presets())
            choices.push_back({QString::fromStdString(p.key), QString::fromStdString(p.label), p.spec});
        return choices;
    }();
    return catalog;
}

const MotorChoice* findMotor(const QString& key) {
    for (const auto& m : motorCatalog())
        if (m.key == key) return &m;
    return nullptr;
}

const MotorChoice* findMotorBySpec(const sdk::MotorSpec& spec) {
    for (const auto& m : motorCatalog())
        if (m.spec.name == spec.name) return &m;
    return nullptr;
}

int RobotBasics::wheelCount() const { return isTank() ? 2 * std::max(basics_limits::kWheelsPerSide.min, wheelsPerSide) : 4; }

const char* bumperColorFor(sdk::Alliance alliance) { return alliance == sdk::Alliance::Red ? kRedBumper : kBlueBumper; }

RobotBasics defaultBasicsFor(const std::string& kind) {
    RobotBasics b;
    b.drivetrain = kind;
    if (kind == sdk::drivetrain_kind::kTank) {
        b.frameLengthM = 0.7620;
        b.frameWidthM = 0.6604;
        b.massKg = 56.0;
        b.wheelsPerSide = 3;
        b.wheelDiameterM = 0.1524;     // 6 in
        b.driveMotor = QStringLiteral("neo");
        b.motorsPerWheel = 1;
        b.driveGearRatio = 8.45;
    } else if (kind == sdk::drivetrain_kind::kMecanum) {
        b.frameLengthM = 0.7112;
        b.frameWidthM = 0.7112;
        b.massKg = 54.0;
        b.wheelDiameterM = 0.1524;
        b.driveMotor = QStringLiteral("neo");
        b.driveGearRatio = 12.0;
    }
    return b;
}

bool hasErrors(const std::vector<sdk::Issue>& issues) {
    return std::any_of(issues.begin(), issues.end(), [](const sdk::Issue& i) { return i.severity == sdk::Severity::Error; });
}

std::vector<sdk::Issue> validateBasics(const RobotBasics& b) {
    std::vector<sdk::Issue> out;
    const auto add = [&out](sdk::Severity s, const char* path, const QString& message) {
        out.push_back({s, path, message.toStdString(), {}});
    };
    if (b.name.trimmed().isEmpty()) add(sdk::Severity::Error, "name", tr("Give the robot a name."));
    if (basics_limits::kFrameM.excludes(b.frameLengthM) || basics_limits::kFrameM.excludes(b.frameWidthM))
        add(sdk::Severity::Error, "frame", tr("The frame must be between 10 in and 59 in on each side."));
    else if (b.frameLengthM > 0.8128 || b.frameWidthM > 0.8128)
        add(sdk::Severity::Warning, "frame", tr("Frames over 32 in are larger than an FRC frame perimeter allows."));
    if (basics_limits::kMassKg.excludes(b.massKg))
        add(sdk::Severity::Error, "mass", tr("The mass must be between 5 kg and 120 kg."));
    else if (b.massKg > kMaxLegalMassKg)
        add(sdk::Severity::Warning, "mass", tr("Over the 125 lb robot weight limit. Allowed in the simulator."));
    if (basics_limits::kWheelDiameterM.excludes(b.wheelDiameterM))
        add(sdk::Severity::Error, "wheel", tr("The wheel diameter must be between 1.6 in and 9.8 in."));
    if (basics_limits::kDriveRatio.excludes(b.driveGearRatio))
        add(sdk::Severity::Error, "gearing", tr("The drive gear ratio must be between 0.5 and 100."));
    if (b.isSwerve() && basics_limits::kSteerRatio.excludes(b.steerGearRatio))
        add(sdk::Severity::Error, "steerGearing", tr("The steer gear ratio must be between 0.5 and 200."));
    if (basics_limits::kMotorsPerWheel.excludes(b.motorsPerWheel))
        add(sdk::Severity::Error, "motors", tr("Use between 1 and 4 motors per wheel."));
    if (b.isTank() && basics_limits::kWheelsPerSide.excludes(b.wheelsPerSide))
        add(sdk::Severity::Error, "wheels", tr("A tank drive has between 2 and 4 wheels per side."));
    if (!findMotor(b.driveMotor)) add(sdk::Severity::Error, "driveMotor", tr("Choose a drive motor."));
    if (b.isSwerve() && !findMotor(b.steerMotor)) add(sdk::Severity::Error, "steerMotor", tr("Choose a steer motor."));
    if (basics_limits::kBumperM.excludes(b.bumperThicknessM))
        add(sdk::Severity::Error, "bumper", tr("The bumper thickness must be between 0 and 5.9 in."));
    if (basics_limits::kStation.excludes(b.station)) add(sdk::Severity::Error, "station", tr("The driver station is 1, 2 or 3."));
    return out;
}

sdk::RobotConfig buildRobotConfig(const RobotBasics& b, const RobotBuildContext& ctx) {
    sdk::RobotConfig config;
    config.name = b.name.trimmed().toStdString();
    config.project.language = ctx.language.toStdString();
    config.project.wpilibYear = ctx.wpilibYear;
    config.project.gradleRioVersion = ctx.gradleRioVersion.toStdString();
    config.sim.stepUs = ctx.stepUs;

    sdk::ChassisSpec& chassis = config.chassis;
    chassis.sizeM = {static_cast<float>(b.frameLengthM), static_cast<float>(b.frameWidthM), kFrameHeightM};
    chassis.massKg = static_cast<float>(b.massKg);
    chassis.comHeightM = kComHeightM;
    chassis.groundClearanceM = kGroundClearanceM;

    const float t = static_cast<float>(b.bumperThicknessM);
    if (t > 0.f) {
        const float len = static_cast<float>(b.frameLengthM);
        const float wid = static_cast<float>(b.frameWidthM);
        const float z = kBumperCentreAboveFloorM - kComHeightM;   // chassis frame origin is the COM
        const char* color = bumperColorFor(b.alliance);
        chassis.primitives.push_back(bumper("bumper_front", {len / 2.f + t / 2.f, 0.f, z}, {t, wid + 2.f * t, kBumperHeightM}, color));
        chassis.primitives.push_back(bumper("bumper_back", {-(len / 2.f + t / 2.f), 0.f, z}, {t, wid + 2.f * t, kBumperHeightM}, color));
        chassis.primitives.push_back(bumper("bumper_left", {0.f, wid / 2.f + t / 2.f, z}, {len, t, kBumperHeightM}, color));
        chassis.primitives.push_back(bumper("bumper_right", {0.f, -(wid / 2.f + t / 2.f), z}, {len, t, kBumperHeightM}, color));
    }

    sdk::DrivetrainSpec& spec = config.drivetrain.spec;
    spec.kind = b.drivetrain;
    spec.wheels = drivetrainWheels(b);
    if (b.isTank()) {
        auto links = sdk::tankMotionLinks(config);
        if (links) config.motionLinks = std::move(links).value();
    }

    if (ctx.gameModule != nullptr) {
        const sdk::GameMetadata meta = ctx.gameModule->metadata();
        const sdk::PluginConfigSchema schema = ctx.gameModule->contributeConfigSchema();
        sdk::PluginSection& section = config.pluginSection(meta.id);
        section.schemaVersion = schema.schemaVersion;
        section.data = sdk::defaultsOf(schema.properties);
    }
    return config;
}

sdk::RobotConfig updateRobotConfig(const sdk::RobotConfig* existing, const RobotBasics& basics, const RobotBuildContext& context) {
    sdk::RobotConfig fresh = buildRobotConfig(basics, context);
    if (existing == nullptr) return fresh;

    sdk::RobotConfig out = *existing;
    out.name = fresh.name;
    out.project = fresh.project;
    out.sim.stepUs = fresh.sim.stepUs;

    out.chassis.sizeM = fresh.chassis.sizeM;
    out.chassis.massKg = fresh.chassis.massKg;
    std::vector<sdk::PrimitiveSpec> primitives;
    for (const auto& p : existing->chassis.primitives)
        if (p.name.rfind("bumper_", 0) != 0) primitives.push_back(p);
    for (const auto& p : fresh.chassis.primitives) primitives.push_back(p);
    out.chassis.primitives = std::move(primitives);

    const bool layoutChanged = [&] {
        const auto& before = existing->drivetrain.spec;
        const auto& after = fresh.drivetrain.spec;
        if (before.kind != after.kind || before.wheels.size() != after.wheels.size()) return true;
        for (std::size_t i = 0; i < before.wheels.size(); ++i)
            if (before.wheels[i].name != after.wheels[i].name) return true;
        return false;
    }();
    out.drivetrain.spec.kind = fresh.drivetrain.spec.kind;
    out.drivetrain.spec.wheels = fresh.drivetrain.spec.wheels;
    std::vector<sdk::SlotMapping> kept;
    for (const auto& m : existing->drivetrain.mappings) {
        for (const auto& w : out.drivetrain.spec.wheels) {
            const std::string prefix = w.name + ".";
            if (m.slot.slot.rfind(prefix, 0) == 0) {
                kept.push_back(m);
                break;
            }
        }
    }
    out.drivetrain.mappings = std::move(kept);
    if (layoutChanged) {
        // New wheels: wheel links are generated again (tank: one powered wheel per side).
        std::erase_if(out.motionLinks, [](const sdk::MotionLink& l) {
            return l.source.owner == sdk::slot_owner::kDrivetrain || l.target.owner == sdk::slot_owner::kDrivetrain;
        });
        if (basics.isTank()) {
            auto links = sdk::tankMotionLinks(out);
            if (!links) {
                out.drivetrain.mappings.clear();   // several mapped wheels on one side: start the mappings over
                links = sdk::tankMotionLinks(out);
            }
            if (links)
                for (auto& link : links.value()) out.motionLinks.push_back(std::move(link));
        }
    }

    for (auto& [id, section] : fresh.plugins)
        if (out.findPlugin(id) == nullptr) out.plugins[id] = section;
    return out;
}

RobotBasics basicsFromConfig(const sdk::RobotConfig& config) {
    const sdk::DrivetrainSpec& spec = config.drivetrain.spec;
    RobotBasics b = defaultBasicsFor(spec.kind.empty() ? sdk::drivetrain_kind::kSwerve : spec.kind);
    b.name = QString::fromStdString(config.name);
    b.frameLengthM = config.chassis.sizeM.x;
    b.frameWidthM = config.chassis.sizeM.y;
    b.massKg = config.chassis.massKg;

    b.bumperThicknessM = 0.0;
    for (const auto& p : config.chassis.primitives) {
        if (p.name == "bumper_front") {
            b.bumperThicknessM = p.sizeM.x;
            b.alliance = p.color == kBlueBumper ? sdk::Alliance::Blue : sdk::Alliance::Red;
        }
    }

    if (!spec.wheels.empty()) {
        const auto source =
            std::find_if(spec.wheels.begin(), spec.wheels.end(), [&](const auto& wheel) {
                return sdk::motionLinkTo(config, {sdk::slot_owner::kDrivetrain,
                                                  sdk::driveSlotName(wheel.name, 0)}) == nullptr;
            });
        const sdk::WheelSpec& w = source == spec.wheels.end() ? spec.wheels.front() : *source;
        b.wheelDiameterM = static_cast<double>(w.radiusM) * 2.0;
        b.driveGearRatio = w.driveGearRatio;
        b.motorsPerWheel = std::max(1, w.driveMotorCount);
        if (const MotorChoice* m = findMotorBySpec(w.driveMotor)) b.driveMotor = m->key;
        if (sdk::isSteered(w)) {
            b.steerGearRatio = w.steerGearRatio;
            if (const MotorChoice* m = findMotorBySpec(w.steerMotor)) b.steerMotor = m->key;
        }
        if (spec.kind == sdk::drivetrain_kind::kTank) b.wheelsPerSide = std::clamp(static_cast<int>(spec.wheels.size() / 2), 2, 4);
    }
    return b;
}

}  // namespace wpiemu::app
