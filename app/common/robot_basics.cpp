#include "common/robot_basics.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app {

namespace {

QString tr(const char* text) { return QCoreApplication::translate("RobotBasics", text); }

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

sdk::WheelSpec baseWheel(const RobotBasics& b, const char* name) {
    sdk::WheelSpec w;
    w.name = name;
    w.radiusM = static_cast<float>(b.wheelDiameterM / 2.0);
    w.driveMotor = findMotor(b.driveMotor) ? findMotor(b.driveMotor)->spec : sdk::motors::krakenX60();
    w.driveMotorCount = std::max(1, b.motorsPerWheel);
    w.driveGearRatio = static_cast<float>(b.driveGearRatio);
    return w;
}

void addSwerveWheels(const RobotBasics& b, std::vector<sdk::WheelSpec>& out) {
    const float x = static_cast<float>(b.frameLengthM) / 2.f - kModuleInsetM;
    const float y = static_cast<float>(b.frameWidthM) / 2.f - kModuleInsetM;
    struct Corner { const char* name; float sx, sy; };
    for (const Corner& c : {Corner{"fl", 1, 1}, Corner{"fr", 1, -1}, Corner{"bl", -1, 1}, Corner{"br", -1, -1}}) {
        sdk::WheelSpec w = baseWheel(b, c.name);
        w.kind = sdk::WheelKind::Swerve;
        w.steered = true;
        w.positionM = {c.sx * x, c.sy * y, w.radiusM};
        w.steerMotor = findMotor(b.steerMotor) ? findMotor(b.steerMotor)->spec : sdk::motors::neo550();
        w.steerGearRatio = static_cast<float>(b.steerGearRatio);
        out.push_back(std::move(w));
    }
}

void addTankWheels(const RobotBasics& b, std::vector<sdk::WheelSpec>& out) {
    const int n = std::max(2, b.wheelsPerSide);
    const float halfSpan = static_cast<float>(b.frameLengthM) / 2.f - kModuleInsetM;
    const float y = static_cast<float>(b.frameWidthM) / 2.f - kTankWheelInsetM;
    for (const char* side : {"left", "right"}) {
        const float sy = side[0] == 'l' ? 1.f : -1.f;
        for (int i = 0; i < n; ++i) {
            const std::string name = std::string(side) + std::to_string(i);
            sdk::WheelSpec w = baseWheel(b, name.c_str());
            w.kind = sdk::WheelKind::Standard;
            const float t = n == 1 ? 0.f : (static_cast<float>(i) / static_cast<float>(n - 1)) * 2.f - 1.f;
            w.positionM = {t * halfSpan, sy * y, w.radiusM};
            out.push_back(std::move(w));
        }
    }
}

void addMecanumWheels(const RobotBasics& b, std::vector<sdk::WheelSpec>& out) {
    const float x = static_cast<float>(b.frameLengthM) / 2.f - kModuleInsetM;
    const float y = static_cast<float>(b.frameWidthM) / 2.f - kTankWheelInsetM;
    constexpr float kQuarterPi = 0.78539816339f;
    // X layout: the roller traction axes of FL/BR point +45 deg, FR/BL point -45 deg.
    struct Corner { const char* name; float sx, sy, angle; };
    for (const Corner& c : {Corner{"fl", 1, 1, kQuarterPi}, Corner{"fr", 1, -1, -kQuarterPi},
                            Corner{"bl", -1, 1, -kQuarterPi}, Corner{"br", -1, -1, kQuarterPi}}) {
        sdk::WheelSpec w = baseWheel(b, c.name);
        w.kind = sdk::WheelKind::Mecanum;
        w.positionM = {c.sx * x, c.sy * y, w.radiusM};
        w.widthM = 0.05f;
        w.tire.tractionAngleRad = c.angle;
        w.tire.lateralScale = 0.15f;
        out.push_back(std::move(w));
    }
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
    static const std::vector<MotorChoice> catalog = {
        {QStringLiteral("kraken_x60"), QStringLiteral("Kraken X60"), sdk::motors::krakenX60()},
        {QStringLiteral("falcon_500"), QStringLiteral("Falcon 500"), sdk::motors::falcon500()},
        {QStringLiteral("neo"), QStringLiteral("NEO"), sdk::motors::neo()},
        {QStringLiteral("vortex"), QStringLiteral("NEO Vortex"), sdk::motors::vortex()},
        {QStringLiteral("neo550"), QStringLiteral("NEO 550"), sdk::motors::neo550()},
        {QStringLiteral("cim"), QStringLiteral("CIM"), sdk::motors::cim()},
        {QStringLiteral("mini_cim"), QStringLiteral("MiniCIM"), sdk::motors::miniCim()},
    };
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

int RobotBasics::wheelCount() const { return isTank() ? 2 * std::max(2, wheelsPerSide) : 4; }

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
    if (b.frameLengthM < 0.25 || b.frameLengthM > 1.5 || b.frameWidthM < 0.25 || b.frameWidthM > 1.5)
        add(sdk::Severity::Error, "frame", tr("The frame must be between 10 in and 59 in on each side."));
    else if (b.frameLengthM > 0.8128 || b.frameWidthM > 0.8128)
        add(sdk::Severity::Warning, "frame", tr("Frames over 32 in are larger than an FRC frame perimeter allows."));
    if (b.massKg < 5.0 || b.massKg > 120.0)
        add(sdk::Severity::Error, "mass", tr("The mass must be between 5 kg and 120 kg."));
    else if (b.massKg > kMaxLegalMassKg)
        add(sdk::Severity::Warning, "mass", tr("Over the 125 lb robot weight limit. Allowed in the simulator."));
    if (b.wheelDiameterM < 0.04 || b.wheelDiameterM > 0.25)
        add(sdk::Severity::Error, "wheel", tr("The wheel diameter must be between 1.6 in and 9.8 in."));
    if (b.driveGearRatio < 0.5 || b.driveGearRatio > 100.0)
        add(sdk::Severity::Error, "gearing", tr("The drive gear ratio must be between 0.5 and 100."));
    if (b.isSwerve() && (b.steerGearRatio < 0.5 || b.steerGearRatio > 200.0))
        add(sdk::Severity::Error, "steerGearing", tr("The steer gear ratio must be between 0.5 and 200."));
    if (b.motorsPerWheel < 1 || b.motorsPerWheel > 4)
        add(sdk::Severity::Error, "motors", tr("Use between 1 and 4 motors per wheel."));
    if (b.isTank() && (b.wheelsPerSide < 2 || b.wheelsPerSide > 4))
        add(sdk::Severity::Error, "wheels", tr("A tank drive has between 2 and 4 wheels per side."));
    if (!findMotor(b.driveMotor)) add(sdk::Severity::Error, "driveMotor", tr("Choose a drive motor."));
    if (b.isSwerve() && !findMotor(b.steerMotor)) add(sdk::Severity::Error, "steerMotor", tr("Choose a steer motor."));
    if (b.bumperThicknessM < 0.0 || b.bumperThicknessM > 0.15)
        add(sdk::Severity::Error, "bumper", tr("The bumper thickness must be between 0 and 5.9 in."));
    if (b.station < 1 || b.station > 3) add(sdk::Severity::Error, "station", tr("The driver station is 1, 2 or 3."));
    return out;
}

sdk::RobotConfig buildRobotConfig(const RobotBasics& b, const RobotBuildContext& ctx) {
    sdk::RobotConfig config;
    config.name = b.name.trimmed().toStdString();
    config.project.path = ctx.projectPath.toStdString();
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
        const char* color = b.alliance == Alliance::Red ? kRedBumper : kBlueBumper;
        chassis.primitives.push_back(bumper("bumper_front", {len / 2.f + t / 2.f, 0.f, z}, {t, wid + 2.f * t, kBumperHeightM}, color));
        chassis.primitives.push_back(bumper("bumper_back", {-(len / 2.f + t / 2.f), 0.f, z}, {t, wid + 2.f * t, kBumperHeightM}, color));
        chassis.primitives.push_back(bumper("bumper_left", {0.f, wid / 2.f + t / 2.f, z}, {len, t, kBumperHeightM}, color));
        chassis.primitives.push_back(bumper("bumper_right", {0.f, -(wid / 2.f + t / 2.f), z}, {len, t, kBumperHeightM}, color));
    }

    sdk::DrivetrainSpec& spec = config.drivetrain.spec;
    spec.kind = b.drivetrain;
    if (b.isSwerve()) addSwerveWheels(b, spec.wheels);
    else if (b.isMecanum()) addMecanumWheels(b, spec.wheels);
    else addTankWheels(b, spec.wheels);

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
            b.alliance = p.color == kBlueBumper ? Alliance::Blue : Alliance::Red;
        }
    }

    if (!spec.wheels.empty()) {
        const sdk::WheelSpec& w = spec.wheels.front();
        b.wheelDiameterM = static_cast<double>(w.radiusM) * 2.0;
        b.driveGearRatio = w.driveGearRatio;
        b.motorsPerWheel = std::max(1, w.driveMotorCount);
        if (const MotorChoice* m = findMotorBySpec(w.driveMotor)) b.driveMotor = m->key;
        if (w.steered) {
            b.steerGearRatio = w.steerGearRatio;
            if (const MotorChoice* m = findMotorBySpec(w.steerMotor)) b.steerMotor = m->key;
        }
        if (spec.kind == sdk::drivetrain_kind::kTank) b.wheelsPerSide = std::clamp(static_cast<int>(spec.wheels.size() / 2), 2, 4);
    }
    return b;
}

}  // namespace wpiemu::app
