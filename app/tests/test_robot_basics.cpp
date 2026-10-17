#include <doctest/doctest.h>

#include <cmath>

#include "common/robot_basics.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"

using namespace wpiemu::app;
namespace sdk = wpiemu::sdk;

namespace {
const sdk::WheelSpec* findWheel(const sdk::RobotConfig& c, const std::string& name) {
    for (const auto& w : c.drivetrain.spec.wheels)
        if (w.name == name) return &w;
    return nullptr;
}
}  // namespace

TEST_CASE("swerve basics become four steered modules at the frame corners") {
    const RobotBasics b = defaultBasicsFor(sdk::drivetrain_kind::kSwerve);
    const sdk::RobotConfig c = buildRobotConfig(b, {});
    CHECK(c.drivetrain.spec.kind == sdk::drivetrain_kind::kSwerve);
    REQUIRE(c.drivetrain.spec.wheels.size() == 4);
    for (const auto& w : c.drivetrain.spec.wheels) {
        CHECK(w.steered);
        CHECK(w.kind == sdk::WheelKind::Swerve);
        CHECK(w.radiusM == doctest::Approx(0.0508));
        CHECK(w.positionM.z == doctest::Approx(w.radiusM));
        CHECK(std::abs(w.positionM.x) < c.chassis.sizeM.x / 2.f);
        CHECK(w.driveMotor.name == "Kraken X60");
        CHECK(w.steerMotor.name == "NEO 550");
    }
    const auto* fl = findWheel(c, "fl");
    const auto* br = findWheel(c, "br");
    REQUIRE(fl != nullptr);
    REQUIRE(br != nullptr);
    CHECK(fl->positionM.x > 0);
    CHECK(fl->positionM.y > 0);
    CHECK(br->positionM.x < 0);
    CHECK(br->positionM.y < 0);
    CHECK(c.chassis.massKg == doctest::Approx(54.0));
    CHECK(c.chassis.sizeM.x == doctest::Approx(0.6858));
}

TEST_CASE("tank basics produce wheels per side and skip steering") {
    RobotBasics b = defaultBasicsFor(sdk::drivetrain_kind::kTank);
    b.wheelsPerSide = 3;
    CHECK(b.wheelCount() == 6);
    const sdk::RobotConfig c = buildRobotConfig(b, {});
    REQUIRE(c.drivetrain.spec.wheels.size() == 6);
    int left = 0, right = 0;
    for (const auto& w : c.drivetrain.spec.wheels) {
        CHECK(!w.steered);
        (w.positionM.y > 0 ? left : right)++;
    }

    CHECK(left == 3);
    CHECK(right == 3);
    CHECK(findWheel(c, "left0") != nullptr);
    CHECK(findWheel(c, "right2") != nullptr);
    // wheels are spread front to back
    CHECK(findWheel(c, "left0")->positionM.x < findWheel(c, "left2")->positionM.x);
    CHECK(c.motionLinksConfigured);
    CHECK(c.motionLinks.size() == 4);
}

TEST_CASE("tank setup updates retain custom links and mappings needing reference repair") {
    auto basics = defaultBasicsFor(sdk::drivetrain_kind::kTank);
    basics.wheelsPerSide = 3;
    auto existing = buildRobotConfig(basics, {});
    existing.motionLinks[0].ratio = -2;
    existing.drivetrain.mappings.push_back(
        {{"drivetrain", "left2.drive"}, {"phoenix6", "talonfx", 1, "", -1}});
    basics.wheelsPerSide = 2;
    const auto updated = updateRobotConfig(&existing, basics, {});
    CHECK(updated.motionLinks.size() == existing.motionLinks.size());
    CHECK(updated.motionLinks[0].ratio == -2);
    CHECK(updated.drivetrain.mappings == existing.drivetrain.mappings);
    CHECK(updated.drivetrain.spec.wheels.size() == 4);
}

TEST_CASE("mecanum basics use opposing roller angles on diagonals") {
    const sdk::RobotConfig c = buildRobotConfig(defaultBasicsFor(sdk::drivetrain_kind::kMecanum), {});
    REQUIRE(c.drivetrain.spec.wheels.size() == 4);
    CHECK(findWheel(c, "fl")->tire.tractionAngleRad == doctest::Approx(findWheel(c, "br")->tire.tractionAngleRad));
    CHECK(findWheel(c, "fl")->tire.tractionAngleRad == doctest::Approx(-findWheel(c, "fr")->tire.tractionAngleRad));
    CHECK(findWheel(c, "fl")->kind == sdk::WheelKind::Mecanum);
    CHECK(findWheel(c, "fl")->tire.lateralScale < 0.5f);
}

TEST_CASE("bumpers follow thickness and alliance colour") {
    RobotBasics b;
    b.alliance = Alliance::Blue;
    b.bumperThicknessM = 0.08;
    sdk::RobotConfig c = buildRobotConfig(b, {});
    REQUIRE(c.chassis.primitives.size() == 4);
    for (const auto& p : c.chassis.primitives) {
        CHECK(p.collides);
        CHECK(p.color == "#2b5fd0");
    }
    CHECK(c.chassis.primitives[0].local.position.x == doctest::Approx(b.frameLengthM / 2 + 0.04));

    b.bumperThicknessM = 0.0;
    CHECK(buildRobotConfig(b, {}).chassis.primitives.empty());
}

TEST_CASE("project info, sim rate and the game plugin section come from the build context") {
    sdk::GamePluginRegistry plugins;
    REQUIRE(sdk::registerBuiltinGamePlugins(plugins).ok());
    RobotBuildContext ctx;
    ctx.projectPath = QStringLiteral("/p/robot");
    ctx.language = QStringLiteral("cpp");
    ctx.wpilibYear = 2026;
    ctx.gradleRioVersion = QStringLiteral("2026.1.1");
    ctx.stepUs = 2000;
    ctx.gameModule = plugins.findByYear(2026);
    REQUIRE(ctx.gameModule != nullptr);
    const sdk::RobotConfig c = buildRobotConfig(RobotBasics{}, ctx);
    CHECK(c.project.language == "cpp");
    CHECK(c.project.wpilibYear == 2026);
    CHECK(c.sim.stepUs == 2000);
    const sdk::PluginSection* section = c.findPlugin("game_2026");
    REQUIRE(section != nullptr);
    CHECK(section->data.isObject());
    CHECK(section->schemaVersion == ctx.gameModule->contributeConfigSchema().schemaVersion);

    ctx.gameModule = nullptr;   // unsupported season: core-only, no plugin section
    CHECK(buildRobotConfig(RobotBasics{}, ctx).plugins.empty());
}

TEST_CASE("basics round-trip through the config") {
    RobotBasics b = defaultBasicsFor(sdk::drivetrain_kind::kTank);
    b.name = QStringLiteral("Tanky");
    b.frameLengthM = 0.8;
    b.massKg = 50.0;
    b.wheelsPerSide = 4;
    b.driveMotor = QStringLiteral("cim");
    b.motorsPerWheel = 2;
    b.driveGearRatio = 10.71;
    b.bumperThicknessM = 0.07;
    b.alliance = Alliance::Blue;
    const RobotBasics back = basicsFromConfig(buildRobotConfig(b, {}));
    CHECK(back.name == b.name);
    CHECK(back.drivetrain == b.drivetrain);
    CHECK(back.frameLengthM == doctest::Approx(0.8));
    CHECK(back.massKg == doctest::Approx(50.0));
    CHECK(back.wheelsPerSide == 4);
    CHECK(back.driveMotor == QStringLiteral("cim"));
    CHECK(back.motorsPerWheel == 2);
    CHECK(back.driveGearRatio == doctest::Approx(10.71));
    CHECK(back.bumperThicknessM == doctest::Approx(0.07));
    CHECK(back.alliance == Alliance::Blue);
    CHECK(back.wheelDiameterM == doctest::Approx(b.wheelDiameterM));

    RobotBasics swerve = defaultBasicsFor(sdk::drivetrain_kind::kSwerve);
    swerve.steerMotor = QStringLiteral("neo");
    swerve.steerGearRatio = 21.4;
    const RobotBasics s = basicsFromConfig(buildRobotConfig(swerve, {}));
    CHECK(s.steerMotor == QStringLiteral("neo"));
    CHECK(s.steerGearRatio == doctest::Approx(21.4));
}

TEST_CASE("validation reports errors and legal-limit warnings in plain language") {
    CHECK(!hasErrors(validateBasics(RobotBasics{})));

    RobotBasics b;
    b.massKg = 2.0;
    b.frameLengthM = 3.0;
    b.name = QStringLiteral("  ");
    b.wheelDiameterM = 0.5;
    b.driveGearRatio = 0.0;
    const auto issues = validateBasics(b);
    CHECK(hasErrors(issues));
    int errors = 0;
    for (const auto& i : issues) {
        if (i.severity == sdk::Severity::Error) {
            ++errors;
            CHECK(!i.message.empty());
        }
    }
    CHECK(errors == 5);

    RobotBasics heavy;
    heavy.massKg = 80.0;
    const auto warn = validateBasics(heavy);
    CHECK(!hasErrors(warn));
    REQUIRE(warn.size() == 1);
    CHECK(warn[0].severity == sdk::Severity::Warning);

    RobotBasics tank = defaultBasicsFor(sdk::drivetrain_kind::kTank);
    tank.wheelsPerSide = 7;
    CHECK(hasErrors(validateBasics(tank)));
}

TEST_CASE("re-running the wizard keeps mechanisms, mappings and hand-made primitives") {
    sdk::RobotConfig existing = buildRobotConfig(defaultBasicsFor(sdk::drivetrain_kind::kSwerve), {});
    sdk::MechanismConfig m;
    m.id = "intake";
    m.type = "roller";
    existing.mechanisms.push_back(m);
    existing.robotMappings.push_back({{"robot", "imu"}, {"phoenix6", "pigeon2", 0, "rio", -1}});
    existing.drivetrain.mappings.push_back({{"drivetrain", "fl.drive"}, {"phoenix6", "talonfx", 1, "rio", -1}});
    existing.drivetrain.mappings.push_back({{"drivetrain", "zz.drive"}, {"phoenix6", "talonfx", 9, "rio", -1}});   // wheel that no longer exists
    sdk::PrimitiveSpec extra;
    extra.name = "arm_base";
    existing.chassis.primitives.push_back(extra);
    existing.battery.nominalVoltageV = 12.2;

    RobotBasics b = defaultBasicsFor(sdk::drivetrain_kind::kSwerve);
    b.massKg = 45.0;
    b.bumperThicknessM = 0.05;
    const sdk::RobotConfig merged = updateRobotConfig(&existing, b, RobotBuildContext{});
    CHECK(merged.chassis.massKg == doctest::Approx(45.0));
    CHECK(merged.mechanisms.size() == 1);
    CHECK(merged.robotMappings.size() == 1);
    REQUIRE(merged.drivetrain.mappings.size() == 1);
    CHECK(merged.drivetrain.mappings[0].slot.slot == "fl.drive");
    CHECK(merged.battery.nominalVoltageV == doctest::Approx(12.2));
    int bumpers = 0, extras = 0;
    for (const auto& p : merged.chassis.primitives) {
        if (p.name.rfind("bumper_", 0) == 0) ++bumpers;
        if (p.name == "arm_base") ++extras;
    }
    CHECK(bumpers == 4);
    CHECK(extras == 1);
}

TEST_CASE("motor catalog lookup") {
    REQUIRE(findMotor(QStringLiteral("kraken_x60")) != nullptr);
    CHECK(findMotor(QStringLiteral("nope")) == nullptr);
    CHECK(findMotorBySpec(sdk::motors::neo())->key == QStringLiteral("neo"));
    sdk::MotorSpec custom;
    custom.name = "Homebrew";
    CHECK(findMotorBySpec(custom) == nullptr);
}
