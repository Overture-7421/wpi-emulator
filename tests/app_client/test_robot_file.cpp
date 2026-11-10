// The player's robot.json: what the picker reports for good files, broken files and robots the host will correct.
#include <doctest/doctest.h>

#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include <string>

#include "client/robot_file.h"
#include "match/match_test_support.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"

using namespace wpiemu;
using namespace wpiemu::app::client;

namespace {

ClientServices& services() {
    static std::unique_ptr<ClientServices> s = ClientServices::create().value();
    return *s;
}

sdk::RobotConfig robot() {
    sdk::RobotConfig spec = match_test::swerveSpec("Iron Owls");
    spec.gamePluginId = std::string(plugins::game_2026::pluginId());
    return spec;
}

}  // namespace

TEST_CASE("robot file: a good robot is accepted as written, with no corrections") {
    const RobotFileReport r = checkRobotJson(match_test::specJson(robot()), "mem", services());
    REQUIRE(r.ok);
    CHECK(r.errors.empty());
    CHECK(r.clamps.empty());
    CHECK(r.robotName == "Iron Owls");
    CHECK(r.wheelCount == 4);
    CHECK(r.gamePluginId == plugins::game_2026::pluginId());
    CHECK(r.gameKnown);
    REQUIRE(r.config.has_value());
    CHECK(r.config->drivetrain.spec.wheels.size() == 4);
}

TEST_CASE("robot file: a robot beyond plausible FRC limits is accepted and lists what the host will clamp") {
    sdk::RobotConfig heavy = robot();
    heavy.chassis.massKg = 5000.f;
    const RobotFileReport r = checkRobotJson(match_test::specJson(heavy), "mem", services());
    REQUIRE(r.ok);
    CHECK_FALSE(r.clamps.empty());
    bool mentionsMass = false;
    for (const std::string& line : r.clamps) mentionsMass = mentionsMass || line.find("mass") != std::string::npos;
    CHECK(mentionsMass);
    CHECK(r.config->chassis.massKg == doctest::Approx(5000.f));   // the file as written; the clamps happen in the intake
}

TEST_CASE("robot file: a robot without wheels is refused with the reason") {
    sdk::RobotConfig none = robot();
    none.drivetrain.spec.wheels.clear();
    const RobotFileReport r = checkRobotJson(match_test::specJson(none), "mem", services());
    CHECK_FALSE(r.ok);
    CHECK_FALSE(r.config.has_value());
    REQUIRE_FALSE(r.errors.empty());
    CHECK_FALSE(r.errors.front().empty());
}

TEST_CASE("robot file: text that is not a robot description is refused") {
    for (const char* text : {"", "not json", "[1, 2, 3]", "{\"schemaVersion\": 99}"}) {
        CAPTURE(text);
        const RobotFileReport r = checkRobotJson(text, "mem", services());
        CHECK_FALSE(r.ok);
        CHECK_FALSE(r.errors.empty());
    }
}

TEST_CASE("robot file: a file on disk is read, and a missing one is reported") {
    QTemporaryDir dir;
    REQUIRE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("robot.json"));
    QFile file(path);
    REQUIRE(file.open(QIODevice::WriteOnly));
    file.write(QByteArray::fromStdString(match_test::specJson(robot())));
    file.close();

    const RobotFileReport good = loadRobotFile(std::filesystem::path(path.toStdU16String()), services());
    CHECK(good.ok);
    CHECK_FALSE(good.path.empty());

    const RobotFileReport missing = loadRobotFile(std::filesystem::path(dir.filePath(QStringLiteral("nope.json")).toStdU16String()), services());
    CHECK_FALSE(missing.ok);
    CHECK_FALSE(missing.errors.empty());
}

TEST_CASE("robot file: a robot that names no game this build has is flagged, not refused") {
    sdk::RobotConfig other = robot();
    other.gamePluginId = "game_1999";
    const RobotFileReport r = checkRobotJson(match_test::specJson(other), "mem", services());
    CHECK(r.gamePluginId == "game_1999");
    CHECK_FALSE(r.gameKnown);
}
