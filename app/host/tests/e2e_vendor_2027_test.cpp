#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <locale>
#include <map>
#include <sstream>
#include <string>

#include "test_support.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

#ifndef WPIEMU_SEASON_BINARY_DIR
#define WPIEMU_SEASON_BINARY_DIR ""
#endif
#ifndef WPIEMU_SEASON_EXTENSION
#define WPIEMU_SEASON_EXTENSION ""
#endif
#ifndef WPIEMU_SEASON_SAMPLES
#define WPIEMU_SEASON_SAMPLES ""
#endif

namespace {

std::map<std::string, double> numericFields(const std::string& line, const char* marker) {
    std::map<std::string, double> values;
    const std::size_t start = line.find(marker);
    if (start == std::string::npos) return values;
    std::istringstream fields(line.substr(start + std::char_traits<char>::length(marker)));
    fields.imbue(std::locale::classic());
    std::string field;
    while (fields >> field) {
        const std::size_t equal = field.find('=');
        if (equal == std::string::npos) continue;
        std::istringstream number(field.substr(equal + 1));
        number.imbue(std::locale::classic());
        double value = 0.0;
        number >> value;
        if (number && number.peek() == std::char_traits<char>::eof())
            values[field.substr(0, equal)] = value;
    }
    return values;
}

bool hasDevice(const sdk::ISimSnapshot& snapshot, const char* type, int id, bool mapped) {
    return std::any_of(snapshot.devices().begin(), snapshot.devices().end(),
                       [&](const auto& device) {
                           return device.live && device.descriptor.type == type &&
                                  device.descriptor.id == id && (!mapped || device.mapped);
                       });
}

sdk::RobotConfig configForMappedPlant() {
    sdk::RobotConfig config;
    config.name = "2027 Phoenix vendor e2e";
    config.project.language = "java";
    config.project.wpilibYear = 2027;
    config.project.gradleRioVersion = "2027.0.0-alpha-7";

    sdk::MechanismConfig axis;
    axis.id = "phoenix_plant";
    axis.type = "mechanism";
    axis.label = "Phoenix 2027 mapped plant";
    axis.properties["motion"] = "rotary";
    axis.properties["motorModel"] = "kraken_x60";
    axis.properties["motorCount"] = 1;
    axis.properties["gearRatio"] = 10.0;
    axis.properties["efficiency"] = 1.0;
    axis.properties["momentOfInertiaKgM2"] = 0.04;
    axis.properties["simulateGravity"] = false;
    axis.mappings.push_back({{axis.id, "motor"}, phoenixKey("talonfx", 1), false});
    axis.mappings.push_back({{axis.id, "encoder"}, phoenixKey("cancoder", 2)});
    config.mechanisms.push_back(std::move(axis));
    return config;
}

}  // namespace

TEST_CASE("e2e: real Phoenix 6 2027 devices configure, drive a mapped plant, and report feedback") {
    if (env("WPIEMU_E2E").empty()) {
        MESSAGE(
            "SKIPPED: set WPIEMU_E2E=1; requires JDK 25 and the real 2027 Phoenix desktop "
            "artifacts");
        return;
    }

    const fs::path extension = core::launcher::locateHalsimExtension(
        WPIEMU_SEASON_BINARY_DIR, &sdk::readEnvironment, 2027, WPIEMU_SEASON_EXTENSION);
    if (extension.empty()) {
        MESSAGE(
            "SKIPPED: build with WPIEMU_SEASONS=2026;2027 or set WPIEMU_EXTENSION to the 2027 "
            "extension");
        return;
    }

    Scratch scratch("wpiemu-phoenix-2027-e2e-");
    const fs::path project = scratch.root / "robot";
    copyProject(fs::path(WPIEMU_SEASON_SAMPLES) / "vendor_java", project);

    TestRegistries registries;
    app::host::Options options;
    options.projectDir = project;
    options.scratchDir = scratch.root / "scratch";
    options.extensionLibrary = extension;
    options.driverStation.enabled = true;
    options.driverStation.mode = sdk::proto::RobotMode::Teleop;
    options.handshakeTimeout = 8min;
    options.extraGradleArgs = {"--no-daemon"};
    if (!env("WPIEMU_E2E_JAVA_HOME").empty()) options.javaHome = env("WPIEMU_E2E_JAVA_HOME");
    if (env("WPIEMU_E2E_LAUNCH") == "direct")
        options.launchMode = core::launcher::LaunchMode::ExternalDirect;

    auto started =
        app::host::RobotSession::start(registries.view(), configForMappedPlant(), options);
    REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
    app::host::RobotSession& session = *started.value();
    std::vector<std::string> console;
    std::map<std::string, double> sample;
    bool configReadback = false;
    bool robotConfigReadback = false;
    bool teleop = false;
    bool disabled = false;
    bool sampleAfterDisabled = false;
    sdk::SessionStatus status;

    const bool completed = sdk::testing::waitUntil(
        [&] {
            for (const auto& entry :
                 drainConsole(session, console, !env("WPIEMU_E2E_VERBOSE").empty())) {
                if (entry.text.find("CTRE27_TELEOP") != std::string::npos) teleop = true;
                if (entry.text.find("CTRE27_DISABLED") != std::string::npos) disabled = true;
                if (entry.text.find("CTRE27_CONFIG") != std::string::npos &&
                    entry.text.find("motorInverted=Clockwise_Positive") != std::string::npos &&
                    entry.text.find("encoderDirection=Clockwise_Positive") != std::string::npos)
                    robotConfigReadback = true;
                const auto fields = numericFields(entry.text, "CTRE27_CONFIG ");
                if (const auto offset = fields.find("encoderOffsetRot"); offset != fields.end())
                    robotConfigReadback &= std::abs(offset->second - 0.125) < 1e-5;
                const auto values = numericFields(entry.text, "CTRE27_SAMPLE ");
                if (!values.empty()) {
                    sample = values;
                    sampleAfterDisabled = disabled;
                }
                if (entry.text.find("bridge.vendor") != std::string::npos &&
                    entry.text.find("config: cw+=true") != std::string::npos &&
                    entry.text.find("magnetOffsetRot=0.125") != std::string::npos)
                    configReadback = true;
            }
            status = session.status();
            const auto snapshot = session.latestSnapshot();
            return (teleop && robotConfigReadback && configReadback && sample.size() == 7 &&
                    sample["rotorPosition"] > 0.1 && snapshot &&
                    snapshot->simTimeUs() >= 1'000'000) ||
                   !status.active();
        },
        8min, 20ms);

    const auto snapshot = session.latestSnapshot();
    if (!completed || !status.active() || !teleop || !robotConfigReadback || !configReadback ||
        sample.size() != 7 || !snapshot) {
        dumpConsole(console);
    }
    REQUIRE_MESSAGE(completed, status.detail);
    REQUIRE_MESSAGE(status.active(), status.detail);
    REQUIRE(teleop);
    REQUIRE(robotConfigReadback);
    REQUIRE_MESSAGE(configReadback,
                    "Phoenix bridge did not read back the firmware TalonFX/CANcoder configuration");
    REQUIRE(snapshot != nullptr);
    REQUIRE(session.runner().robotHello());
    CHECK(session.runner().robotHello()->wpilibYear == 2027);
    CHECK(snapshot->simTimeUs() >= 1'000'000);
    CHECK(hasDevice(*snapshot, "talonfx", 1, true));
    CHECK(hasDevice(*snapshot, "cancoder", 2, true));
    CHECK(hasDevice(*snapshot, "pigeon2", 3, false));

    REQUIRE(sample.size() == 7);
    CHECK(sample.contains("duty"));
    CHECK(sample.contains("volts"));
    CHECK(sample.contains("rotorPosition"));
    CHECK(sample.contains("rotorVelocity"));
    CHECK(sample.contains("encoderPosition"));
    CHECK(sample.contains("encoderVelocity"));
    CHECK(sample.contains("yaw"));
    CHECK(sample["duty"] > 0.1);
    CHECK(sample["volts"] > 1.0);
    CHECK(sample["rotorPosition"] > 0.1);
    CHECK(sample["rotorVelocity"] > 0.01);
    CHECK(sample["encoderPosition"] < -0.01);
    CHECK(std::abs(sample["encoderVelocity"]) > 0.01);
    CHECK(std::abs(sample["yaw"] - 22.5) < 2.0);

    sdk::proto::DsState disabledDs = snapshot->driverStation();
    disabledDs.enabled = false;
    session.controller().setDriverStation(disabledDs);
    const bool settled = sdk::testing::waitUntil(
        [&] {
            for (const auto& entry :
                 drainConsole(session, console, !env("WPIEMU_E2E_VERBOSE").empty())) {
                if (entry.text.find("CTRE27_DISABLED") != std::string::npos) disabled = true;
                const auto values = numericFields(entry.text, "CTRE27_SAMPLE ");
                if (!values.empty()) {
                    sample = values;
                    sampleAfterDisabled = disabled;
                }
            }
            const auto latest = session.latestSnapshot();
            if (!latest || !disabled || !sampleAfterDisabled || sample.size() != 7) return false;
            const auto axis =
                std::find_if(latest->mechanisms().begin(), latest->mechanisms().end(),
                             [](const auto& mechanism) { return mechanism.id == "phoenix_plant"; });
            if (axis == latest->mechanisms().end()) return false;
            const auto velocity = std::find_if(
                axis->values.begin(), axis->values.end(),
                [](const auto& value) { return value.name == sdk::value_name::kVelocity; });
            return velocity != axis->values.end() && std::abs(velocity->value) < 0.005 &&
                   std::abs(sample["rotorVelocity"]) < 0.01 &&
                   std::abs(sample["encoderVelocity"]) < 0.001;
        },
        30s, 20ms);
    REQUIRE_MESSAGE(settled,
                    "Phoenix feedback and mapped plant did not settle after disabling the robot");

    const auto settledSnapshot = session.latestSnapshot();
    REQUIRE(settledSnapshot != nullptr);
    const auto plant =
        std::find_if(settledSnapshot->mechanisms().begin(), settledSnapshot->mechanisms().end(),
                     [](const auto& mechanism) { return mechanism.id == "phoenix_plant"; });
    REQUIRE(plant != settledSnapshot->mechanisms().end());
    const auto physicalPosition =
        std::find_if(plant->values.begin(), plant->values.end(),
                     [](const auto& value) { return value.name == sdk::value_name::kPosition; });
    REQUIRE(physicalPosition != plant->values.end());
    const double outputRotations = physicalPosition->value / sdk::kTwoPi;

    // At rest, firmware-reported CANcoder position must equal the physical axis frame after the
    // configured clockwise direction and magnet offset have each been accounted for once.
    CHECK(outputRotations < -0.01);
    CHECK(std::abs(sample["encoderPosition"] - outputRotations) < 0.001);
    // The integrated sensor reports rotor-side rotations, with CTRE's CW-positive direction.
    CHECK(std::abs(sample["rotorPosition"] + 10.0 * outputRotations) < 0.005);
    CHECK(std::abs(sample["encoderVelocity"]) < 0.001);
    CHECK(std::abs(sample["rotorVelocity"]) < 0.01);

    const std::int64_t pid = session.runner().robotHello()->pid;
    session.requestStop();
    session.waitUntilFinished();
    CHECK_FALSE(processAlive(pid));
}

int main(int argc, char** argv) {
    if (env("WPIEMU_E2E").empty()) {
        std::puts("SKIPPED: set WPIEMU_E2E=1; requires JDK 25 and 2027 Phoenix desktop artifacts");
        return kSkipExitCode;
    }
    const auto extension = core::launcher::locateHalsimExtension(
        WPIEMU_SEASON_BINARY_DIR, &sdk::readEnvironment, 2027, WPIEMU_SEASON_EXTENSION);
    if (extension.empty()) {
        std::puts("SKIPPED: no matching 2027 HALSim extension is available");
        return kSkipExitCode;
    }
    return doctest::Context(argc, argv).run();
}
