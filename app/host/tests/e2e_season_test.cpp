#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>

#include "test_support.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

TEST_CASE("e2e: 2027 Java and C++ projects use the matching extension and receive teleop") {
    for (const char* language : {"java", "cpp"}) {
        CAPTURE(language);
        Scratch scratch("wpiemu-season-e2e-");
        copyProject(fs::path(WPIEMU_SEASON_SAMPLES) / language, scratch.root / "robot");
        TestRegistries registries;
        sdk::RobotConfig config;
        config.project.language = language;
        config.project.wpilibYear = 2027;
        app::host::Options options;
        options.projectDir = scratch.root / "robot";
        options.scratchDir = scratch.root / "scratch";
        options.extensionLibrary = core::launcher::locateHalsimExtension(
            WPIEMU_SEASON_BINARY_DIR, &sdk::readEnvironment, 2027, WPIEMU_SEASON_EXTENSION);
        options.driverStation.enabled = true;
        options.driverStation.mode = sdk::proto::RobotMode::Teleop;
        options.handshakeTimeout = 8min;
        options.extraGradleArgs = {"--no-daemon"};
        if (!env("WPIEMU_E2E_JAVA_HOME").empty()) options.javaHome = env("WPIEMU_E2E_JAVA_HOME");
        if (env("WPIEMU_E2E_LAUNCH") == "direct") options.launchMode = core::launcher::LaunchMode::ExternalDirect;
        auto started = app::host::RobotSession::start(registries.view(), config, options);
        REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
        auto& session = *started.value();
        std::vector<std::string> console;
        bool teleop = false;
        const bool ready = sdk::testing::waitUntil([&] {
            drainConsole(session, console, !env("WPIEMU_E2E_VERBOSE").empty());
            teleop |= std::any_of(console.begin(), console.end(), [](const auto& line) {
                return line.find("SEASON_SAMPLE teleop") != std::string::npos;
            });
            const auto snapshot = session.latestSnapshot();
            return (teleop && snapshot && snapshot->simTimeUs() >= 200000) || !session.status().active();
        }, 8min, 20ms);
        const auto snapshot = session.latestSnapshot();
        if (!ready || !teleop || !snapshot) dumpConsole(console);
        REQUIRE(ready);
        REQUIRE_MESSAGE(teleop, session.status().detail);
        REQUIRE(snapshot);
        CHECK(session.coreOnly());
        REQUIRE(session.runner().robotHello());
        CHECK(session.runner().robotHello()->wpilibYear == 2027);
        CHECK(snapshot->simTimeUs() >= 200000);
        CHECK(std::any_of(snapshot->devices().begin(), snapshot->devices().end(), [](const auto& device) {
            return device.live && device.descriptor.type == "pwm" && device.descriptor.id == 0 &&
                   device.descriptor.capabilities.has(sdk::Capability::MotorOutput);
        }));
        const auto pid = session.runner().robotHello()->pid;
        session.requestStop();
        session.waitUntilFinished();
        CHECK_FALSE(processAlive(pid));
    }
}

int main(int argc, char** argv) {
    if (env("WPIEMU_E2E").empty()) {
        std::puts("SKIPPED: set WPIEMU_E2E=1; requires JDK 25 and the 2027 native desktop toolchain");
        return kSkipExitCode;
    }
    const auto extension = core::launcher::locateHalsimExtension(
        WPIEMU_SEASON_BINARY_DIR, &sdk::readEnvironment, 2027, WPIEMU_SEASON_EXTENSION);
    if (extension.empty()) {
        std::puts("SKIPPED: build with WPIEMU_SEASONS=2026;2027 or set WPIEMU_EXTENSION to the 2027 extension");
        return kSkipExitCode;
    }
    return doctest::Context(argc, argv).run();
}
