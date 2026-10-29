# Drive day workshop: a match through the real robot process

**In 30 seconds:** now we stop placing the test chassis at the fence. The robot must drive from
its saved start pose and release the preload using normal PWM commands through HALSim.
The host advances a full 15-second autonomous and 135-second teleop, including endgame.

## 1. Use the existing host, not a second emulator

In the scratch export only, replace <code>app\host\tests\e2e_season_test.cpp</code> with the
following blocks. Its existing CMake target already links the host and built-in aggregators.
The old core-only season probe is being replaced for this experiment, not silently kept as
evidence for the plugin. The broader product should keep core-only coverage separately.

The helpers require HUD keys rather than returning success-shaped defaults.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "season_probe.h"
#include "test_support.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

double number(const sdk::ISimSnapshot& snapshot, const char* key) {
    const auto* value = snapshot.hudValue(key);
    REQUIRE_MESSAGE(value != nullptr, key);
    return value->asDouble();
}

std::string text(const sdk::ISimSnapshot& snapshot, const char* key) {
    const auto* value = snapshot.hudValue(key);
    REQUIRE_MESSAGE(value != nullptr, key);
    return value->asString();
}

}  // namespace
```

Load the actual saved JSON from the previous workshop, validate it through the real config
pipeline, and copy the corresponding ordinary WPILib project into an isolated temporary directory.
This proves the published model works; it does not reconstruct a different model inside the test.
The two languages deliberately cover opposite near-switch assignments.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
TEST_CASE("e2e: Power UP plugin runs a physical full match with Java and C++") {
    for (const char* language : {"java", "cpp"}) {
        const std::string assignment = std::string(language) == "java" ? "LRL" : "RLR";
        CAPTURE(language);
        CAPTURE(assignment);
        Scratch scratch("wpiemu-power-up-match-");
        copyProject(season::samples() / language, scratch.root / "robot");
        TestRegistries registries;
        core::config::FileConfigStore store(
            core::config::projectConfigPath(season::samples() / language),
            "Power UP book fixture", registries.json, registries.codec);
        const auto saved = store.load();
        REQUIRE_MESSAGE(saved.ok(), (saved.ok() ? "" : saved.error().message));
        auto config = saved.value();
        REQUIRE(config.project.language == language);
        REQUIRE(config.project.wpilibYear == season::year());
        const auto validated = registries.pipeline.load(registries.codec.toValue(config));
        REQUIRE_MESSAGE(validated.ok(), (validated.ok() ? "" : validated.error().message));
        REQUIRE(validated.value().valid());
        config = validated.value().config;
```

Start disabled and paused, with a sentinel FMS message. Seeing the plugin's assignment replace
that sentinel proves we are testing plugin-supplied data, not just forwarding a test string.
Time scale zero means unpaced, **not paused**; the explicit start-paused option makes stepping
controlled. An eight-minute timeout allows Gradle startup, not eight simulated minutes.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        app::host::Options options;
        options.projectDir = scratch.root / "robot";
        options.scratchDir = scratch.root / "scratch";
        options.extensionLibrary = core::launcher::locateHalsimExtension(
            season::binaryDir(), &sdk::readEnvironment, season::year(), season::buildExtension());
        options.driverStation.dsAttached = true;
        options.driverStation.fmsAttached = true;
        options.driverStation.station = sdk::proto::AllianceStation::Blue1;
        options.driverStation.gameSpecificMessage = "NOT_PLUGIN_DATA";
        options.timeScale = 0.0;
        options.startPaused = true;
        options.handshakeTimeout = 8min;
        options.extraGradleArgs = {"--no-daemon"};
        if (!env("WPIEMU_E2E_JAVA_HOME").empty()) options.javaHome = env("WPIEMU_E2E_JAVA_HOME");
```

The pump drains logs and waits for observable state. It uses wall time only for a test timeout,
never as game time. Require readiness, a connected robot, a paused snapshot and the real game
module before beginning. Otherwise a failed launch could look like a motionless passing robot.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        auto started = app::host::RobotSession::start(registries.view(), config, options);
        REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
        auto& session = *started.value();
        std::vector<std::string> console;
        auto pump = [&](auto timeout, auto predicate) {
            const bool ready = sdk::testing::waitUntil([&] {
                drainConsole(session, console, !env("WPIEMU_E2E_VERBOSE").empty());
                return predicate() || !session.status().active();
            }, timeout, 10ms);
            if (!ready || !session.status().active()) dumpConsole(console);
            return ready && session.status().active() && predicate();
        };
        REQUIRE(pump(8min, [&] {
            const auto snap = session.latestSnapshot();
            return session.status().programStarted && snap && snap->paused() && session.runner().robotHello();
        }));
        REQUIRE_FALSE(session.coreOnly());
        REQUIRE(session.gameModule());
        CHECK(session.gameModule()->metadata().id == "game_2027");
        CHECK(session.runner().robotHello()->wpilibYear == season::year());
        REQUIRE(session.latestSnapshot());
        CHECK(session.latestSnapshot()->driverStation().gameSpecificMessage == assignment);
```

## 2. Give the robot exactly one simulated second at a time

Two hundred step credits at 5 ms produce one second. Wait for the snapshot to reach the exact
target before granting more. The runner still permits only one outstanding robot tick, so this
does not bypass HAL acknowledgment or fabricate actuator frames.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        auto ds = options.driverStation;
        auto advance = [&](int seconds) {
            const auto target = session.latestSnapshot()->simTimeUs() + seconds * 1'000'000LL;
            sdk::proto::SimControl step;
            step.command = sdk::proto::SimCommand::Step;
            step.stepCount = static_cast<std::uint32_t>(seconds * 200);
            session.controller().control(step);
            REQUIRE(pump(120s, [&] {
                const auto snap = session.latestSnapshot();
                return snap && snap->simTimeUs() >= target;
            }));
            REQUIRE(session.latestSnapshot()->simTimeUs() == target);
        };
```

Let the disabled field settle, record the starting X and body budget, then enable autonomous.
Update remaining match time once per simulated second, as a host would. Each tick still runs
the entire mechanism, drivetrain, physics and HALSim loop.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        advance(1);
        const auto initial = session.latestSnapshot();
        const auto bodyBudget = initial->physicsStats().bodies;
        const auto startX = initial->chassis().pose.position.x;
        CHECK(number(*initial, "game_2027.inventory") == 1);
        CHECK(number(*initial, "game_2027.blue_score") == 0);

        ds.enabled = true;
        ds.mode = sdk::proto::RobotMode::Autonomous;
        for (int remaining = 15; remaining > 0; --remaining) {
            ds.matchTimeS = remaining;
            session.controller().setDriverStation(ds);
            advance(1);
        }
```

The robot must have moved more than 1.8 m, released exactly one cube, emptied inventory and
earned ownership points. A console message alone cannot pass these checks. Then insert a
disabled gap and require the score to stop changing.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        const auto autoEnd = session.latestSnapshot();
        CAPTURE(autoEnd->chassis().pose.position.x);
        CHECK(autoEnd->chassis().pose.position.x > startX + 1.8f);
        CHECK(number(*autoEnd, "game_2027.released") == 1);
        CHECK(number(*autoEnd, "game_2027.inventory") == 0);
        REQUIRE(number(*autoEnd, "game_2027.blue_score") > 5.0);
        CHECK(number(*autoEnd, "game_2027.red_score") == 0);
        CHECK(number(*autoEnd, "game_2027.auto_seconds") == doctest::Approx(15.0));
        const auto autoScore = number(*autoEnd, "game_2027.blue_score");
        ds.enabled = false;
        ds.matchTimeS = 0.0;
        session.controller().setDriverStation(ds);
        advance(2);
        CHECK(number(*session.latestSnapshot(), "game_2027.blue_score") == autoScore);
        CHECK(text(*session.latestSnapshot(), "game_2027.phase") == "Disabled");
```

## 3. Run teleop, not a timestamp jump

Advance all 135 seconds through the same locked-step loop, checking the last-thirty-second
endgame boundary. With one cube retaining switch ownership, teleop adds 135 points in this
continuous-rate approximation. Every published entity position must stay finite and body count
must remain bounded.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        ds.enabled = true;
        ds.mode = sdk::proto::RobotMode::Teleop;
        for (int remaining = 135; remaining > 0; --remaining) {
            ds.matchTimeS = remaining;
            session.controller().setDriverStation(ds);
            advance(1);
            if (remaining == 30)
                CHECK(text(*session.latestSnapshot(), "game_2027.phase") == "Endgame");
        }
        const auto final = session.latestSnapshot();
        const auto finalScore = number(*final, "game_2027.blue_score");
        CHECK(finalScore - autoScore == doctest::Approx(135.0).epsilon(0.0001));
        CHECK(number(*final, "game_2027.teleop_seconds") == doctest::Approx(135.0));
        CHECK(number(*final, "game_2027.endgame_seconds") == doctest::Approx(30.0));
        CHECK(final->physicsStats().bodies == bodyBudget);
        for (const auto& group : final->entityGroups())
            for (const auto& p : group.positions)
                CHECK((std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)));
```

Disable at the end and advance another three seconds. Confirm both robot mode callbacks appeared
and print the measured result. The sample holds position in teleop; this is a full-length
single-delivery test, not a multi-cycle autonomous or a six-robot match.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        ds.enabled = false;
        ds.matchTimeS = 0.0;
        session.controller().setDriverStation(ds);
        advance(3);
        CHECK(number(*session.latestSnapshot(), "game_2027.blue_score") == finalScore);
        CHECK(text(*session.latestSnapshot(), "game_2027.phase") == "Disabled");
        CHECK(std::any_of(console.begin(), console.end(), [&](const auto& line) {
            return line.find("POWER_UP auto " + assignment) != std::string::npos;
        }));
        CHECK(std::any_of(console.begin(), console.end(), [](const auto& line) {
            return line.find("POWER_UP teleop") != std::string::npos;
        }));
        std::printf("POWER_UP MATCH %s data=%s auto=%.3f final=%.3f sim=%.3f bodies=%d chassis_x=%.3f\n",
                    language, assignment.c_str(), autoScore, finalScore,
                    static_cast<double>(session.latestSnapshot()->simTimeUs()) / 1e6,
                    final->physicsStats().bodies, final->chassis().pose.position.x);
```

## 4. Reset without relaunching the robot

Request a normal acknowledged reset and wait for a new epoch at simulation time zero.
After a settling second, scores and phase counters must be zero, the preload restored, reserve
counts restored and the chassis back at its saved pose. No process restart is hiding stale state.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        const auto oldEpoch = session.runner().runEpoch();
        sdk::proto::SimControl reset;
        reset.command = sdk::proto::SimCommand::Reset;
        session.controller().control(reset);
        REQUIRE(pump(30s, [&] {
            const auto snap = session.latestSnapshot();
            return session.runner().runEpoch() > oldEpoch && snap && snap->simTimeUs() == 0;
        }));
        advance(1);
        const auto restored = session.latestSnapshot();
        CHECK(number(*restored, "game_2027.blue_score") == 0);
        CHECK(number(*restored, "game_2027.inventory") == 1);
        CHECK(number(*restored, "game_2027.released") == 0);
        CHECK(number(*restored, "game_2027.auto_seconds") == 0);
        CHECK(number(*restored, "game_2027.teleop_seconds") == 0);
        CHECK(restored->entityGroups()[0].positions.size() == 32);
        CHECK(number(*restored, "game_2027.portal_blue") == 13);
        CHECK(number(*restored, "game_2027.portal_red") == 14);
        CHECK(restored->driverStation().gameSpecificMessage == assignment);
        CHECK(restored->chassis().pose.position.x == doctest::Approx(startX).epsilon(0.01));
        CHECK(restored->physicsStats().bodies == bodyBudget);
```

Enable autonomous again. Both robots must repeat actual delivery with approximately the same
score and body count. Finally stop the session and verify its child process is gone.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
        ds.enabled = true;
        ds.mode = sdk::proto::RobotMode::Autonomous;
        for (int remaining = 15; remaining > 0; --remaining) {
            ds.matchTimeS = remaining;
            session.controller().setDriverStation(ds);
            advance(1);
        }
        const auto repeated = session.latestSnapshot();
        CHECK(number(*repeated, "game_2027.released") == 1);
        CHECK(number(*repeated, "game_2027.inventory") == 0);
        CHECK(number(*repeated, "game_2027.blue_score") == doctest::Approx(autoScore).epsilon(0.01));
        CHECK(repeated->physicsStats().bodies == bodyBudget);
        std::printf("POWER_UP RESET %s epoch=%llu auto=%.3f released=%.0f bodies=%d\n", language,
                    static_cast<unsigned long long>(session.runner().runEpoch()),
                    number(*repeated, "game_2027.blue_score"), number(*repeated, "game_2027.released"),
                    repeated->physicsStats().bodies);
        const auto pid = session.runner().robotHello()->pid;
        session.requestStop();
        session.waitUntilFinished();
        CHECK_FALSE(processAlive(pid));
    }
}
```

The test main is intentionally opt-in. Exit code 77 means skipped, not verified. A missing extension
is reported as a prerequisite failure rather than a passing simulated robot.

<!-- book-file: app\host\tests\e2e_season_test.cpp -->
```cpp
int main(int argc, char** argv) {
    if (env("WPIEMU_E2E").empty()) {
        std::puts("SKIPPED: set WPIEMU_E2E=1; requires the season's JDK and native desktop toolchain");
        return kSkipExitCode;
    }
    const auto extension = core::launcher::locateHalsimExtension(
        season::binaryDir(), &sdk::readEnvironment, season::year(), season::buildExtension());
    if (extension.empty()) {
        std::puts("SKIPPED: build with WPIEMU_SEASONS=all or set WPIEMU_EXTENSION to the season's extension");
        return kSkipExitCode;
    }
    return doctest::Context(argc, argv).run();
}
```

## 5. Run it and read the evidence

The original CTest selector names the old probe. For this scratch experiment, run the executable
directly with the following environment rather than changing app CMake. Start at the export root:

```powershell
cmake --build build\power-up-book --config Release `
  --target wpiemu_season_e2e_tests halsim_wpiemu --parallel
$env:WPIEMU_E2E = "1"
$env:WPIEMU_E2E_JAVA_HOME = "C:\Users\Public\wpilib\2027_alpha7\jdk"
$env:JAVA_HOME = $env:WPIEMU_E2E_JAVA_HOME
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
$env:WPIEMU_E2E_VERBOSE = "1"
$env:WPIEMU_SEASON_YEAR = "2027"
$env:WPIEMU_SEASON_VERSION = "2027.0.0-alpha-7"
$env:WPIEMU_SEASON_SAMPLES = (Resolve-Path samples\season_2027).Path
$env:WPIEMU_SEASON_BINARY_DIR = (Resolve-Path build\power-up-book\bin\Release).Path
$env:WPIEMU_SEASON_EXTENSION = Join-Path $env:WPIEMU_SEASON_BINARY_DIR "halsim_wpiemu.dll"
& (Join-Path $env:WPIEMU_SEASON_BINARY_DIR "wpiemu_season_e2e_tests.exe") `
  '--test-case=e2e: Power UP*'
if ($LASTEXITCODE -ne 0) { throw "Power Up match did not pass (exit $LASTEXITCODE)" }
```

For Ninja, use `build\power-up-book\bin` instead of its `Release` subdirectory, and configure/build
inside the Visual Studio environment as described in [Building](../BUILDING.md). Use your installed
JDK path if different. No global toolchain settings need changing.

Expect a Java and a C++ `POWER_UP MATCH` result plus a `POWER_UP RESET` result for each.
The tested fixture scores about 15.29 in autonomous and 150.29 overall; small startup/tick alignment
differences are not a claim about official integer scorekeeping. The assertions measure physical
delivery, rate differences, frozen disabled scores, repeat delivery and cleanup.

## 6. Look at it, then separate working features from homework

Configure a Qt-enabled build as in [Building](../BUILDING.md), build the app, and open the saved
project:

```powershell
.\build\power-up-book\bin\Release\wpiemu_app.exe --project .\samples\season_2027\cpp
```

Use **Blue** alliance, choose Practice and set its durations to **15 s auto / 2 s gap / 135 s
teleop**. Choose Run, then Enable. You should see two pyramids, two rows of six, off-field supply,
a robot driving to the near switch, one released cube and a rising Blue score. Reset should restore
that scene. With the Java fixture the start is on the other side of the near switch. The renderer
draws cubes axis-aligned even when their physics rotates.

This working slice has no human feeding, exchange/vault, force/boost/levitate, parking/climb awards,
ranking points, penalties or opponent robots. The earlier bonus-feature discussion is a design
roadmap, not implemented source or passing coverage. A full-feature Power Up book still needs those
chapters and their tests. Do not treat the limitations list as a substitute for implementing them.

Next: [the next-kickoff playbook](06-next-kickoff.md), or [replay these listings](appendix-replay.md)
to check that your copy has every line.
