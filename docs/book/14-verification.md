# 14. System verification

## 14.1 Objectives

After this chapter the reader can distinguish component evidence from
robot-process evidence, reproduce a complete match, and evaluate repeatability
and tick cost. Chapter 13 is the prerequisite. The match harness and evaluation
tests address R10 and R13.

## 14.2 Problem statement

Passing a scoring unit test does not establish that HAL outputs arrive.
Seeing a cube on screen does not establish that a score froze during disable.
Different failure modes require different observations and test levels.

## 14.3 Background and definitions

**Definition 14.1 (oracle).** An oracle determines an expected result independently
of the production calculation under test. Examples include conserved cube
count, a rule's exact time boundary and a known Driver Station input sequence.

| Level | Evidence |
|---|---|
| L0 | Compilation, warnings and repository boundary checks |
| L1 | Pure rules with explicit inputs and expected outputs |
| L2 | Real collision, movement, custody and teardown |
| L3 | Configuration pipeline and synchronous simulation session |
| L4 | Robot process, real HALSim extension, IPC and simulation |
| L5 | Desktop rendering, field layout and interaction |

L2 does not require a live robot. L4 does not require Qt. An offscreen Qt test
cannot establish that the Quick 3D scene renders correctly.

## 14.4 Design

A real-time test could wait through a match at wall-clock speed. A paused
session with explicit steps instead has exact simulation-time targets.
The harness requests 200 physics ticks per simulated second and changes the
Driver Station countdown between requests.

The observation still waits on real processes with a wall-clock timeout.
That timeout belongs to the test harness, not to game rules. A failed
handshake prints the robot console rather than manufacturing a successful
snapshot.

**Invariant 14.1 (test causality).** The harness may set match controls and load
configuration, but cannot award points or change cube custody.

## 14.5 Implementation

The fixture separates launch, controls and assertions. Its complete declaration
shows the lifetimes: registries outlive the session; the session stops before
its temporary project is removed.

Listing 14.1. Real-process match fixture. <code>app\host\tests\power_up_match.h</code>.

<!-- book-file: app\host\tests\power_up_match.h -->
```cpp
#pragma once

#include <functional>

#include "season_probe.h"
#include "test_support.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/testing/wait.h"

namespace power_up_match {

namespace s = wpiemu::sdk;
namespace host = wpiemu::app::host;
using namespace host::testing;
using namespace std::chrono_literals;

struct Match {
    Scratch scratch{"wpiemu-power-up-match-"};
    TestRegistries registries;
    std::unique_ptr<host::RobotSession> session;
    std::vector<std::string> console;
    s::proto::DsState ds;
    std::string assignment;

    explicit Match(const std::string& language);
    ~Match();
    bool pump(std::chrono::milliseconds timeout, const std::function<bool()>& predicate);
    void advance(int seconds);
    void period(s::proto::RobotMode mode, int seconds);
    std::shared_ptr<const s::ISimSnapshot> snapshot() const { return session->latestSnapshot(); }
    double number(const char* key) const;
    std::string text(const char* key) const;
    void reset();
};

}  // namespace power_up_match
```

Launch reads the published JSON through the normal configuration pipeline.
The module's message must replace the deliberately wrong host-supplied message.
`SessionRunner`, not the synchronous session alone, performs this overlay.

Listing 14.2. Launch and process lifetime. <code>app\host\tests\power_up_launch.cpp</code>.

<!-- book-file: app\host\tests\power_up_launch.cpp -->
```cpp
#include <algorithm>

#include "power_up_match.h"

namespace power_up_match {

Match::Match(const std::string& language) {
    assignment = language == "java" ? "LRL" : "RLR";
    const auto source = season::samples() / ("power_up_" + language);
    copyProject(source, scratch.root / "robot");
    wpiemu::core::config::FileConfigStore store(wpiemu::core::config::projectConfigPath(source),
                                                "Power UP textbook fixture", registries.json,
                                                registries.codec);
    const auto saved = store.load();
    REQUIRE_MESSAGE(saved.ok(), (saved.ok() ? "" : saved.error().message));
    const auto checked = registries.pipeline.load(registries.codec.toValue(saved.value()));
    REQUIRE_MESSAGE(checked.ok(), (checked.ok() ? "" : checked.error().message));
    REQUIRE(checked.value().valid());
    REQUIRE(checked.value().config.project.language == language);
    host::Options options;
    options.projectDir = scratch.root / "robot";
    options.scratchDir = scratch.root / "scratch";
    options.extensionLibrary = wpiemu::core::launcher::locateHalsimExtension(
        season::binaryDir(), &s::readEnvironment, season::year(), season::buildExtension());
    options.driverStation.dsAttached = options.driverStation.fmsAttached = true;
    options.driverStation.station = s::proto::AllianceStation::Blue1;
    options.driverStation.gameSpecificMessage = "NOT_PLUGIN_DATA";
    options.timeScale = 0.0;
    options.startPaused = true;
    options.handshakeTimeout = 8min;
    options.extraGradleArgs = {"--no-daemon"};
    if (!env("WPIEMU_E2E_JAVA_HOME").empty()) options.javaHome = env("WPIEMU_E2E_JAVA_HOME");
    auto started = host::RobotSession::start(registries.view(), checked.value().config, options);
    REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
    session = std::move(started.value());
    ds = options.driverStation;
    REQUIRE(pump(8min, [&] {
        return session->status().programStarted && snapshot() && snapshot()->paused() &&
               session->runner().robotHello().has_value();
    }));
    REQUIRE_FALSE(session->coreOnly());
    CHECK(session->gameModule()->metadata().id == "game_2027");
    CHECK(snapshot()->driverStation().gameSpecificMessage == assignment);
    CHECK(session->runner().robotHello()->wpilibYear == season::year());
}

Match::~Match() {
    if (!session) return;
    const auto hello = session->runner().robotHello();
    session->requestStop();
    session->waitUntilFinished();
    if (hello) CHECK_FALSE(processAlive(hello->pid));
}

bool Match::pump(std::chrono::milliseconds timeout, const std::function<bool()>& predicate) {
    const bool ready = s::testing::waitUntil(
        [&] {
            drainConsole(*session, console, !env("WPIEMU_E2E_VERBOSE").empty());
            return predicate() || !session->status().active();
        },
        timeout, 10ms);
    if (!ready || !session->status().active()) dumpConsole(console);
    return ready && session->status().active() && predicate();
}

}  // namespace power_up_match
```

Control requests have exact time targets. Reset checks the run epoch before
checking restored inventory, supply, score and game data.

Listing 14.3. Stepping, period controls and reset assertions. <code>app\host\tests\power_up_control.cpp</code>.

<!-- book-file: app\host\tests\power_up_control.cpp -->
```cpp
#include "power_up_match.h"

namespace power_up_match {

void Match::advance(int seconds) {
    const auto target = snapshot()->simTimeUs() + seconds * 1'000'000LL;
    s::proto::SimControl command;
    command.command = s::proto::SimCommand::Step;
    command.stepCount = static_cast<std::uint32_t>(seconds * 200);
    session->controller().control(command);
    REQUIRE(pump(120s, [&] { return snapshot() && snapshot()->simTimeUs() >= target; }));
    REQUIRE(snapshot()->simTimeUs() == target);
}

void Match::period(s::proto::RobotMode mode, int seconds) {
    ds.enabled = true;
    ds.mode = mode;
    for (int remaining = seconds; remaining > 0; --remaining) {
        ds.matchTimeS = remaining;
        session->controller().setDriverStation(ds);
        advance(1);
        if (mode == s::proto::RobotMode::Teleop && remaining == 30)
            CHECK(text("game_2027.phase") == "Endgame");
    }
}

double Match::number(const char* key) const {
    const auto snap = snapshot();
    const auto* value = snap->hudValue(key);
    REQUIRE_MESSAGE(value, key);
    return value->asDouble();
}

std::string Match::text(const char* key) const {
    const auto snap = snapshot();
    const auto* value = snap->hudValue(key);
    REQUIRE_MESSAGE(value, key);
    return value->asString();
}

void Match::reset() {
    const auto epoch = session->runner().runEpoch();
    s::proto::SimControl command;
    command.command = s::proto::SimCommand::Reset;
    session->controller().control(command);
    REQUIRE(pump(30s, [&] {
        return snapshot() && session->runner().runEpoch() > epoch && snapshot()->simTimeUs() == 0;
    }));
    advance(1);
    CHECK(number("game_2027.blue_score") == 0);
    CHECK(number("game_2027.inventory") == 1);
    CHECK(number("game_2027.released") == 0);
    CHECK(number("game_2027.portal_blue") == 13);
    CHECK(number("game_2027.portal_red") == 14);
    CHECK(snapshot()->entityGroups()[0].positions.size() == 32);
    CHECK(snapshot()->driverStation().gameSpecificMessage == assignment);
}

}  // namespace power_up_match
```

The test runs 15 s autonomous, 2 s disabled, 135 s teleoperation and 3 s disabled.
It then resets the same robot process and repeats autonomous. The body budget
must remain constant and every published position must remain finite.

Listing 14.4. Full matches in both languages. <code>app\host\tests\e2e_power_up_test.cpp</code>.

<!-- book-file: app\host\tests\e2e_power_up_test.cpp -->
```cpp
#define DOCTEST_CONFIG_IMPLEMENT
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "power_up_match.h"

using namespace power_up_match;

TEST_CASE("e2e: Power UP physical full match and reset with Java and C++") {
    for (const std::string language : {"java", "cpp"}) {
        CAPTURE(language);
        Match match(language);
        match.advance(1);
        const auto initialX = match.snapshot()->chassis().pose.position.x;
        const auto budget = match.snapshot()->physicsStats().bodies;
        match.period(s::proto::RobotMode::Autonomous, 15);
        CHECK(match.snapshot()->chassis().pose.position.x > initialX + 1.8f);
        CHECK(match.number("game_2027.released") == 1);
        CHECK(match.number("game_2027.inventory") == 0);
        CHECK(match.number("game_2027.auto_seconds") == doctest::Approx(15.0));
        const auto autoScore = match.number("game_2027.blue_score");
        REQUIRE(autoScore > 5.0);
        match.ds.enabled = false;
        match.ds.matchTimeS = 0.0;
        match.session->controller().setDriverStation(match.ds);
        match.advance(2);
        CHECK(match.number("game_2027.blue_score") == autoScore);
        match.period(s::proto::RobotMode::Teleop, 135);
        const auto finalScore = match.number("game_2027.blue_score");
        CHECK(finalScore - autoScore == doctest::Approx(135.0));
        CHECK(match.number("game_2027.teleop_seconds") == doctest::Approx(135.0));
        CHECK(match.number("game_2027.endgame_seconds") == doctest::Approx(30.0));
        CHECK(match.snapshot()->physicsStats().bodies == budget);
        for (const auto& group : match.snapshot()->entityGroups())
            for (const auto& p : group.positions)
                CHECK((std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)));
        match.ds.enabled = false;
        match.ds.matchTimeS = 0.0;
        match.session->controller().setDriverStation(match.ds);
        match.advance(3);
        CHECK(match.number("game_2027.blue_score") == finalScore);
        CHECK(std::any_of(match.console.begin(), match.console.end(), [&](const auto& line) {
            return line.find("POWER_UP auto " + match.assignment) != std::string::npos;
        }));
        std::printf("POWER_UP MATCH %s auto=%.3f final=%.3f sim=%.3f bodies=%d chassis_x=%.3f\n",
                    language.c_str(), autoScore, finalScore,
                    static_cast<double>(match.snapshot()->simTimeUs()) / 1e6,
                    match.snapshot()->physicsStats().bodies,
                    match.snapshot()->chassis().pose.position.x);
        match.reset();
        match.period(s::proto::RobotMode::Autonomous, 15);
        CHECK(match.number("game_2027.released") == 1);
        CHECK(match.number("game_2027.blue_score") == doctest::Approx(autoScore).epsilon(0.01));
        CHECK(match.snapshot()->physicsStats().bodies == budget);
    }
}

int main(int argc, char** argv) {
    if (env("WPIEMU_E2E").empty()) {
        std::puts("SKIPPED: set WPIEMU_E2E=1 and install the pinned toolchain");
        return kSkipExitCode;
    }
    return doctest::Context(argc, argv).run();
}
```

The existing probe remains a separate test. Its platform-only assertion now
depends on whether that season has a registered game. CMake registers the
additional match test by project presence, not a year-specific host branch.

Listing 14.5. Retain the probe for seasons with or without a game. <code>app\host\tests\e2e_season_test.cpp</code>.

<!-- book-edit: app\host\tests\e2e_season_test.cpp -->
```json
{
  "old": "        REQUIRE(snapshot);\n        CHECK(session.coreOnly());\n        REQUIRE(session.runner().robotHello());\n",
  "new": "        REQUIRE(snapshot);\n        CHECK(session.coreOnly() == (registries.plugins.findByYear(season::year()) == nullptr));\n        REQUIRE(session.runner().robotHello());\n"
}
```

Listing 14.6. Add the independent match executable and opt-in test. <code>app\host\CMakeLists.txt</code>.

<!-- book-edit: app\host\CMakeLists.txt -->
```json
{
  "old": "    if(_wpiemu_probe_seasons)\n        add_executable(wpiemu_season_e2e_tests tests/e2e_season_test.cpp tests/e2e_vision_season_test.cpp)\n        add_dependencies(wpiemu_season_e2e_tests halsim_wpiemu)\n        target_link_libraries(wpiemu_season_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)\n        wpiemu_doctest_target(wpiemu_season_e2e_tests)\n        add_executable(wpiemu_season_vendor_e2e_tests tests/e2e_vendor_season_test.cpp)\n        add_dependencies(wpiemu_season_vendor_e2e_tests halsim_wpiemu)\n        target_link_libraries(wpiemu_season_vendor_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)\n        wpiemu_doctest_target(wpiemu_season_vendor_e2e_tests)\n        foreach(_year IN LISTS _wpiemu_probe_seasons)\n            wpiemu_season_value(_probes \"${_year}\" probes)\n            wpiemu_season_value(_version \"${_year}\" wpilib version)\n            # The build tree's own extension only serves the primary season; other seasons come from the composed install.\n            if(_year STREQUAL WPIEMU_WPILIB_SEASON)\n                set(_extension \"$<TARGET_FILE:halsim_wpiemu>\")\n            else()\n                set(_extension \"\")\n            endif()\n            set(_env \"WPIEMU_SEASON_YEAR=${_year}\" \"WPIEMU_SEASON_VERSION=${_version}\"\n                \"WPIEMU_SEASON_SAMPLES=${CMAKE_SOURCE_DIR}/${_probes}\" \"WPIEMU_SEASON_EXTENSION=${_extension}\")\n            add_test(NAME e2e.season_${_year} COMMAND wpiemu_season_e2e_tests \"--test-case=e2e: season probe*\")\n",
  "new": "    if(_wpiemu_probe_seasons)\n        add_executable(wpiemu_power_up_e2e_tests\n            tests/e2e_power_up_test.cpp tests/power_up_launch.cpp tests/power_up_control.cpp)\n        add_dependencies(wpiemu_power_up_e2e_tests halsim_wpiemu)\n        target_link_libraries(wpiemu_power_up_e2e_tests PRIVATE\n            wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)\n        wpiemu_doctest_target(wpiemu_power_up_e2e_tests)\n        add_executable(wpiemu_season_e2e_tests tests/e2e_season_test.cpp tests/e2e_vision_season_test.cpp)\n        add_dependencies(wpiemu_season_e2e_tests halsim_wpiemu)\n        target_link_libraries(wpiemu_season_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)\n        wpiemu_doctest_target(wpiemu_season_e2e_tests)\n        add_executable(wpiemu_season_vendor_e2e_tests tests/e2e_vendor_season_test.cpp)\n        add_dependencies(wpiemu_season_vendor_e2e_tests halsim_wpiemu)\n        target_link_libraries(wpiemu_season_vendor_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)\n        wpiemu_doctest_target(wpiemu_season_vendor_e2e_tests)\n        foreach(_year IN LISTS _wpiemu_probe_seasons)\n            wpiemu_season_value(_probes \"${_year}\" probes)\n            wpiemu_season_value(_version \"${_year}\" wpilib version)\n            # The build tree's own extension only serves the primary season; other seasons come from the composed install.\n            if(_year STREQUAL WPIEMU_WPILIB_SEASON)\n                set(_extension \"$<TARGET_FILE:halsim_wpiemu>\")\n            else()\n                set(_extension \"\")\n            endif()\n            set(_env \"WPIEMU_SEASON_YEAR=${_year}\" \"WPIEMU_SEASON_VERSION=${_version}\"\n                \"WPIEMU_SEASON_SAMPLES=${CMAKE_SOURCE_DIR}/${_probes}\" \"WPIEMU_SEASON_EXTENSION=${_extension}\")\n            if(EXISTS \"${CMAKE_SOURCE_DIR}/${_probes}/power_up_cpp\")\n                add_test(NAME e2e.power_up_${_year} COMMAND wpiemu_power_up_e2e_tests)\n                set_tests_properties(e2e.power_up_${_year} PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1500\n                    ENVIRONMENT \"${_env};WPIEMU_SEASON_BINARY_DIR=$<TARGET_FILE_DIR:wpiemu_power_up_e2e_tests>\")\n            endif()\n            add_test(NAME e2e.season_${_year} COMMAND wpiemu_season_e2e_tests \"--test-case=e2e: season probe*\")\n"
}
```

The boundary checker also records the new executable's owner. This is a target
ownership entry, not an exception to the SDK dependency rule.

Listing 14.7. Declare the new test target's application-host ownership. <code>tools\check_boundaries.py</code>.

<!-- book-edit: tools\check_boundaries.py -->
```json
{
  "old": "    \"wpiemu_season_e2e_tests\": \"app/host\",\n    \"wpiemu_season_vendor_e2e_tests\": \"app/host\",\n",
  "new": "    \"wpiemu_season_e2e_tests\": \"app/host\",\n    \"wpiemu_power_up_e2e_tests\": \"app/host\",\n    \"wpiemu_season_vendor_e2e_tests\": \"app/host\",\n"
}
```

## 14.6 Verification

Repeated input histories compare physics hashes at every tick, first between
two one-worker runs, then with two and four workers. Equal hashes are evidence
for this history and backend, not a proof of cross-platform determinism.

The timing case warms the model for 1,000 ticks, measures 10,000 further ticks
in Release, and prints their mean cost. Clock use here measures a test; game
logic still uses simulation time.

Listing 14.8. Repeatability and measured tick cost. <code>tests\game_2027\test_determinism.cpp</code>.

<!-- book-file: tests\game_2027\test_determinism.cpp -->
```cpp
#include <chrono>
#include <iostream>

#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: repeated fixed input histories have equal physics hashes") {
    auto run = [](int workers) {
        Match match("LRL", false, true, workers);
        match.services.driverStation.enabled = true;
        match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
        match.services.driverStation.matchTimeS = 15.0;
        std::vector<std::uint64_t> hashes;
        for (int i = 0; i < 1000; ++i) {
            match.advance(1);
            hashes.push_back(match.world->stateHash());
        }
        return hashes;
    };
    const auto reference = run(1);
    CHECK(reference == run(1));
    CHECK(reference == run(2));
    CHECK(reference == run(4));
}

TEST_CASE("Power UP: Release tick evaluation records measured cost") {
    Match match("LRL");
    const auto initial = match.snapshot();
    std::cout << "Power UP initial: field=" << initial->entityGroups()[0].positions.size()
              << " held=" << match.hud("game_2027.inventory") << " reserve="
              << match.hud("game_2027.portal_blue") + match.hud("game_2027.portal_red") << "\n";
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Teleop;
    match.services.driverStation.matchTimeS = 135.0;
    match.advance(1000);
    const auto start = std::chrono::steady_clock::now();
    match.advance(10000);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const auto us = std::chrono::duration<double, std::micro>(elapsed).count() / 10000.0;
    std::cout << "Power UP evaluation: " << us << " microseconds/tick\n";
    CHECK(match.world->stats().bodies > 60);
}
```

The final evaluation is recorded in [Appendix C](appendix-c-conformance.md).
Core and extended scoring are reported separately because Chapter 15 changes
the accounting oracle. Earlier prototype runs are not evidence for the
Markdown reproduction.

## 14.7 Reproduction

Extract through Chapter 14, create the robot scaffolding, and build both
<code>wpiemu_tests</code> and <code>wpiemu_power_up_e2e_tests</code>.
Enable the opt-in match test:

```powershell
$env:WPIEMU_E2E = '1'
$env:WPIEMU_E2E_JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk'
ctest --test-dir build\book -R "e2e.power_up_" --no-tests=error --output-on-failure
.\build\book\bin\wpiemu_tests.exe '--source-file=*test_determinism*'
```

Appendix B gives the complete environment and desktop procedure.

## 14.8 Discussion

The fixtures isolate a physical core, but cover one simple robot, nominal
geometry and a finite collection of assignments. Tick cost includes physics
and game updates in the fixture; it excludes robot-process waits, UI drawing
and snapshot serialization. It is not a desktop frame-rate measurement.

## 14.9 Summary

A complete match now exercises the same output path as an ordinary robot
project. Separate tests establish repeatability, reset and scoring freezes.
Each result has a stated observation boundary.

## 14.10 Exercises

1. Explain which test level would detect a reversed right-motor mapping.
2. Identify why a constant body budget does not, by itself, prove correct custody.
3. Extend the match's disabled interval and predict the two clocks that change.

[Chapter 15](15-scoring.md)
