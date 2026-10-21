// End-to-end test of the real product pipeline (opt-in, label `e2e`):
//
//   copy of a real WPILib project -> gradle simulateJava with the real halsim_wpiemu -> IPC -> real
//   SessionRunner (Box3D world, 2026 plugin, swerve drivetrain, mechanisms) -> DS enabled in teleop
//   -> robot drives.
//
// Needs WPILib (JDK + offline Maven), Gradle (the project's wrapper), a robot project and a built
// halsim_wpiemu. Skipped (exit code 77, CTest SKIP_RETURN_CODE) with a clear message when
// WPIEMU_E2E is not set or a prerequisite is missing.
//
// The robot is samples/swerve_robot/java; the CAN layout and expectations below are its own.
//
//   $env:WPIEMU_E2E = "1"; ctest -C Release -L e2e --output-on-failure
//   $env:WPIEMU_E2E_PROJECT = "C:\path\to\robot"    another robot with the same CAN layout
//   $env:WPIEMU_E2E_KEEP = "1"                      keep the temp copy for inspection
//   $env:WPIEMU_E2E_VERBOSE = "1"                   echo the robot console
//   $env:WPIEMU_E2E_LAUNCH = "direct"               use the simulateExternal direct launch
//                                                   instead of the gradle init script
//   $env:WPIEMU_E2E_TANK = "1"                      opt in to the saved six-wheel/two-motor
//                                                   tank fixture case
//     Run with --test-case="e2e: saved tank robot*" and WPIEMU_E2E_PROJECT pointing to that robot.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "host/robot_session.h"
#include "test_support.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"
#include "wpiemu/sdk/testing/wait.h"

namespace fs = std::filesystem;
using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

#ifndef WPIEMU_E2E_EXTENSION
#define WPIEMU_E2E_EXTENSION ""
#endif
#ifndef WPIEMU_E2E_DEFAULT_PROJECT
#define WPIEMU_E2E_DEFAULT_PROJECT ""
#endif

namespace {

double seconds(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double>(b - a).count(); }

struct SwerveModuleIds {
    const char* name;
    float x, y;
    int driveId, steerId, encoderId;
};
// samples/swerve_robot Swerve.java: track 11.125 in x 10.875 in; CAN ids from its module list.
constexpr SwerveModuleIds kModules[] = {
    {"fl", 0.2826f, 0.2762f, 4, 3, 12},
    {"fr", 0.2826f, -0.2762f, 8, 7, 11},
    {"bl", -0.2826f, 0.2762f, 2, 1, 9},
    {"br", -0.2826f, -0.2762f, 6, 5, 10},
};
constexpr int kPigeonId = 13;
constexpr int kIntakeRollerId = 32;
constexpr int kIntakePivotId = 29;
constexpr int kIntakePivotEncoderId = 30;

sdk::RobotConfig baseConfig(const fs::path& project) {
    sdk::RobotConfig c;
    c.name = "Swerve sample e2e";
    c.project.path = project.string();
    c.project.language = "java";
    c.project.wpilibYear = 2026;
    c.project.gradleRioVersion = "2026.2.1";
    return c;
}

// Minimal but complete model: swerve chassis, IMU, one roller mechanism, every mapping to a device that discovery announced.
sdk::RobotConfig mappedConfig(const fs::path& project) {
    sdk::RobotConfig c = baseConfig(project);
    c.chassis.sizeM = {0.775f, 0.80f, 0.15f};
    c.chassis.massKg = 61.235f;
    c.chassis.comHeightM = 0.20f;
    c.drivetrain.spec.kind = "swerve";
    c.drivetrain.spec.tireModel = "brush";
    for (const SwerveModuleIds& m : kModules) {
        sdk::WheelSpec w;
        w.name = m.name;
        w.kind = sdk::WheelKind::Swerve;
        w.steered = true;
        w.positionM = {m.x, m.y, 0.0508f};
        w.radiusM = 0.0508f;
        w.driveGearRatio = 7.03f;
        w.steerGearRatio = 287.0f / 11.0f;
        w.steerMotor = sdk::motors::falcon500();
        c.drivetrain.spec.wheels.push_back(w);
        c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, w.name + ".drive"}, phoenixKey("talonfx", m.driveId)});
        c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, sdk::steerSlotName(w.name)}, phoenixKey("talonfx", m.steerId)});
        c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, sdk::steerEncoderSlotName(w.name)},
                                         phoenixKey("cancoder", m.encoderId)});
    }
    c.robotMappings.push_back({{sdk::slot_owner::kRobot, "imu"}, phoenixKey("pigeon2", kPigeonId)});

    sdk::MechanismConfig roller;
    roller.id = "intake_roller";
    roller.type = "mechanism";
    roller.label = "Intake roller";
    roller.mount.position = {0.45f, 0.f, 0.15f};
    roller.mappings.push_back({{roller.id, "motor"}, phoenixKey("talonfx", kIntakeRollerId)});
    c.mechanisms.push_back(std::move(roller));

    // The intake pivot: teleopInit runs a Motion Magic move to 114 deg through a fused CANcoder, which closes the loop through
    // the emulator (Phoenix firmware sim -> rotor feedback from the pivot plant -> CANcoder position).
    sdk::MechanismConfig pivot;
    pivot.id = "intake_pivot";
    pivot.type = "mechanism";
    pivot.properties["motion"] = "arm";
    pivot.label = "Intake pivot";
    pivot.mount.position = {0.35f, 0.f, 0.20f};
    pivot.properties["gearRatio"] = 40.0;
    pivot.properties["minPosition"] = 0.0;
    pivot.properties["maxPosition"] = 2.3;
    pivot.properties["startPosition"] = 0.087;
    pivot.properties["simulateGravity"] = false;
    pivot.properties["momentOfInertiaKgM2"] = 0.4;
    pivot.properties["statorCurrentLimitA"] = 75.0;
    pivot.properties["neutralMode"] = "brake";
    pivot.mappings.push_back({{pivot.id, "motor"}, phoenixKey("talonfx", kIntakePivotId), true});
    pivot.mappings.push_back({{pivot.id, "encoder"}, phoenixKey("cancoder", kIntakePivotEncoderId)});
    c.mechanisms.push_back(std::move(pivot));
    return c;
}

int countType(const sdk::ISimSnapshot& snap, const std::string& type) {
    int n = 0;
    for (const sdk::DiscoveredDevice& d : snap.devices())
        if (d.descriptor.type == type && d.live) ++n;
    return n;
}

bool hasDevice(const sdk::ISimSnapshot& snap, const char* type, int id) {
    for (const sdk::DiscoveredDevice& d : snap.devices())
        if (d.descriptor.type == type && d.descriptor.id == id) return true;
    return false;
}

// Pumps status() / logs until `done`, a terminal state or the timeout. Robot console lines are echoed when WPIEMU_E2E_VERBOSE is set.
template <class Done>
bool waitFor(app::host::RobotSession& s, std::chrono::seconds timeout, Done&& done, sdk::SessionStatus& last, std::vector<std::string>& console) {
    const bool verbose = !env("WPIEMU_E2E_VERBOSE").empty();
    bool reached = false;
    sdk::testing::waitUntil(
        [&] {
            for (const sdk::SessionLogEntry& e : s.takeLogs(5000)) {
                const std::string line = (e.stream == sdk::LogStream::Emulator ? "[emu " + e.source + "] " : "[robot] ") + e.text;
                console.push_back(line);
                if (verbose) std::printf("%s\n", line.c_str());
            }
            last = s.status();
            reached = done(last);
            return reached || !last.active();   // crashed / failed / exited
        },
        timeout, 50ms);
    return reached;
}

void dumpConsole(const std::vector<std::string>& console) {
    const std::size_t from = console.size() > 80 ? console.size() - 80 : 0;
    for (std::size_t i = from; i < console.size(); ++i) std::printf("    | %s\n", console[i].c_str());
}

}  // namespace

TEST_CASE("e2e: swerve sample robot - discovery, mapping and a driven teleop run") {
    const Clock::time_point t0 = Clock::now();
    const fs::path original = !env("WPIEMU_E2E_PROJECT").empty() ? fs::path(env("WPIEMU_E2E_PROJECT")) : fs::path(WPIEMU_E2E_DEFAULT_PROJECT);
    const fs::path extension = WPIEMU_E2E_EXTENSION;
    const bool direct = env("WPIEMU_E2E_LAUNCH") == "direct";

    Scratch scratch("wpiemu-e2e-");
    const fs::path project = scratch.root / "project";
    copyProject(original, project);
    std::printf("e2e: project copy %s (launch mode: %s)\n", project.string().c_str(), direct ? "simulateExternal direct" : "gradle init script");

    TestRegistries registries;

    app::host::Options base;
    base.projectDir = project;
    base.extensionLibrary = extension;
    base.scratchDir = scratch.root / "scratch";
    base.launchMode = direct ? core::launcher::LaunchMode::ExternalDirect : core::launcher::LaunchMode::GradleInitScript;
    base.handshakeTimeout = 8min;
    base.extraEnv[sdk::env_names::kLogLevel] = "info";

    std::vector<std::string> console;
    sdk::SessionStatus st;

    // ---------------------------------------------------------------- phase 1: discovery
    std::printf("e2e: phase 1 - Detect devices\n");
    std::int64_t discoveryRobotPid = 0;
    {
        app::host::Options o = base;
        o.mode = sdk::SessionMode::Armed;
        const Clock::time_point t = Clock::now();
        auto started = app::host::RobotSession::start(registries.view(), baseConfig(project), o);
        REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
        app::host::RobotSession& session = *started.value();
        CHECK_FALSE(session.coreOnly());

        double tConnected = -1, tProgram = -1;
        const bool ok = waitFor(
            session, 480s,
            [&](const sdk::SessionStatus& s) {
                if (tConnected < 0 && s.link == sdk::LinkState::Connected) tConnected = seconds(t, Clock::now());
                if (tProgram < 0 && s.programStarted) tProgram = seconds(t, Clock::now());
                return s.programStarted;
            },
            st, console);
        if (!ok) {
            std::printf("e2e: discovery failed: %s\n", st.detail.c_str());
            dumpConsole(console);
        }
        REQUIRE_MESSAGE(ok, st.detail);
        // Let the announce messages that follow the lifecycle signal arrive.
        waitFor(session, 2s, [](const sdk::SessionStatus&) { return false; }, st, console);
        const auto snap = session.latestSnapshot();
        REQUIRE(snap != nullptr);
        std::printf("e2e:   connected after %.1f s, program started after %.1f s, %zu devices announced\n", tConnected, tProgram, snap->devices().size());
        for (const sdk::DiscoveredDevice& d : snap->devices())
            std::printf("e2e:     %-9s %-9s id=%-3d %s\n", d.descriptor.family.c_str(), d.descriptor.type.c_str(), d.descriptor.id, d.live ? "" : "(retired)");
        CHECK(countType(*snap, "talonfx") >= 10);   // 8 swerve motors, the roller and the pivot
        CHECK(countType(*snap, "cancoder") >= 5);   // 4 azimuth encoders and the pivot's
        CHECK(countType(*snap, "pigeon2") >= 1);
        for (const SwerveModuleIds& m : kModules) {
            CHECK(hasDevice(*snap, "talonfx", m.driveId));
            CHECK(hasDevice(*snap, "talonfx", m.steerId));
        }
        CHECK(hasDevice(*snap, "pigeon2", kPigeonId));
        CHECK(hasDevice(*snap, "talonfx", kIntakeRollerId));
        if (const auto hello = session.runner().robotHello()) discoveryRobotPid = hello->pid;
        CHECK(discoveryRobotPid > 0);

        // The user presses Stop: the whole tree must go.
        const Clock::time_point tStop = Clock::now();
        session.requestStop();
        session.waitUntilFinished();
        std::printf("e2e:   stopped in %.2f s\n", seconds(tStop, Clock::now()));
        CHECK(session.status().process == sdk::ProcessState::Exited);
        sdk::testing::waitUntil([&] { return !processAlive(discoveryRobotPid); }, 10s, 100ms);
        CHECK_FALSE(processAlive(discoveryRobotPid));
    }

    // ---------------------------------------------------------------- phase 2: mapped run in teleop
    std::printf("e2e: phase 2 - run with a generated RobotConfig\n");
    {
        app::host::Options o = base;
        o.mode = sdk::SessionMode::Run;
        o.timeScale = 1.0;
        sdk::proto::DsState& ds = o.driverStation;
        ds.enabled = true;
        ds.mode = sdk::proto::RobotMode::Teleop;
        ds.station = sdk::proto::AllianceStation::Blue1;
        ds.joysticks[0].present = true;
        ds.joysticks[0].axes = {0.f, -0.35f, 0.f, 0.f, 0.f, 0.f};
        ds.joysticks[0].povs = {-1};
        ds.joysticks[0].buttonCount = 12;

        const Clock::time_point t = Clock::now();
        auto started = app::host::RobotSession::start(registries.view(), mappedConfig(project), o);
        REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
        app::host::RobotSession& session = *started.value();
        for (const sdk::Issue& i : session.configIssues()) std::printf("e2e:   config issue [%s] %s\n", i.code.c_str(), i.message.c_str());

        bool ok = waitFor(session, 480s, [](const sdk::SessionStatus& s) { return s.programStarted; }, st, console);
        if (!ok) {
            std::printf("e2e: run failed to start: %s\n", st.detail.c_str());
            dumpConsole(console);
        }
        REQUIRE_MESSAGE(ok, st.detail);
        const double tStart = seconds(t, Clock::now());
        const Clock::time_point runStart = Clock::now();

        auto simSeconds = [&] {
            const auto snap = session.latestSnapshot();
            return snap ? static_cast<double>(snap->simTimeUs()) / 1e6 : 0.0;
        };
        constexpr double kRealtimeSimS = 5.0;
        constexpr double kContinuedSimS = 6.0;
        bool stalled = false;
        std::vector<std::pair<double, double>> pace;   // (wall s, sim s) once per wall second: shows start-up hiccups
        ok = waitFor(
            session, 90s,
            [&](const sdk::SessionStatus& s) {
                stalled |= s.link == sdk::LinkState::Stalled;
                const double wall = seconds(runStart, Clock::now());
                if (pace.empty() || wall - pace.back().first >= 1.0) pace.emplace_back(wall, simSeconds());
                return simSeconds() >= kRealtimeSimS;
            },
            st, console);
        const double realtimeWall = seconds(runStart, Clock::now());
        for (const auto& [wall, sim] : pace) std::printf("e2e:     pace: wall %.2f s -> sim %.2f s\n", wall, sim);
        if (!ok) dumpConsole(console);
        REQUIRE_MESSAGE(ok, st.detail);

        // Phoenix closed loops run on wall time: keep the entire driven run paced at 1x.
        const Clock::time_point continuedStart = Clock::now();
        const double simAtContinue = simSeconds();
        ok = waitFor(
            session, 120s,
            [&](const sdk::SessionStatus& s) {
                stalled |= s.link == sdk::LinkState::Stalled;
                return simSeconds() >= kRealtimeSimS + kContinuedSimS;
            },
            st, console);
        const double continuedWall = seconds(continuedStart, Clock::now());
        if (!ok) dumpConsole(console);
        REQUIRE_MESSAGE(ok, st.detail);
        const double simNow = simSeconds();

        // pause so the numbers below belong to one instant
        sdk::proto::SimControl pause;
        pause.command = sdk::proto::SimCommand::Pause;
        session.controller().control(pause);
        waitFor(session, 1s, [](const sdk::SessionStatus&) { return false; }, st, console);

        const auto snap = session.latestSnapshot();
        REQUIRE(snap != nullptr);
        const double simTotal = static_cast<double>(snap->simTimeUs()) / 1e6;
        const std::uint64_t loops = session.runner().robotLoopCount();
        const std::uint64_t ticks = session.runner().ticksSent();
        const double expectedLoops = simTotal / 0.020;
        const sdk::Vec3 p0 = snap->chassis().pose.position;
        std::printf(
            "e2e:   program started after %.1f s; first part: %.1f sim s in %.2f wall s; "
            "continued: %.1f sim s in %.2f wall s (%.1fx)\n",
            tStart, kRealtimeSimS, realtimeWall, simNow - simAtContinue, continuedWall,
            (simNow - simAtContinue) / std::max(continuedWall, 1e-6));
        std::printf("e2e:   ticks sent %llu, sim time %.3f s, robot loops %llu (expected ~%.0f), dropped frames %llu\n", static_cast<unsigned long long>(ticks),
                    simTotal, static_cast<unsigned long long>(loops), expectedLoops, static_cast<unsigned long long>(session.runner().droppedFrames()));
        std::printf("e2e:   chassis pose (%.2f, %.2f, %.2f) v=(%.2f, %.2f), battery %.2f V\n", p0.x, p0.y, p0.z, snap->chassis().linearVelocity.x,
                    snap->chassis().linearVelocity.y, snap->batteryVoltageV());
        for (const sdk::WheelSample& w : snap->wheels())
            std::printf("e2e:     wheel %s spin=%.2f rad steer=%.2f rad load=%.0f N\n", w.name.c_str(), w.spinAngleRad, w.steerAngleRad, w.normalLoadN);
        for (const sdk::MechanismState& m : snap->mechanisms()) {
            std::printf("e2e:     mechanism %s active=%d", m.id.c_str(), m.active ? 1 : 0);
            for (const sdk::NamedValue& v : m.values) std::printf(" %s=%.3f", v.name.c_str(), v.value);
            std::printf("\n");
        }
        int mapped = 0;
        for (const sdk::DiscoveredDevice& d : snap->devices()) mapped += d.mapped ? 1 : 0;
        std::printf("e2e:   %zu devices, %d mapped, %zu warnings\n", snap->devices().size(), mapped, snap->warnings().size());
        for (const sdk::HubWarning& w : snap->warnings()) std::printf("e2e:     warning [%s] %s\n", w.code.c_str(), w.message.c_str());

        CHECK_FALSE(stalled);
        CHECK(session.runner().state() == core::runtime::RunnerState::Paused);
        CHECK(session.runner().droppedFrames() == 0);
        CHECK(ticks + 2 >= static_cast<std::uint64_t>(simTotal / 0.005));
        CHECK(std::abs(static_cast<double>(loops) - expectedLoops) <= std::max(3.0, expectedLoops * 0.03));
        CHECK(realtimeWall >= kRealtimeSimS * 0.9);
        CHECK(realtimeWall <= kRealtimeSimS * 1.5);
        CHECK(snap->robotConnected());
        CHECK(snap->driverStation().enabled);
        CHECK(mapped ==
              16);  // 8 swerve motors + 4 azimuth encoders + imu + roller + pivot motor/encoder
        CHECK(snap->wheels().size() == 4);
        double spin = 0;
        for (const sdk::WheelSample& w : snap->wheels()) spin += std::abs(w.spinAngleRad);
        CHECK(spin > 1.0);   // the robot's drive commands reached the wheels
        CHECK(p0.x > 2.3);   // forward stick drives forward, not merely "somewhere"
        CHECK(std::abs(p0.y - 4.0345) < 0.15);
        CHECK(std::abs(snap->chassis().pose.rotation.yaw()) < 2.0 * sdk::kPi / 180.0);
        for (const auto& wheel : snap->wheels())
            CHECK(std::abs(wheel.steerAngleRad) < 2.0 * sdk::kPi / 180.0);
        CHECK(p0.z > 0.02);
        CHECK(p0.z < 0.6);
        CHECK(snap->batteryVoltageV() < 12.6);   // motor current sagged the battery
        bool rollerSeen = false;
        double pivotPosition = -1;
        for (const sdk::MechanismState& m : snap->mechanisms()) {
            rollerSeen |= m.id == "intake_roller";
            if (m.id == "intake_pivot")
                for (const sdk::NamedValue& v : m.values)
                    if (v.name == sdk::value_name::kPosition) pivotPosition = v.value;
        }
        CHECK(rollerSeen);
        CHECK(std::abs(pivotPosition - 114.0 * sdk::kPi / 180.0) < sdk::kPi / 180.0);
        for (const sdk::HubWarning& w : snap->warnings()) {
            CHECK(w.code != "slot_missing_device");
            CHECK(w.code != "slot_incompatible");
        }

        const Clock::time_point tStop = Clock::now();
        session.requestStop();
        session.waitUntilFinished();
        std::printf("e2e:   stopped in %.2f s\n", seconds(tStop, Clock::now()));
        std::int64_t pid = 0;
        if (const auto hello = session.runner().robotHello()) pid = hello->pid;
        sdk::testing::waitUntil([&] { return !processAlive(pid); }, 10s, 100ms);
        CHECK_FALSE(processAlive(pid));
    }
    std::printf("e2e: total %.1f s\n", seconds(t0, Clock::now()));
}

TEST_CASE("e2e: saved tank robot keeps its two mappings and drives six linked wheels" *
          doctest::skip(env("WPIEMU_E2E_TANK").empty())) {
    Scratch scratch("wpiemu-tank-e2e-");
    const fs::path project = scratch.root / "project";
    copyProject(fs::path(env("WPIEMU_E2E_PROJECT")), project);
    TestRegistries registries;
    core::config::FileConfigStore store(project / ".wpiemu" / "robot.json", "tank fixture",
                                        registries.json, registries.codec);
    auto original = store.load();
    REQUIRE_MESSAGE(original.ok(), (original.ok() ? "" : original.error().message));
    REQUIRE(original.value().drivetrain.spec.kind == sdk::drivetrain_kind::kTank);
    const auto mappings = original.value().drivetrain.mappings;
    auto loaded = registries.pipeline.process(original.value());
    REQUIRE(loaded.ok());
    for (const auto& issue : loaded.value().issues) INFO(issue.code << ": " << issue.message);
    REQUIRE(loaded.value().valid());
    auto config = loaded.value().config;
    config.project.path = project.string();
    REQUIRE(config.drivetrain.mappings == mappings);
    REQUIRE(config.drivetrain.mappings.size() == 2);
    REQUIRE(config.drivetrain.spec.wheels.size() == 6);
    REQUIRE(config.motionLinks.size() == 4);
    REQUIRE(store.save(config).ok());
    auto persisted = store.load();
    REQUIRE(persisted.ok());
    auto again = registries.pipeline.process(persisted.value());
    REQUIRE(again.ok());
    CHECK(again.value().config.motionLinks.size() == 4);

    app::host::Options options;
    options.projectDir = project;
    options.extensionLibrary = WPIEMU_E2E_EXTENSION;
    options.scratchDir = scratch.root / "scratch";
    options.handshakeTimeout = 8min;
    options.driverStation.enabled = true;
    options.driverStation.mode = sdk::proto::RobotMode::Teleop;
    options.driverStation.joysticks[0].present = true;
    options.driverStation.joysticks[0].axes = {0.f, 0.f, 0.f, 0.f, 0.f, -0.8f};
    options.driverStation.joysticks[0].buttonCount = 10;
    options.driverStation.joysticks[0].povs = {-1};
    sdk::SessionStatus status;
    std::vector<std::string> console;
    std::int64_t firstPid = 0;
    for (int run = 0; run < 2; ++run) {
        if (run == 1) options.driverStation.enabled = false;
        auto started = app::host::RobotSession::start(registries.view(), config, options);
        REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
        auto& session = *started.value();
        const bool ready = waitFor(
            session, 480s,
            [&](const sdk::SessionStatus& s) {
                const auto snapshot = session.latestSnapshot();
                return s.programStarted && snapshot &&
                       snapshot->simTimeUs() >= (run == 0 ? 2000000 : 20000);
            },
            status, console);
        if (!ready) dumpConsole(console);
        REQUIRE_MESSAGE(ready, status.detail);
        const auto snapshot = session.latestSnapshot();
        REQUIRE(snapshot != nullptr);
        REQUIRE(session.runner().robotHello().has_value());
        const auto pid = session.runner().robotHello()->pid;
        if (run == 0) {
            firstPid = pid;
            for (const auto& link : config.motionLinks) {
                const auto wheel = [&](const std::string& axis) {
                    return std::find_if(snapshot->wheels().begin(), snapshot->wheels().end(),
                                        [&](const auto& w) { return w.name + ".drive" == axis; });
                };
                const auto source = wheel(link.source.slot);
                const auto target = wheel(link.target.slot);
                REQUIRE(source != snapshot->wheels().end());
                REQUIRE(target != snapshot->wheels().end());
                CHECK(std::abs(source->spinAngleRad) > 0.01);
                CHECK(target->spinAngleRad == doctest::Approx(source->spinAngleRad));
            }
        } else {
            CHECK(pid != firstPid);
            CHECK(snapshot->simTimeUs() < 1000000);
            CHECK_FALSE(snapshot->driverStation().enabled);
            for (const auto& wheel : snapshot->wheels()) CHECK(std::abs(wheel.spinAngleRad) < 0.01);
        }
        session.requestStop();
        session.waitUntilFinished();
        CHECK_FALSE(processAlive(pid));
    }
}

int main(int argc, char** argv) {
    auto skip = [](const std::string& why) {
        std::printf("SKIPPED e2e: %s\n", why.c_str());
        return kSkipExitCode;
    };
    if (env("WPIEMU_E2E").empty()) return skip("set WPIEMU_E2E=1 to run the end-to-end test (needs WPILib 2026, Gradle and a robot project)");
    const fs::path project = !env("WPIEMU_E2E_PROJECT").empty() ? fs::path(env("WPIEMU_E2E_PROJECT")) : fs::path(WPIEMU_E2E_DEFAULT_PROJECT);
    std::error_code ec;
    if (project.empty() || !fs::is_directory(project, ec)) return skip("robot project not found: '" + project.string() + "' (set WPIEMU_E2E_PROJECT)");
    const bool windows = fs::path::preferred_separator == '\\';
    if (!fs::exists(project / (windows ? "gradlew.bat" : "gradlew"), ec)) return skip("the project has no Gradle wrapper: " + project.string());
    if (!fs::exists(fs::path(WPIEMU_E2E_EXTENSION), ec)) return skip(std::string("the HALSim extension is not built: ") + WPIEMU_E2E_EXTENSION);
    if (!core::launcher::findWpilibInstall(2026, {}) && env("JAVA_HOME").empty())
        return skip("no WPILib 2026 installation (C:\\Users\\Public\\wpilib\\2026) and no JAVA_HOME");
    doctest::Context context(argc, argv);
    return context.run();
}
