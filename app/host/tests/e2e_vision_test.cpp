// End-to-end test of the vision feature (opt-in, label `e2e`):
//
//   copy of samples/vision_robot/<java|cpp> (an ordinary WPILib project that reads a Limelight and a PhotonVision
//   camera) -> gradle simulate with the real halsim_wpiemu -> IPC -> real SessionRunner with two simulated cameras
//   -> the robot's own `VISION ...` console lines are compared with the emulator's ground truth.
//
//   $env:WPIEMU_E2E = "1"; $env:WPIEMU_E2E_VISION_PROJECT = "<repo>\samples\vision_robot\java"
//   ctest -C Release -R e2e.vision -V
//
// Skipped (exit code 77, CTest SKIP_RETURN_CODE) with a clear message when WPIEMU_E2E is not set or a prerequisite is
// missing. WPIEMU_E2E_KEEP / WPIEMU_E2E_VERBOSE behave as in e2e_test.cpp; WPIEMU_E2E_VISION_LANG = java (default) | cpp.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "host/robot_session.h"
#include "test_support.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/sdk/core/units.h"

namespace fs = std::filesystem;
using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

#ifndef WPIEMU_E2E_EXTENSION
#define WPIEMU_E2E_EXTENSION ""
#endif
#ifndef WPIEMU_E2E_VISION_DEFAULT_PROJECT
#define WPIEMU_E2E_VISION_DEFAULT_PROJECT ""
#endif

namespace {

constexpr double kLatencyMs = 40.0;

// Same constants as the sample robot (samples/vision_robot/<lang>/.../Vision, Robot).
constexpr float kFrontX = 0.30f;
constexpr float kRearX = -0.30f;
constexpr float kCameraHeightM = 0.50f;   // WPILib robot frame: above the floor
constexpr float kComHeightM = 0.20f;

sdk::CameraConfig makeCamera(const char* id, sdk::CameraKind kind, const char* name, float robotFrameX, float yawRad) {
    sdk::CameraConfig c;
    c.id = id;
    c.label = id;
    c.kind = kind;
    c.name = name;
    // The chassis frame's origin is the centre of mass; the robot frame's is on the floor below the hull centre.
    c.local = sdk::Pose3({robotFrameX, 0.f, kCameraHeightM - kComHeightM}, sdk::Quat::fromYaw(yawRad));
    c.hFovDeg = 75.0;
    c.vFovDeg = 50.0;
    c.maxRangeM = 8.0;
    c.fps = 30.0;
    c.latencyMs = kLatencyMs;
    return c;
}

sdk::RobotConfig visionConfig(const fs::path& project, const std::string& language) {
    sdk::RobotConfig c;
    c.name = "Vision e2e";
    c.project.path = project.string();
    c.project.language = language;
    c.project.wpilibYear = 2026;
    c.project.gradleRioVersion = "2026.2.1";
    c.chassis.sizeM = {0.80f, 0.80f, 0.15f};
    c.chassis.massKg = 50.f;
    c.chassis.comHeightM = kComHeightM;
    c.drivetrain.spec.kind = "tank";
    c.drivetrain.spec.tireModel = "brush";
    for (const char* side : {"left", "right"}) {
        sdk::WheelSpec w;
        w.name = side;
        w.kind = sdk::WheelKind::Standard;
        w.positionM = {0.f, std::string(side) == "left" ? 0.30f : -0.30f, 0.0762f};
        w.radiusM = 0.0762f;
        w.driveGearRatio = 8.0f;
        c.drivetrain.spec.wheels.push_back(w);
        c.drivetrain.mappings.push_back(
            {{sdk::slot_owner::kDrivetrain, w.name + ".drive"}, phoenixKey("talonfx", std::string(side) == "left" ? 1 : 2)});
    }
    c.robotMappings.push_back({{sdk::slot_owner::kRobot, "imu"}, phoenixKey("pigeon2", 10)});
    c.cameras.push_back(makeCamera("front", sdk::CameraKind::Limelight, "front", kFrontX, 0.f));
    c.cameras.push_back(makeCamera("rear", sdk::CameraKind::PhotonVision, "photon-rear", kRearX, static_cast<float>(sdk::kPi)));
    return c;
}

// "VISION limelight tv=1 tags=2 ..." -> {"camera": "limelight", "tv": "1", ...}
std::optional<std::map<std::string, std::string>> parseVision(const std::string& text) {
    const std::size_t at = text.find("VISION ");
    if (at == std::string::npos) return std::nullopt;
    std::istringstream in(text.substr(at + 7));
    std::map<std::string, std::string> out;
    std::string token;
    if (!(in >> token)) return std::nullopt;
    out["camera"] = token;
    while (in >> token) {
        const std::size_t eq = token.find('=');
        if (eq != std::string::npos) out[token.substr(0, eq)] = token.substr(eq + 1);
    }
    return out;
}

double num(const std::map<std::string, std::string>& m, const char* key, double fallback = 0.0) {
    const auto it = m.find(key);
    return it == m.end() ? fallback : std::atof(it->second.c_str());
}
bool flag(const std::map<std::string, std::string>& m, const char* key) {
    const auto it = m.find(key);
    return it != m.end() && (it->second == "true" || it->second == "1");
}

double wrapDeg(double d) {
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

using Fields = std::map<std::string, std::string>;

struct Collected {
    std::vector<std::string> console;
    std::vector<Fields> limelight;
    std::vector<Fields> photon;
};

void drain(app::host::RobotSession& s, Collected& out, bool verbose) {
    for (const app::host::LogEntry& e : s.takeLogs(5000)) {
        const std::string line = (e.stream == app::host::LogStream::Emulator ? "[emu " + e.source + "] " : "[robot] ") + e.text;
        out.console.push_back(line);
        if (verbose) std::printf("%s\n", line.c_str());
        if (const auto f = parseVision(e.text)) {
            if ((*f).at("camera") == "limelight") out.limelight.push_back(*f);
            else if ((*f).at("camera") == "photon") out.photon.push_back(*f);
        }
    }
}

void dumpConsole(const std::vector<std::string>& console) {
    const std::size_t from = console.size() > 60 ? console.size() - 60 : 0;
    for (std::size_t i = from; i < console.size(); ++i) std::printf("    | %s\n", console[i].c_str());
}

}  // namespace

TEST_CASE("e2e: vision sample - Limelight and PhotonVision cameras see tags, with the configured latency") {
    const Clock::time_point t0 = Clock::now();
    const fs::path original = !env("WPIEMU_E2E_VISION_PROJECT").empty() ? fs::path(env("WPIEMU_E2E_VISION_PROJECT"))
                                                                         : fs::path(WPIEMU_E2E_VISION_DEFAULT_PROJECT);
    const std::string language = env("WPIEMU_E2E_VISION_LANG").empty() ? "java" : env("WPIEMU_E2E_VISION_LANG");
    const bool verbose = !env("WPIEMU_E2E_VERBOSE").empty();

    Scratch scratch("wpiemu-vision-e2e-");
    const fs::path project = scratch.root / "project";
    copyProject(original, project);
    std::printf("e2e: project copy %s (%s)\n", project.string().c_str(), language.c_str());

    TestRegistries registries;
    app::host::Options o;
    o.projectDir = project;
    o.extensionLibrary = WPIEMU_E2E_EXTENSION;
    o.scratchDir = scratch.root / "scratch";
    o.launchMode = core::launcher::LaunchMode::GradleInitScript;
    o.handshakeTimeout = 10min;
    o.mode = app::host::Mode::Run;
    o.timeScale = 1.0;
    o.extraEnv["WPIEMU_LOG_LEVEL"] = "info";
    sdk::proto::DsState& ds = o.driverStation;
    ds.enabled = true;
    ds.mode = sdk::proto::RobotMode::Autonomous;
    ds.station = sdk::proto::AllianceStation::Blue1;

    const sdk::RobotConfig config = visionConfig(project, language);
    auto started = app::host::RobotSession::start(registries.view(), config, o);
    REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
    app::host::RobotSession& session = *started.value();
    for (const sdk::Issue& i : session.configIssues()) std::printf("e2e:   config issue [%s] %s\n", i.code.c_str(), i.message.c_str());

    Collected got;
    auto simSeconds = [&] {
        const auto snap = session.latestSnapshot();
        return snap ? static_cast<double>(snap->simTimeUs()) / 1e6 : 0.0;
    };
    // Pumps logs/status until `done`, a terminal state or the timeout; samples the camera states as they change.
    std::set<int> everVisible[2];
    auto pump = [&](std::chrono::seconds timeout, auto&& done) {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            drain(session, got, verbose);
            if (const auto snap = session.latestSnapshot()) {
                const auto cams = snap->cameras();
                for (std::size_t i = 0; i < cams.size(); ++i) {
                    const int slot = cams[i].id == "front" ? 0 : 1;
                    for (int id : cams[i].visibleTagIds) everVisible[slot].insert(id);
                }
            }
            const app::host::SessionStatus st = session.status();
            if (done(st)) return true;
            if (!st.active()) return false;
            std::this_thread::sleep_for(20ms);
        }
        return false;
    };

    bool ok = pump(600s, [](const app::host::SessionStatus& s) { return s.programStarted; });
    if (!ok) {
        std::printf("e2e: robot did not start: %s\n", session.status().detail.c_str());
        dumpConsole(got.console);
    }
    REQUIRE_MESSAGE(ok, session.status().detail);
    std::printf("e2e: robot program started after %.1f s\n", std::chrono::duration<double>(Clock::now() - t0).count());

    // Run the autonomous drive as fast as possible.
    sdk::proto::SimControl fast;
    fast.command = sdk::proto::SimCommand::SetTimeScale;
    fast.timeScale = 0.0;
    session.controller().control(fast);
    ok = pump(240s, [&](const app::host::SessionStatus&) { return simSeconds() >= 6.0; });
    if (!ok) dumpConsole(got.console);
    REQUIRE(ok);

    // Stop the robot (disabled) and let the pose estimators settle while the chassis coasts to rest.
    ds.enabled = false;
    session.controller().setDriverStation(ds);
    const double stopAt = simSeconds();
    ok = pump(240s, [&](const app::host::SessionStatus&) { return simSeconds() >= stopAt + 3.0; });
    if (!ok) dumpConsole(got.console);
    REQUIRE(ok);
    sdk::proto::SimControl pause;
    pause.command = sdk::proto::SimCommand::Pause;
    session.controller().control(pause);
    pump(2s, [](const app::host::SessionStatus&) { return false; });
    drain(session, got, verbose);

    const auto snap = session.latestSnapshot();
    REQUIRE(snap != nullptr);
    const sdk::Vec3 truth = snap->chassis().pose.position;
    const double truthYawDeg = sdk::radToDeg(snap->chassis().pose.rotation.yaw());
    std::printf("e2e: sim %.2f s, chassis (%.3f, %.3f, z %.3f) yaw %.1f deg; %zu limelight / %zu photon lines\n", simSeconds(), truth.x, truth.y,
                truth.z, truthYawDeg, got.limelight.size(), got.photon.size());
    for (const sdk::CameraState& c : snap->cameras()) {
        std::printf("e2e:   camera %s visible=%d tags:", c.id.c_str(), c.tagsVisible ? 1 : 0);
        for (int id : c.visibleTagIds) std::printf(" %d", id);
        std::printf("\n");
    }
    std::printf("e2e:   ever visible: front %zu tag(s), rear %zu tag(s)\n", everVisible[0].size(), everVisible[1].size());

    // ---- the robot ran its vision code and both cameras are connected -----------------------------------------
    REQUIRE(got.limelight.size() >= 8);
    REQUIRE(got.photon.size() >= 8);
    CHECK(flag(got.limelight.back(), "connected"));
    CHECK(flag(got.photon.back(), "connected"));
    CHECK(snap->cameras().size() == 2);

    // The Java sample drives forward in autonomous (the Phoenix 6 C++ drive path is unverified, docs/reference/bridges.md: only vision is checked there).
    if (language != "cpp") CHECK(truth.x > 2.0 + 0.5);
    // ---- tags were seen by the robot code wherever the emulator says a camera saw them ------------------------
    const bool llSaw = std::any_of(got.limelight.begin(), got.limelight.end(), [](const Fields& f) { return flag(f, "tv") && num(f, "tags") >= 1; });
    const bool phSaw = std::any_of(got.photon.begin(), got.photon.end(), [](const Fields& f) { return flag(f, "hasTargets") && num(f, "tags") >= 1; });
    CHECK(llSaw == !everVisible[0].empty());
    CHECK(phSaw == !everVisible[1].empty());
    CHECK((llSaw || phSaw));

    // ---- latency: Limelight reports tl+cl, PhotonLib derives the result age from capture/publish timestamps -------
    for (const Fields& f : got.limelight)
        if (flag(f, "tv")) CHECK(num(f, "latencyMs") == doctest::Approx(kLatencyMs).epsilon(0.05));
    int agesChecked = 0;
    // The age of the NEWEST result: the latency plus up to one frame spacing (30 fps) and one robot period. The C++ PhotonLib
    // was observed to report results about one more NT cycle later than the Java one, hence the wider C++ allowance.
    const double photonAgeSlackMs = language == "cpp" ? 85.0 : 1000.0 / 30.0 + 22.0;
    for (const Fields& f : got.photon) {
        if (!flag(f, "hasTargets")) continue;
        const double age = num(f, "resultAgeMs");
        // The age of the NEWEST result: the latency plus up to one frame spacing (30 fps) and one 20 ms robot period.
        CHECK(age >= kLatencyMs - 1.0);
        CHECK(age <= kLatencyMs + photonAgeSlackMs);
        ++agesChecked;
    }

    // ---- pose accuracy while the robot is at rest: the vision estimates equal the ground truth --------------------
    const auto lastWithPose = [](const std::vector<Fields>& v, const char* seenKey) -> const Fields* {
        for (auto it = v.rbegin(); it != v.rend(); ++it)
            if (flag(*it, seenKey)) return &*it;
        return nullptr;
    };
    if (const Fields* f = lastWithPose(got.limelight, "tv")) {
        std::printf("e2e:   limelight botpose (%.3f, %.3f) yaw %.1f vs truth (%.3f, %.3f) yaw %.1f\n", num(*f, "botposeX"), num(*f, "botposeY"),
                    num(*f, "botposeYawDeg"), truth.x, truth.y, truthYawDeg);
        CHECK(std::abs(num(*f, "botposeX") - truth.x) < 0.02);
        CHECK(std::abs(num(*f, "botposeY") - truth.y) < 0.02);
        CHECK(std::abs(wrapDeg(num(*f, "botposeYawDeg") - truthYawDeg)) < 2.0);
    }
    if (const Fields* f = lastWithPose(got.photon, "hasTargets")) {
        std::printf("e2e:   photon pose (%.3f, %.3f) yaw %.1f vs truth (%.3f, %.3f) yaw %.1f\n", num(*f, "poseX"), num(*f, "poseY"),
                    num(*f, "poseYawDeg"), truth.x, truth.y, truthYawDeg);
        if (num(*f, "tags") >= 2) {
            // PhotonLib solves the pose from the true (pitched) camera pose, so a few cm of chassis pitch show up here.
            CHECK(std::abs(num(*f, "poseX") - truth.x) < 0.04);
            CHECK(std::abs(num(*f, "poseY") - truth.y) < 0.04);
            CHECK(std::abs(wrapDeg(num(*f, "poseYawDeg") - truthYawDeg)) < 2.0);
        }
    }

    session.requestStop();
    session.waitUntilFinished();
    std::int64_t pid = 0;
    if (const auto hello = session.runner().robotHello()) pid = hello->pid;
    for (int i = 0; i < 100 && processAlive(pid); ++i) std::this_thread::sleep_for(100ms);
    CHECK_FALSE(processAlive(pid));
    std::printf("e2e: total %.1f s\n", std::chrono::duration<double>(Clock::now() - t0).count());
}

int main(int argc, char** argv) {
    auto skip = [](const std::string& why) {
        std::printf("SKIPPED e2e vision: %s\n", why.c_str());
        return kSkipExitCode;
    };
    if (env("WPIEMU_E2E").empty()) return skip("set WPIEMU_E2E=1 to run the end-to-end test (needs WPILib 2026, Gradle and the vision sample)");
    const fs::path project = !env("WPIEMU_E2E_VISION_PROJECT").empty() ? fs::path(env("WPIEMU_E2E_VISION_PROJECT"))
                                                                        : fs::path(WPIEMU_E2E_VISION_DEFAULT_PROJECT);
    std::error_code ec;
    if (project.empty() || !fs::is_directory(project, ec)) return skip("vision sample not found: '" + project.string() + "' (set WPIEMU_E2E_VISION_PROJECT)");
    const bool windows = fs::path::preferred_separator == '\\';
    if (!fs::exists(project / (windows ? "gradlew.bat" : "gradlew"), ec)) return skip("the project has no Gradle wrapper: " + project.string());
    if (!fs::exists(fs::path(WPIEMU_E2E_EXTENSION), ec)) return skip(std::string("the HALSim extension is not built: ") + WPIEMU_E2E_EXTENSION);
    if (!core::launcher::findWpilibInstall(2026, {}) && env("JAVA_HOME").empty())
        return skip("no WPILib 2026 installation (C:\\Users\\Public\\wpilib\\2026) and no JAVA_HOME");
    doctest::Context context(argc, argv);
    return context.run();
}
