#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "host/robot_session.h"
#include "season_probe.h"
#include "test_support.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;

namespace {

// A synthetic test field, not a claimed game layout.
class VisionTestModule final : public sdk::testing::FakeGameModule {
public:
    VisionTestModule() : FakeGameModule("vision_test_season", season::year()) {}

    sdk::Result<std::unique_ptr<sdk::IField>> createField(sdk::IPhysicsWorld& world) override {
        sdk::BodyDesc body;
        body.pose.position = {8.f, 4.f, -0.25f};
        const auto id = world.createBody(body);
        if (!id.valid())
            return sdk::makeError(sdk::ErrorCode::Internal, "cannot create test floor");
        sdk::ShapeDesc shape;
        shape.geometry = sdk::BoxShape{{18.f, 10.f, 0.5f}};
        if (!world.addShape(id, shape).valid())
            return sdk::makeError(sdk::ErrorCode::Internal, "cannot create test floor shape");
        return FakeGameModule::createField(world);
    }

    sdk::AprilTagLayout aprilTagLayout() const override {
        sdk::AprilTagLayout layout;
        layout.name = "synthetic-season-vision-test";
        layout.fieldLengthM = 16.f;
        layout.fieldWidthM = 8.f;
        layout.tags = {{3, {{5.f, 3.75f, 0.5f}, sdk::Quat::fromYaw(static_cast<float>(sdk::kPi))}},
                       {7, {{5.f, 4.25f, 0.5f}, sdk::Quat::fromYaw(static_cast<float>(sdk::kPi))}}};
        return layout;
    }
};

using Fields = std::map<std::string, double>;

Fields parse(const std::string& text) {
    const auto at = text.find("VISION2027 ");
    Fields result;
    if (at == std::string::npos) return result;
    std::istringstream in(text.substr(at + 11));
    in.imbue(std::locale::classic());
    std::string token;
    while (in >> token) {
        const auto eq = token.find('=');
        if (eq == std::string::npos) continue;
        std::istringstream number(token.substr(eq + 1));
        number.imbue(std::locale::classic());
        double value = 0;
        if (number >> value) result[token.substr(0, eq)] = value;
    }
    return result;
}

double get(const Fields& fields, const char* key) {
    const auto it = fields.find(key);
    return it == fields.end() ? -1 : it->second;
}

}  // namespace

TEST_CASE(
    "e2e: season vision - actual PhotonLib and Limelight decode targets, timing, controls and "
    "reset") {
    const std::string language =
        env("WPIEMU_E2E_VISION_LANG").empty() ? "java" : env("WPIEMU_E2E_VISION_LANG");
    REQUIRE((language == "java" || language == "cpp"));
    Scratch scratch("wpiemu-vision-season-");
    const std::filesystem::path project = scratch.root / "project";
    copyProject(season::samples() / ("vision_" + language), project);
    TestRegistries registries;
    REQUIRE(registries.plugins.add(std::make_unique<VisionTestModule>()).ok());

    sdk::RobotConfig config;
    config.name = "season vision integration";
    config.project.language = language;
    config.project.wpilibYear = season::year();
    config.project.gradleRioVersion = season::wpilibVersion();
    config.chassis.sizeM = {0.8f, 0.8f, 0.4f};
    config.chassis.comHeightM = 0.2f;
    for (const auto kind : {sdk::CameraKind::Limelight, sdk::CameraKind::PhotonVision}) {
        sdk::CameraConfig camera;
        camera.id = kind == sdk::CameraKind::Limelight ? "ll" : "photon";
        camera.name = "front";
        camera.kind = kind;
        camera.local.position = {0.f, 0.f, 0.3f};
        camera.hFovDeg = 90;
        camera.vFovDeg = 60;
        camera.latencyMs = 40;
        config.cameras.push_back(camera);
    }
    app::host::Options options;
    options.projectDir = project;
    options.extensionLibrary = core::launcher::locateHalsimExtension(
        season::binaryDir(), &sdk::readEnvironment, season::year(), season::buildExtension());
    options.scratchDir = scratch.root / "scratch";
    options.handshakeTimeout = 5min;
    options.extraGradleArgs = {"--no-daemon"};
    if (env("WPIEMU_E2E_LAUNCH") == "direct")
        options.launchMode = core::launcher::LaunchMode::ExternalDirect;
    if (!env("WPIEMU_E2E_JAVA_HOME").empty()) options.javaHome = env("WPIEMU_E2E_JAVA_HOME");
    options.startPose = sdk::RobotStartPose{2.f, 4.f, 0.f};
    options.timeScale = 0;
    options.driverStation.dsAttached = true;
    auto started = app::host::RobotSession::start(registries.view(), config, options);
    REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
    auto& session = *started.value();
    std::vector<std::string> console;
    std::vector<Fields> samples;
    auto pump = [&](auto timeout, auto predicate) {
        const bool reached = sdk::testing::waitUntil(
            [&] {
                for (const auto& line :
                     drainConsole(session, console, !env("WPIEMU_E2E_VERBOSE").empty())) {
                    auto fields = parse(line.text);
                    if (!fields.empty()) samples.push_back(std::move(fields));
                }
                return predicate() || !session.status().active();
            },
            timeout, 20ms);
        if (!reached || !session.status().active()) dumpConsole(console);
        return reached && session.status().active() && predicate();
    };
    REQUIRE(pump(600s, [&] { return !samples.empty() && get(samples.back(), "pipeline") == 2; }));
    const auto latest = samples.back();
    CHECK(get(latest, "photon") == 1);
    CHECK(get(latest, "tags") == 2);
    CHECK(get(latest, "multi") == 1);
    CHECK(get(latest, "ll") == 1);
    CHECK(get(latest, "llTags") == 2);
    CHECK(get(latest, "llLatency") == doctest::Approx(40).epsilon(0.001));
    CHECK(get(latest, "latency") == doctest::Approx(40).epsilon(0.001));
    CHECK((get(latest, "id") == 3 || get(latest, "id") == 7));
    CHECK(get(latest, "corners") == 4);
    CHECK(get(latest, "calibration") == 1);
    const auto snapshot = session.latestSnapshot();
    REQUIRE(snapshot);
    const auto truth = snapshot->chassis().pose.position;
    for (const auto* key : {"x", "llX"}) CHECK(std::abs(get(latest, key) - truth.x) < 0.005);
    for (const auto* key : {"y", "llY"}) CHECK(std::abs(get(latest, key) - truth.y) < 0.005);
    CHECK(std::abs(get(latest, "targetX") - (5 - truth.x)) < 0.005);
    CHECK(get(latest, "age") >= 40);
    CHECK(get(latest, "age") < 150);
    CHECK(get(latest, "llAge") >= 0);
    CHECK(get(latest, "llAge") <= 120);
    bool driverEmpty = false, disabledEmpty = false;
    for (const auto& line : samples) {
        if (get(line, "driver") == 1 && get(line, "tags") == 0) driverEmpty = true;
        if (get(line, "enabled") == 0 && get(line, "tags") == 0) disabledEmpty = true;
    }
    CHECK(driverEmpty);
    CHECK(disabledEmpty);

    const auto originalEpoch = session.runner().runEpoch();
    session.controller().setStartPlacement(
        sdk::RobotStartPose{2.f, 4.f, static_cast<float>(sdk::kPi)});
    sdk::proto::SimControl reset;
    reset.command = sdk::proto::SimCommand::Reset;
    samples.clear();
    session.controller().control(reset);
    REQUIRE(pump(30s, [&] {
        const auto snap = session.latestSnapshot();
        return session.runner().runEpoch() > originalEpoch && snap &&
               snap->simTimeUs() > 3'000'000 && !samples.empty() &&
               get(samples.back(), "tags") == 0 && get(samples.back(), "ll") == 0;
    }));
    CHECK(get(samples.back(), "photon") == 1);
    const auto resetEpoch = session.runner().runEpoch();
    session.controller().setStartPlacement(sdk::RobotStartPose{2.f, 4.f, 0.f});
    samples.clear();
    session.controller().control(reset);
    REQUIRE(pump(30s, [&] {
        return session.runner().runEpoch() > resetEpoch && !samples.empty() &&
               get(samples.back(), "pipeline") == 2 && get(samples.back(), "tags") == 2 &&
               get(samples.back(), "ll") == 1;
    }));
    CHECK(get(samples.back(), "age") >= 40);
    CHECK(get(samples.back(), "age") < 150);
    REQUIRE(session.runner().robotHello());
    const auto pid = session.runner().robotHello()->pid;
    session.requestStop();
    session.waitUntilFinished();
    CHECK(session.finished());
    CHECK_FALSE(processAlive(pid));
}
