// RobotSession failure handling against synthetic projects: the "gradlew" is a tiny script, so no WPILib, JDK or network is
// involved. A fake extension (an IPC client in the test) plays the robot side of the handshake where needed.
#include <doctest/doctest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "test_support.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/local_ipc.h"
#include "wpiemu/sdk/testing/fake_extension.h"
#include "wpiemu/sdk/testing/wait.h"

namespace fs = std::filesystem;
using namespace wpiemu;
using namespace wpiemu::app::host;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
using sdk::LinkState;
using sdk::ProcessState;
using sdk::SessionMode;
using sdk::SessionStatus;
using Clock = std::chrono::steady_clock;

TEST_CASE("start placement uses the configured field pose without changing the default") {
    sdk::FieldDescription field;
    field.lengthM = 16.5f;
    field.widthM = 8.f;
    const auto normal = startPlacementFor(&field);
    CHECK(normal.xM == 2.f);
    CHECK(normal.yM == 4.f);
    CHECK(normal.headingRad == 0.f);
    const sdk::RobotStartPose pose{4.51f, 0.52654f, 1.2f};
    for (const auto* description : {&field, static_cast<sdk::FieldDescription*>(nullptr)}) {
        const auto custom = startPlacementFor(description, pose);
        CHECK(custom.xM == pose.xM);
        CHECK(custom.yM == pose.yM);
        CHECK(custom.headingRad == pose.headingRad);
    }
}

namespace {

constexpr bool kWindows = fs::path::preferred_separator == '\\';

#if defined(_WIN32)
// Waits (polling a flag file) so the test decides when the "robot" ends.
const char* kWaitScript = "@echo off\r\necho fake gradle started\r\n:loop\r\nif exist \"%~dp0stop.flag\" exit /b 7\r\nping -n 2 127.0.0.1 >nul\r\ngoto loop\r\n";
const char* kFailScript = "@echo off\r\necho fake gradle: BUILD FAILED\r\necho something went wrong 1>&2\r\nexit /b 3\r\n";
#else
const char* kWaitScript = "#!/bin/sh\necho fake gradle started\nwhile [ ! -f \"$(dirname \"$0\")/stop.flag\" ]; do sleep 1; done\nexit 7\n";
const char* kFailScript = "#!/bin/sh\necho fake gradle: BUILD FAILED\necho something went wrong 1>&2\nexit 3\n";
#endif

struct SyntheticProject {
    fs::path root;
    fs::path extension;

    explicit SyntheticProject(const char* script, bool withWrapper = true) {
        root = uniqueTempPath("wpiemu-host-test-");
        fs::create_directories(root / "project");
        if (withWrapper) {
            const fs::path wrapper = root / "project" / (kWindows ? "gradlew.bat" : "gradlew");
            std::ofstream(wrapper, std::ios::binary) << script;
#if !defined(_WIN32)
            fs::permissions(wrapper, fs::perms::owner_all);
#endif
        }
        extension = root / "halsim_wpiemu_fake";
        std::ofstream(extension).put('x');
    }
    ~SyntheticProject() { removeTree(root, 10, 200ms); }
    fs::path dir() const { return root / "project"; }
    void signalStop() const { std::ofstream(dir() / "stop.flag").put('x'); }

    Options options(SessionMode mode = SessionMode::Run) const {
        Options o;
        o.mode = mode;
        o.projectDir = dir();
        o.extensionLibrary = extension;
        o.scratchDir = root / "scratch";
        o.stopGrace = 100ms;
        return o;
    }
    sdk::RobotConfig config(int year = 2026) const {
        sdk::RobotConfig c;
        c.project.language = "java";
        c.project.wpilibYear = year;
        return c;
    }
};

template <class Pred>
bool waitForStatus(RobotSession& s, std::chrono::milliseconds timeout, Pred&& pred, SessionStatus* out = nullptr) {
    return sdk::testing::waitUntil(
        [&] {
            const SessionStatus st = s.status();
            if (out != nullptr) *out = st;
            return pred(st);
        },
        timeout, 20ms);
}

// The robot side, played by the test instead of halsim_wpiemu, over the session's real local transport.
std::unique_ptr<sdk::testing::FakeExtension> connectFakeExtension(const std::string& endpoint) {
    auto client = ipc::makeLocalClient();
    auto transport = client->connect(endpoint, 10s);
    if (!transport) return nullptr;
    return std::make_unique<sdk::testing::FakeExtension>(std::move(transport).value(),
                                                         std::make_shared<ipc::BinaryMessageCodec>());
}

// True when the emulator accepted the Hello the fake sent; false when it refused or dropped the connection.
bool awaitHandshake(sdk::testing::FakeExtension& ext, std::chrono::milliseconds timeout = 5s) {
    sdk::testing::waitUntil([&] { return ext.helloAck.has_value() || ext.disconnected; }, timeout, 5ms,
                            [&] { ext.pump(); });
    return ext.handshaken();
}

}  // namespace

TEST_CASE("start refuses a project without a Gradle wrapper with an actionable message") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript, /*withWrapper=*/false);
    auto r = RobotSession::start(reg.view(), p.config(), p.options());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("Gradle wrapper") != std::string::npos);
}

TEST_CASE("start refuses a missing project folder and a missing extension") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    {
        Options o = p.options();
        o.projectDir = p.root / "nope";
        auto r = RobotSession::start(reg.view(), p.config(), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message.find("project folder") != std::string::npos);
    }
    {
        Options o = p.options();
        o.extensionLibrary = p.root / "missing.dll";
        auto r = RobotSession::start(reg.view(), p.config(), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message.find("HALSim extension") != std::string::npos);
    }
}

TEST_CASE("an invalid model refuses a run but not device discovery") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    sdk::RobotConfig c = p.config();
    c.chassis.massKg = -1.f;   // chassis_invalid (Error)
    auto run = RobotSession::start(reg.view(), c, p.options(SessionMode::Run));
    REQUIRE_FALSE(run.ok());
    CHECK(run.error().message.find("errors") != std::string::npos);

    auto discovery = RobotSession::start(reg.view(), c, p.options(SessionMode::Armed));
    REQUIRE_MESSAGE(discovery.ok(), (discovery.ok() ? "" : discovery.error().message));   // falls back to the bare model
    discovery.value()->requestStop();
    discovery.value()->waitUntilFinished();
}

TEST_CASE("a robot project that ends before the extension connects is FailedToStart with its console tail") {
    TestRegistries reg;
    SyntheticProject p(kFailScript);
    auto r = RobotSession::start(reg.view(), p.config(), p.options());
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    RobotSession& s = *r.value();
    SessionStatus st;
    REQUIRE(waitForStatus(s, 20s, [](const SessionStatus& x) { return x.process == ProcessState::FailedToStart; }, &st));
    CHECK(st.exitCode == 3);
    CHECK(st.detail.find("BUILD FAILED") != std::string::npos);
    CHECK_FALSE(st.active());
    s.waitUntilFinished();
    CHECK(s.finished());
}

TEST_CASE("the handshake timeout reports HandshakeTimeout and stops the process tree") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    Options o = p.options();
    o.handshakeTimeout = 700ms;
    auto r = RobotSession::start(reg.view(), p.config(), o);
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    RobotSession& s = *r.value();
    SessionStatus st;
    REQUIRE(waitForStatus(s, 20s, [](const SessionStatus& x) { return x.link == LinkState::HandshakeTimeout; }, &st));
    CHECK(st.process == ProcessState::FailedToStart);
    CHECK(st.detail.find("did not connect") != std::string::npos);
    s.waitUntilFinished();
    CHECK(s.finished());
}

TEST_CASE("an extension with an unsupported protocol version is reported as ProtocolMismatch") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    auto r = RobotSession::start(reg.view(), p.config(), p.options());
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    RobotSession& s = *r.value();

    auto ext = connectFakeExtension(s.endpoint());
    REQUIRE(ext != nullptr);
    ext->hello(0, 0);   // below kMinCompatibleProtocolVersion
    CHECK_FALSE(awaitHandshake(*ext));

    SessionStatus st;
    REQUIRE(waitForStatus(s, 10s, [](const SessionStatus& x) { return x.link == LinkState::ProtocolMismatch; }, &st));
    CHECK(st.process == ProcessState::FailedToStart);
    CHECK(st.detail.find("protocol") != std::string::npos);
    s.waitUntilFinished();
}

TEST_CASE("a connected robot that then dies is Crashed with its exit code, stop reports Exited") {
    TestRegistries reg;
    {
        SyntheticProject p(kWaitScript);
        auto r = RobotSession::start(reg.view(), p.config(), p.options());
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        RobotSession& s = *r.value();
        CHECK(s.status().process == ProcessState::Starting);

        auto ext = connectFakeExtension(s.endpoint());
        REQUIRE(ext != nullptr);
        ext->hello();
        REQUIRE(awaitHandshake(*ext));
        SessionStatus st;
        REQUIRE(waitForStatus(s, 10s, [](const SessionStatus& x) { return x.link == LinkState::Connected; }, &st));
        CHECK(st.process == ProcessState::Running);
        CHECK_FALSE(st.programStarted);   // the constructor has not finished yet: no ticks, no watchdog

        ext->programStarted();
        REQUIRE(waitForStatus(s, 10s, [](const SessionStatus& x) { return x.programStarted; }));

        p.signalStop();   // the "robot" exits with code 7
        REQUIRE(waitForStatus(s, 20s, [](const SessionStatus& x) { return x.process == ProcessState::Crashed; }, &st));
        CHECK(st.exitCode == 7);
        CHECK(st.link == LinkState::Disconnected);
        s.waitUntilFinished();
    }
    {
        SyntheticProject p(kWaitScript);
        auto r = RobotSession::start(reg.view(), p.config(), p.options());
        REQUIRE(r.ok());
        RobotSession& s = *r.value();
        const auto t0 = Clock::now();
        s.requestStop();
        CHECK(s.stopRequested());
        s.waitUntilFinished();
        CHECK(std::chrono::duration<double>(Clock::now() - t0).count() < 10.0);
        CHECK(s.status().process == ProcessState::Exited);
    }
}

TEST_CASE("a season without a game plugin runs core-only with a floor under the robot") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    sdk::RobotConfig c = p.config(2099);
    c.chassis.massKg = 50.f;
    auto r = RobotSession::start(reg.view(), c, p.options());
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    RobotSession& s = *r.value();
    CHECK(s.coreOnly());
    CHECK(s.gameModule() == nullptr);
    const auto snap = s.latestSnapshot();
    REQUIRE(snap != nullptr);
    CHECK(snap->chassis().pose.position.z > 0.f);
    s.requestStop();
    s.waitUntilFinished();
}

TEST_CASE("the launch's physics worker count reaches the world and its threads are torn down with the session") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    const auto t0 = Clock::now();
    {
        Options o = p.options();
        o.physicsWorkers = 2;
        auto r = RobotSession::start(reg.view(), p.config(), o);
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        RobotSession& s = *r.value();
        const auto snap = s.latestSnapshot();
        REQUIRE(snap != nullptr);
        CHECK(snap->physicsStats().workerCount == 2);
        s.requestStop();
        s.waitUntilFinished();
    }   // the Box3D world and its helper threads are destroyed with the session
    CHECK(std::chrono::duration<double>(Clock::now() - t0).count() < 15.0);
}

TEST_CASE("the destructor kills the robot process tree (app exit)") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    const auto t0 = Clock::now();
    {
        auto r = RobotSession::start(reg.view(), p.config(), p.options());
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        std::this_thread::sleep_for(300ms);
        // leaving the scope without a stop request
    }
    CHECK(std::chrono::duration<double>(Clock::now() - t0).count() < 15.0);
    // The synthetic project folder can be deleted only when no process of the tree still holds it.
    std::error_code ec;
    std::this_thread::sleep_for(200ms);
    fs::remove(p.dir() / "gradlew.bat", ec);
}
