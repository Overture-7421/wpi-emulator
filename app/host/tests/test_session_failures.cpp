// RobotSession failure handling against synthetic projects: the "gradlew" is a tiny script, so no WPILib, JDK or network is
// involved. A fake extension (an IPC client in the test) plays the robot side of the handshake where needed.
#include <doctest/doctest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <thread>

#include "test_support.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/handshake.h"
#include "wpiemu/ipc/local_ipc.h"
#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/ipc/lifecycle.h"

namespace fs = std::filesystem;
using namespace wpiemu;
using namespace wpiemu::app::host;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

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
        static int counter = 0;
        root = fs::temp_directory_path() / ("wpiemu-host-test-" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count() % 100000000) + "-" + std::to_string(counter++));
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
    ~SyntheticProject() {
        std::error_code ec;
        for (int i = 0; i < 10 && fs::exists(root, ec); ++i) {
            fs::remove_all(root, ec);
            if (fs::exists(root, ec)) std::this_thread::sleep_for(200ms);
        }
    }
    fs::path dir() const { return root / "project"; }
    void signalStop() const { std::ofstream(dir() / "stop.flag").put('x'); }
    void addVendordep(const std::string& fileName, const std::string& json) const {
        fs::create_directories(dir() / "vendordeps");
        std::ofstream(dir() / "vendordeps" / fileName, std::ios::binary) << json;
    }

    Options options(Mode mode = Mode::Run) const {
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
        c.project.path = dir().string();
        c.project.language = "java";
        c.project.wpilibYear = year;
        return c;
    }
};

template <class Pred>
bool waitUntil(RobotSession& s, std::chrono::milliseconds timeout, Pred&& pred, SessionStatus* out = nullptr) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        const SessionStatus st = s.status();
        if (out != nullptr) *out = st;
        if (pred(st)) return true;
        std::this_thread::sleep_for(20ms);
    }
    return false;
}

// The robot side of the handshake, played by the test instead of halsim_wpiemu.
struct FakeExtension {
    sdk::proto::MessageChannel channel;
    static std::unique_ptr<FakeExtension> connect(const std::string& endpoint) {
        auto client = ipc::makeLocalClient();
        auto transport = client->connect(endpoint, 10s);
        if (!transport) return nullptr;
        return std::unique_ptr<FakeExtension>(new FakeExtension(std::move(transport).value()));
    }
    explicit FakeExtension(std::unique_ptr<sdk::proto::ITransport> t) : channel(std::move(t), std::make_shared<ipc::BinaryMessageCodec>()) {}
};

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
    auto run = RobotSession::start(reg.view(), c, p.options(Mode::Run));
    REQUIRE_FALSE(run.ok());
    CHECK(run.error().message.find("errors") != std::string::npos);

    auto discovery = RobotSession::start(reg.view(), c, p.options(Mode::Armed));
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
    REQUIRE(waitUntil(s, 20s, [](const SessionStatus& x) { return x.process == ProcessState::FailedToStart; }, &st));
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
    REQUIRE(waitUntil(s, 20s, [](const SessionStatus& x) { return x.link == LinkState::HandshakeTimeout; }, &st));
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

    auto ext = FakeExtension::connect(s.endpoint());
    REQUIRE(ext != nullptr);
    sdk::proto::Hello hello;
    hello.protocolVersion = 0;   // below kMinCompatibleProtocolVersion
    hello.pid = 1;
    CHECK_FALSE(ipc::performClientHandshake(ext->channel, hello, 5s).ok());

    SessionStatus st;
    REQUIRE(waitUntil(s, 10s, [](const SessionStatus& x) { return x.link == LinkState::ProtocolMismatch; }, &st));
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

        auto ext = FakeExtension::connect(s.endpoint());
        REQUIRE(ext != nullptr);
        sdk::proto::Hello hello;
        hello.pid = 4242;
        hello.programName = "fake robot";
        REQUIRE(ipc::performClientHandshake(ext->channel, hello, 5s).ok());
        SessionStatus st;
        REQUIRE(waitUntil(s, 10s, [](const SessionStatus& x) { return x.link == LinkState::Connected; }, &st));
        CHECK(st.process == ProcessState::Running);
        CHECK_FALSE(st.programStarted);   // the constructor has not finished yet: no ticks, no watchdog

        REQUIRE(ext->channel.send(sdk::proto::makeLifecycleLog(sdk::proto::kLifecycleProgramStarted)).ok());
        REQUIRE(waitUntil(s, 10s, [](const SessionStatus& x) { return x.programStarted; }));

        p.signalStop();   // the "robot" exits with code 7
        REQUIRE(waitUntil(s, 20s, [](const SessionStatus& x) { return x.process == ProcessState::Crashed; }, &st));
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

// ---- the PhotonVision message definition the runner is told about (the project's photonlib vendordep) -------------------------

namespace {

constexpr const char* kPhotonUuid = "4b2ff16a964b5e2bf04be0c1454d91c4";

std::string photonlibJson(const std::string& version) { return R"json({"name": "photonlib", "version": ")json" + version + "\"}"; }

bool logged(RobotSession& s, sdk::LogLevel level, const std::string& source, const std::string& text) {
    for (const LogEntry& e : s.takeLogs())
        if (e.level == level && e.source == source && e.text.find(text) != std::string::npos) return true;
    return false;
}

}  // namespace

TEST_CASE("photon profile: a verified release maps to its message definition, anything else to unverified, no photonlib to the default") {
    TestRegistries reg;
    struct Case { const char* version; const char* expected; };
    const Case cases[] = {
        {"v2026.3.4", kPhotonUuid},
        {"2026.1.1", kPhotonUuid},
        {"2025.3.2", kPhotonUuid},
        {"2026.9.0", kPhotonUuidUnverified},  // newer than any verified release
        {"2024.3.1", kPhotonUuidUnverified},
        {"garbage", kPhotonUuidUnverified},
        {"", kPhotonUuidUnverified},  // present, but no version to trust
    };
    for (const Case& c : cases) {
        INFO(c.version);
        SyntheticProject p(kWaitScript);
        p.addVendordep("photonlib.json", photonlibJson(c.version));
        CHECK(photonMessageUuidFor(p.dir(), reg.json) == c.expected);
    }

    SyntheticProject unreadable(kWaitScript);
    unreadable.addVendordep("photonlib.json", "[[[");
    CHECK(photonMessageUuidFor(unreadable.dir(), reg.json) == kPhotonUuidUnverified);

    SyntheticProject other(kWaitScript);
    other.addVendordep("Phoenix6.json", R"json({"name": "CTRE-Phoenix (v6)", "version": "26.1.0"})json");
    CHECK(photonMessageUuidFor(other.dir(), reg.json).empty());
    SyntheticProject plain(kWaitScript);
    CHECK(photonMessageUuidFor(plain.dir(), reg.json).empty());
    CHECK(photonMessageUuidFor(plain.root / "nope", reg.json).empty());  // a failed inspection never fails the session
}

TEST_CASE("start gives the runner the profile of the project's PhotonLib") {
    TestRegistries reg;
    struct Case { const char* version; const char* expected; };
    const Case cases[] = {{"v2026.3.4", kPhotonUuid}, {"2026.9.0", kPhotonUuidUnverified}, {nullptr, ""}};
    for (const Case& c : cases) {
        INFO((c.version != nullptr ? c.version : "no photonlib"));
        SyntheticProject p(kWaitScript);
        if (c.version != nullptr) p.addVendordep("photonlib.json", photonlibJson(c.version));
        auto r = RobotSession::start(reg.view(), p.config(), p.options());
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        CHECK(r.value()->photonMessageUuid() == c.expected);
        r.value()->requestStop();
        r.value()->waitUntilFinished();
    }
}

TEST_CASE("an unverified PhotonLib is called out only when a PhotonVision camera would be disabled by it") {
    TestRegistries reg;
    auto addCamera = [](sdk::RobotConfig& config, sdk::CameraVendor vendor) {
        sdk::CameraConfig camera;
        camera.id = "front";
        camera.vendor = vendor;
        config.vision.cameras.push_back(camera);
    };
    // Starts a session over a project with an unverified PhotonLib and reports whether the console explains the consequence.
    auto warns = [&](const std::function<void(sdk::RobotConfig&)>& configure) {
        SyntheticProject p(kWaitScript);
        p.addVendordep("photonlib.json", photonlibJson("2026.9.0"));
        sdk::RobotConfig config = p.config();
        configure(config);
        auto r = RobotSession::start(reg.view(), config, p.options());
        REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
        const bool called = logged(*r.value(), sdk::LogLevel::Warn, "vision", "PhotonVision cameras are disabled");
        r.value()->requestStop();
        r.value()->waitUntilFinished();
        return called;
    };
    CHECK(warns([&](sdk::RobotConfig& c) { addCamera(c, sdk::CameraVendor::PhotonVision); }));
    CHECK_FALSE(warns([&](sdk::RobotConfig& c) { addCamera(c, sdk::CameraVendor::Limelight); }));  // needs no vendor library
    CHECK_FALSE(warns([](sdk::RobotConfig&) {}));                                                  // no cameras
    CHECK_FALSE(warns([&](sdk::RobotConfig& c) {
        addCamera(c, sdk::CameraVendor::PhotonVision);
        c.vision.photonMessageUuid = kPhotonUuid;  // the user's override wins: nothing to warn about
    }));
}

TEST_CASE("a relative vision layout file resolves against the opened project, not the path stored in the configuration") {
    TestRegistries reg;
    SyntheticProject p(kWaitScript);
    fs::create_directories(p.dir() / "tags");
    std::ofstream(p.dir() / "tags" / "field.json", std::ios::binary)
        << R"({"tags":[{"ID":3,"pose":{"translation":{"x":5.0,"y":2.0,"z":0.5},
                "rotation":{"quaternion":{"W":1.0,"X":0.0,"Y":0.0,"Z":0.0}}}}],"field":{"length":16.5,"width":8.0}})";

    sdk::RobotConfig config = p.config();
    config.project.path = (p.root / "an-old-checkout").string();  // what a committed robot.json carries once the project moved
    sdk::CameraConfig camera;
    camera.id = "front";
    camera.vendor = sdk::CameraVendor::Limelight;
    camera.ntName = "limelight";
    config.vision.cameras.push_back(camera);
    config.vision.layoutFile = "tags/field.json";

    auto r = RobotSession::start(reg.view(), config, p.options());
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    RobotSession& s = *r.value();
    const auto snap = s.latestSnapshot();
    REQUIRE(snap != nullptr);
    CHECK(snap->fiducials().size() == 1);  // the layout was found, so the engine exists
    for (const sdk::HubWarning& w : snap->warnings()) CHECK(w.code != sdk::validation_code::kVisionLayoutMissing);
    s.requestStop();
    s.waitUntilFinished();
}
