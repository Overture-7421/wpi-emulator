#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include "launcher_test_support.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

// RobotLauncher: phases, cancellation and the build-then-run flow, driven by scripted fake processes, plus one end-to-end run
// through a real fake gradlew.

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::test_runtime::CollectingSink;
using wpiemu::test_runtime::ScratchDir;
using wpiemu::test_runtime::kWindows;
using wpiemu::test_runtime::makeRequest;
using wpiemu::test_runtime::shell;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

struct FakeScript {
    int exitCode = 0;
    bool blockUntilKilled = false;
    bool failToStart = false;
    std::function<void(const ProcessSpec&)> onStart;
    std::vector<std::string> lines;
};

struct FakeWorld {
    std::mutex m;
    std::vector<ProcessSpec> started;
    std::deque<FakeScript> scripts;
    std::atomic<int> killed{0};
    std::atomic<int> stopRequested{0};
    std::atomic<bool> rescue{false};
    std::function<void()> onWait;
};

class FakeProcess final : public IRobotProcess {
public:
    FakeProcess(FakeWorld& w, FakeScript s) : world_(w), script_(std::move(s)) {}
    ~FakeProcess() override { kill(); }
    Status start(const ProcessSpec& spec, ILogSink& sink) override {
        if (script_.failToStart) return makeError(ErrorCode::NotFound, "program not found: " + spec.program);
        {
            std::lock_guard lock(world_.m);
            world_.started.push_back(spec);
        }
        if (script_.onStart) script_.onStart(spec);
        for (const std::string& l : script_.lines) sink.line(OutputStream::Stdout, l);
        std::lock_guard lock(m_);
        started_ = true;
        if (!script_.blockUntilKilled) finish(script_.exitCode, false);
        return okStatus();
    }
    bool running() const override {
        std::lock_guard lock(m_);
        return started_ && !exited_;
    }
    std::int64_t pid() const override { return 1234; }
    std::optional<ExitInfo> exitInfo() const override {
        std::lock_guard lock(m_);
        return exited_ ? std::optional<ExitInfo>(exit_) : std::nullopt;
    }
    bool waitForExit(std::chrono::milliseconds t) override {
        if (world_.onWait) world_.onWait();
        std::unique_lock lock(m_);
        const bool done = cv_.wait_for(lock, t, [this] { return exited_ || !started_ || world_.rescue; });
        if (world_.rescue && !exited_) finish(-1, false);
        return done;
    }
    void requestStop() override { ++world_.stopRequested; }
    void kill() override {
        std::lock_guard lock(m_);
        if (started_ && !exited_) {
            ++world_.killed;
            finish(137, true);
        }
    }

private:
    void finish(int code, bool killed) {
        exit_.exitCode = code;
        exit_.killedByUs = killed;
        exited_ = true;
        cv_.notify_all();
    }
    FakeWorld& world_;
    FakeScript script_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool started_ = false, exited_ = false;
    ExitInfo exit_;
};

ProcessFactory fakeFactory(FakeWorld& w) {
    return [&w]() -> std::unique_ptr<IRobotProcess> {
        FakeScript s;
        {
            std::lock_guard lock(w.m);
            if (!w.scripts.empty()) {
                s = std::move(w.scripts.front());
                w.scripts.pop_front();
            }
        }
        return std::make_unique<FakeProcess>(w, std::move(s));
    };
}

bool waitFor(const std::function<bool()>& pred, std::chrono::milliseconds timeout = 5s) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return pred();
}

struct StartupGate {
    void hold() {
        std::unique_lock lock(mutex);
        entered = true;
        cv.notify_all();
        if (!cv.wait_for(lock, 5s, [this] { return released; })) timedOut = true;
    }
    bool waitUntilEntered() {
        std::unique_lock lock(mutex);
        return cv.wait_for(lock, 2s, [this] { return entered; });
    }
    void release() {
        std::lock_guard lock(mutex);
        released = true;
        cv.notify_all();
    }
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false;
    bool released = false;
    bool timedOut = false;
};

}  // namespace

TEST_CASE("launcher: cancellation during startup is handed to the published process") {
    ScratchDir dir;
    FakeWorld world;
    StartupGate gate;
    world.scripts.push_back({0, true, false, [&](const ProcessSpec&) { gate.hold(); }, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    const bool entered = gate.waitUntilEntered();
    launcher.requestStop(0ms);
    launcher.requestStop(10s);  // cannot extend the first cancellation deadline
    gate.release();
    const bool completed = launcher.waitUntilFinished(2s);
    // The fake never exits on its own. Rescue only after recording failure, so a regression cannot hang the suite.
    if (!completed) world.rescue = true;
    launcher.stop(0ms);
    CHECK(entered);
    CHECK_FALSE(gate.timedOut);
    CHECK(completed);
    CHECK(world.killed == 1);
    CHECK(world.stopRequested == 1);
    CHECK(launcher.status().phase == LaunchPhase::Stopped);
}

TEST_CASE("launcher: cancellation is observed again after a bounded process wait") {
    ScratchDir dir;
    FakeWorld world;
    StartupGate gate;
    world.scripts.push_back({0, true, false, nullptr, {}});
    world.onWait = [&] { gate.hold(); };
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    const bool entered = gate.waitUntilEntered();
    launcher.requestStop(0ms);
    gate.release();
    const bool completed = launcher.waitUntilFinished(2s);
    if (!completed) world.rescue = true;
    launcher.stop(0ms);
    CHECK(entered);
    CHECK_FALSE(gate.timedOut);
    CHECK(completed);
    CHECK(world.killed == 1);
    CHECK(world.stopRequested == 1);
}

TEST_CASE("process: cancellation before native startup prevents creating a child") {
    CollectingSink sink;
    auto process = createSystemProcess();
    process->kill();
    const Status started = process->start(shell("echo should not run"), sink);
    CHECK_FALSE(started.ok());
    if (!started) CHECK(started.error().code == ErrorCode::Cancelled);
    CHECK(process->pid() == 0);
    CHECK_FALSE(process->running());
    CHECK(process->waitForExit(0ms));
    CHECK(sink.texts(OutputStream::Stdout).empty());
}

TEST_CASE("launcher: gradle mode runs one process, phases Running -> Exited, output is forwarded") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, false, false, nullptr, {"> Task :simulateJava", "HAL Extensions: Successfully loaded extension"}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    std::vector<LaunchPhase> phases;
    std::mutex pm;
    launcher.setOnChange([&](const LaunchStatus& s) {
        std::lock_guard lock(pm);
        if (phases.empty() || phases.back() != s.phase) phases.push_back(s.phase);
    });
    CollectingSink sink;
    const LaunchRequest request = makeRequest(dir);
    REQUIRE(launcher.launch(request, sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    const LaunchStatus st = launcher.status();
    CHECK(st.phase == LaunchPhase::Exited);
    REQUIRE(st.exit.has_value());
    CHECK(st.exit->exitCode == 0);
    {
        std::lock_guard lock(pm);
        CHECK(phases == std::vector<LaunchPhase>{LaunchPhase::Running, LaunchPhase::Exited});
    }
    REQUIRE(world.started.size() == 1);
    CHECK(world.started[0].args.back() == "simulateJava");
    CHECK(world.started[0].env.at(env_var::kEndpoint) == request.ipcEndpoint);

    // a finished launcher can launch again
    world.scripts.push_back({5, false, false, nullptr, {}});
    REQUIRE(launcher.launch(request, sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    CHECK(launcher.status().phase == LaunchPhase::Failed);
    CHECK(launcher.status().detail.find("code 5") != std::string::npos);
    CHECK(launcher.status().exit->exitCode == 5);
}

TEST_CASE("launcher: plan errors are returned immediately, a second launch while active is refused") {
    ScratchDir dir;
    FakeWorld world;
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    CollectingSink sink;
    LaunchRequest bad = makeRequest(dir);
    bad.projectDir = dir.path() / "none";
    CHECK(launcher.launch(bad, sink).error().code == ErrorCode::NotFound);
    CHECK(launcher.status().phase == LaunchPhase::Idle);
    CHECK(world.started.empty());

    world.scripts.push_back({0, true, false, nullptr, {}});
    const LaunchRequest good = makeRequest(dir);
    REQUIRE(launcher.launch(good, sink).ok());
    REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Running; }));
    CHECK(launcher.launch(good, sink).error().code == ErrorCode::InvalidState);
    CHECK_FALSE(launcher.waitUntilFinished(20ms));
    launcher.stop(0ms);
    CHECK(launcher.waitUntilFinished(2s));
}

TEST_CASE("launcher: a program that cannot start is a Failed launch with the reason") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, false, true, nullptr, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    CollectingSink sink;
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    CHECK(launcher.status().phase == LaunchPhase::Failed);
    CHECK(launcher.status().detail.find("program not found") != std::string::npos);
}

TEST_CASE("launcher: stop() asks politely, then kills the tree after the grace period") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, true, false, nullptr, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    CollectingSink sink;
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Running; }));
    CHECK(launcher.status().pid == 1234);
    launcher.stop(100ms);
    CHECK(world.stopRequested >= 1);
    CHECK(world.killed == 1);
    const LaunchStatus st = launcher.status();
    CHECK(st.phase == LaunchPhase::Stopped);
    CHECK(st.exit->killedByUs);
    launcher.stop(0ms);  // idempotent
}

TEST_CASE("launcher: the destructor kills a running robot") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, true, false, nullptr, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    {
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
        REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Running; }));
    }
    CHECK(world.killed == 1);
}

TEST_CASE("launcher: ExternalDirect builds first, then runs the JVM from the descriptor") {
    ScratchDir dir;
    FakeWorld world;
    const LaunchRequest request = makeRequest(dir, LaunchMode::ExternalDirect);
    dir.write("robot/build/libs/robot.jar", std::string(100, 'x'));
    // step 1 (describe) "produces" the descriptor; step 2 is the robot
    world.scripts.push_back({0, false, false,
                             [&](const ProcessSpec&) { dir.write("robot/build/sim/release_java.json", R"json({"libraryDir": "/jni", "mainClassName": "frc.robot.Main"})json"); },
                             {"BUILD SUCCESSFUL"}});
    world.scripts.push_back({0, false, false, nullptr, {"robot output"}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    std::vector<LaunchPhase> phases;
    std::mutex pm;
    launcher.setOnChange([&](const LaunchStatus& s) {
        std::lock_guard lock(pm);
        if (phases.empty() || phases.back() != s.phase) phases.push_back(s.phase);
    });
    CollectingSink sink;
    REQUIRE(launcher.launch(request, sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    CHECK(launcher.status().phase == LaunchPhase::Exited);
    {
        std::lock_guard lock(pm);
        CHECK(phases == std::vector<LaunchPhase>{LaunchPhase::Building, LaunchPhase::Running, LaunchPhase::Exited});
    }
    REQUIRE(world.started.size() == 2);
    CHECK(world.started[0].args.back() == "simulateExternalJavaRelease");
    const auto& jvmArgs = world.started[1].args;   // macOS adds -XstartOnFirstThread before -jar
    CHECK(std::find(jvmArgs.begin(), jvmArgs.end(), "-jar") != jvmArgs.end());
    CHECK(world.started[1].env.at(env_var::kHalExtensions) == request.extensionLibrary.string());
}

TEST_CASE("launcher: ExternalDirect failure modes - describe fails, descriptor missing, descriptor unusable") {
    ScratchDir dir;
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    const LaunchRequest request = makeRequest(dir, LaunchMode::ExternalDirect);
    {
        FakeWorld world;
        world.scripts.push_back({1, false, false, nullptr, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(launcher.waitUntilFinished(5s));
        CHECK(launcher.status().phase == LaunchPhase::Failed);
        CHECK(world.started.size() == 1);  // no second step
    }
    {
        FakeWorld world;
        world.scripts.push_back({0, false, false, nullptr, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(launcher.waitUntilFinished(5s));
        CHECK(launcher.status().phase == LaunchPhase::Failed);
        CHECK(launcher.status().detail.find("did not produce") != std::string::npos);
    }
    {
        FakeWorld world;
        world.scripts.push_back({0, false, false, [&](const ProcessSpec&) { dir.write("robot/build/sim/release_java.json", "{}"); }, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(launcher.waitUntilFinished(5s));
        CHECK(launcher.status().phase == LaunchPhase::Failed);
        CHECK(launcher.status().detail.find("cannot plan") != std::string::npos);
    }
    {  // stopped during the build step: the robot is never launched
        FakeWorld world;
        world.scripts.push_back({0, true, false, nullptr, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Building; }));
        launcher.stop(0ms);
        CHECK(launcher.status().phase == LaunchPhase::Stopped);
        CHECK(world.started.size() == 1);
    }
}

TEST_CASE("launcher: end to end with a real fake gradlew - env, init script path and output") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    if (kWindows) {
        dir.write("robot/gradlew.bat",
                  "@echo off\r\necho gradlew args: %*\r\necho ext=%WPIEMU_EXT%\r\necho endpoint=%WPIEMU_IPC_ENDPOINT%\r\necho clock=%WPIEMU_CLOCK%\r\ncd\r\nexit /b 0\r\n");
    } else {
        const fs::path g = dir.write("robot/gradlew", "#!/bin/sh\necho \"gradlew args: $*\"\necho \"ext=$WPIEMU_EXT\"\necho \"endpoint=$WPIEMU_IPC_ENDPOINT\"\necho \"clock=$WPIEMU_CLOCK\"\npwd\n");
        fs::permissions(g, fs::perms::owner_exec | fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::add);
    }
    r.ipcEndpoint = "wpiemu-e2e";
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(systemProcessFactory(), json, logger);
    CollectingSink sink;
    REQUIRE(launcher.launch(r, sink).ok());
    REQUIRE(launcher.waitUntilFinished(30s));
    const LaunchStatus st = launcher.status();
    CHECK_MESSAGE(st.phase == LaunchPhase::Exited, st.detail);

    const std::string initPath = (r.scratchDir / "wpiemu-init.gradle").string();
    const auto lines = sink.texts(OutputStream::Stdout);
    bool args = false;
    for (const std::string& l : lines)
        if (l.rfind("gradlew args:", 0) == 0) {
            args = true;
            CHECK(l.find("--no-daemon") != std::string::npos);
            CHECK(l.find("--console=plain") != std::string::npos);
            CHECK(l.find("-I") != std::string::npos);
            CHECK(l.find(initPath) != std::string::npos);
            CHECK(l.find("simulateJava") != std::string::npos);
        }
    CHECK(args);
    CHECK(sink.has(OutputStream::Stdout, "ext=" + r.extensionLibrary.string()));
    CHECK(sink.has(OutputStream::Stdout, "endpoint=wpiemu-e2e"));
    CHECK(sink.has(OutputStream::Stdout, "clock=master"));
    REQUIRE_FALSE(lines.empty());
    std::error_code ec;
    CHECK(fs::equivalent(fs::path(lines.back()), r.projectDir, ec));  // ran in the project directory
}
