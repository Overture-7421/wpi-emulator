#include <doctest/doctest.h>

#include "console_model.h"
#include "run_control.h"

using namespace wpiemu::app::runview;
using wpiemu::sdk::LogLevel;

namespace {
RunStatus status(RobotProcessState p, LinkState l = LinkState::Disconnected, int code = 0, bool started = false, bool simulating = false) {
    RunStatus s;
    s.process = p;
    s.link = l;
    s.exitCode = code;
    s.programStarted = started;
    s.simulating = simulating;
    return s;
}
}  // namespace

TEST_CASE("control state follows the robot and simulation lifecycle") {
    const auto idle = deriveControlState(status(RobotProcessState::Idle), false);
    CHECK(idle.canRestart);
    CHECK_FALSE(idle.simulating);

    const auto starting = deriveControlState(status(RobotProcessState::Starting, LinkState::Connecting), false);
    CHECK(starting.canRestart);
    CHECK_FALSE(starting.simulating);

    const auto armed = deriveControlState(status(RobotProcessState::Running, LinkState::Connected, 0, true), false);
    CHECK(armed.canRestart);
    CHECK_FALSE(armed.simulating);

    const auto running = deriveControlState(status(RobotProcessState::Running, LinkState::Connected, 0, true, true), false);
    CHECK(running.canRestart);
    CHECK(running.simulating);

    const auto paused = deriveControlState(status(RobotProcessState::Running, LinkState::Connected, 0, true, true), true);
    CHECK(paused.paused);

    for (auto p : {RobotProcessState::Exited, RobotProcessState::Crashed, RobotProcessState::FailedToStart}) CHECK(deriveControlState(status(p), false).canRestart);
    CHECK_FALSE(deriveControlState(status(RobotProcessState::Stopping), false).canRestart);
}

TEST_CASE("status presentation covers empty, loading and error states") {
    const auto idle = presentStatus(status(RobotProcessState::Idle), false);
    CHECK(idle.showOverlay);
    CHECK(idle.offerRestart);

    const auto starting = presentStatus(status(RobotProcessState::Starting), false);
    CHECK(starting.tone == Tone::Info);
    CHECK(starting.showOverlay);
    CHECK_FALSE(starting.offerRestart);

    const auto initializing = presentStatus(status(RobotProcessState::Running, LinkState::Connected), false);
    CHECK(initializing.tone == Tone::Info);
    CHECK(initializing.showOverlay);

    const auto armed = presentStatus(status(RobotProcessState::Running, LinkState::Connected, 0, true), false);
    CHECK(armed.tone == Tone::Success);
    CHECK_FALSE(armed.showOverlay);
    CHECK(armed.chipText.contains(QStringLiteral("ready")));

    const auto ok = presentStatus(status(RobotProcessState::Running, LinkState::Connected, 0, true, true), true);
    CHECK(ok.tone == Tone::Success);
    CHECK_FALSE(ok.showOverlay);
    CHECK(ok.chipText == QStringLiteral("Simulating"));

    const auto waiting = presentStatus(status(RobotProcessState::Running, LinkState::Connecting), false);
    CHECK(waiting.showOverlay);
    CHECK(waiting.tone == Tone::Info);

    const auto timeout = presentStatus(status(RobotProcessState::Running, LinkState::HandshakeTimeout), false);
    CHECK(timeout.tone == Tone::Error);
    CHECK(timeout.offerRestart);
    CHECK(timeout.offerShowConsole);

    const auto mismatch = presentStatus(status(RobotProcessState::Running, LinkState::ProtocolMismatch), false);
    CHECK(mismatch.tone == Tone::Error);
    CHECK(mismatch.offerRestart);

    const auto stalled = presentStatus(status(RobotProcessState::Running, LinkState::Stalled), true);
    CHECK(stalled.tone == Tone::Warning);
    CHECK_FALSE(stalled.showOverlay);

    const auto crashed = presentStatus(status(RobotProcessState::Crashed, LinkState::Disconnected, 137), false);
    CHECK(crashed.tone == Tone::Error);
    CHECK(crashed.offerRestart);
    CHECK(crashed.chipText.contains(QStringLiteral("137")));

    RunStatus failed = status(RobotProcessState::FailedToStart);
    failed.detail = QStringLiteral("gradlew missing");
    const auto f = presentStatus(failed, false);
    CHECK(f.detail == QStringLiteral("gradlew missing"));
    CHECK(f.tone == Tone::Error);
}

TEST_CASE("log level guessing from raw robot output") {
    CHECK(guessLogLevel(QStringLiteral("ERROR 1 something"), LogStream::Stdout) == LogLevel::Error);
    CHECK(guessLogLevel(QStringLiteral("Warning at frc.robot.Robot: x"), LogStream::Stdout) == LogLevel::Warn);
    CHECK(guessLogLevel(QStringLiteral("    at frc.robot.Robot.run(Robot.java:3)"), LogStream::Stderr) == LogLevel::Error);
    CHECK(guessLogLevel(QStringLiteral("hello"), LogStream::Stdout) == LogLevel::Info);
    CHECK(guessLogLevel(QStringLiteral("hello"), LogStream::Stderr) == LogLevel::Warn);
    CHECK(guessLogLevel(QStringLiteral("DEBUG x"), LogStream::Stdout) == LogLevel::Debug);
}

TEST_CASE("console model is bounded and filterable") {
    ConsoleModel model(nullptr, 5);
    ConsoleFilterModel filter;
    filter.setSourceModel(&model);

    auto entry = [](LogLevel l, LogStream s, const QString& text) {
        RunLogEntry e;
        e.level = l;
        e.stream = s;
        e.text = text;
        e.source = QStringLiteral("robot");
        return e;
    };
    for (int i = 0; i < 8; ++i) model.append(entry(LogLevel::Info, LogStream::Stdout, QStringLiteral("line %1").arg(i)));
    CHECK(model.rowCount() == 5);
    CHECK(model.at(0).text == QStringLiteral("line 3"));

    model.clear();
    model.append(entry(LogLevel::Debug, LogStream::Stdout, QStringLiteral("alpha debug")));
    model.append(entry(LogLevel::Warn, LogStream::Stderr, QStringLiteral("beta warn")));
    model.append(entry(LogLevel::Error, LogStream::Stderr, QStringLiteral("gamma error")));
    model.append(entry(LogLevel::Info, LogStream::Emulator, QStringLiteral("delta info")));
    CHECK(filter.rowCount() == 4);

    filter.setMinimumLevel(LogLevel::Warn);
    CHECK(filter.rowCount() == 2);
    filter.setMinimumLevel(LogLevel::Trace);

    filter.setStreamEnabled(LogStream::Stderr, false);
    CHECK(filter.rowCount() == 2);
    filter.setStreamEnabled(LogStream::Stderr, true);

    filter.setSearchText(QStringLiteral("BETA"));
    CHECK(filter.rowCount() == 1);
    CHECK(filter.plainText().contains(QStringLiteral("beta warn")));
    filter.setSearchText({});

    const QString all = filter.plainText();
    CHECK(all.count(QLatin1Char('\n')) == 3);
    const QModelIndexList rows{filter.index(2, 0), filter.index(0, 0)};
    const QString picked = filter.plainText(rows);
    CHECK(picked.indexOf(QStringLiteral("alpha")) < picked.indexOf(QStringLiteral("gamma")));
}
