#include <doctest/doctest.h>

#include "console_model.h"
#include "run_control.h"

using namespace wpiemu::app::runview;
using wpiemu::sdk::LogLevel;

namespace {
RunStatus status(RobotProcessState p, LinkState l = LinkState::Disconnected, int code = 0) {
    RunStatus s;
    s.process = p;
    s.link = l;
    s.exitCode = code;
    return s;
}
}  // namespace

TEST_CASE("control state follows the robot process lifecycle") {
    const auto idle = deriveControlState(status(RobotProcessState::Idle), false);
    CHECK(idle.canStart);
    CHECK_FALSE(idle.canStop);
    CHECK(idle.canRestart);
    CHECK_FALSE(idle.canReset);
    CHECK_FALSE(idle.canPause);

    const auto starting = deriveControlState(status(RobotProcessState::Starting, LinkState::Connecting), false);
    CHECK_FALSE(starting.canStart);
    CHECK(starting.canStop);
    CHECK_FALSE(starting.canReset);

    const auto running = deriveControlState(status(RobotProcessState::Running, LinkState::Connected), false);
    CHECK_FALSE(running.canStart);
    CHECK(running.canStop);
    CHECK(running.canReset);
    CHECK(running.canPause);
    CHECK_FALSE(running.canStep);

    const auto paused = deriveControlState(status(RobotProcessState::Running, LinkState::Connected), true);
    CHECK(paused.canStep);
    CHECK(paused.paused);

    for (auto p : {RobotProcessState::Exited, RobotProcessState::Crashed, RobotProcessState::FailedToStart}) {
        const auto s = deriveControlState(status(p), false);
        CHECK(s.canStart);
        CHECK_FALSE(s.canStop);
    }
    CHECK_FALSE(deriveControlState(status(RobotProcessState::Stopping), false).canStart);
}

TEST_CASE("status presentation covers empty, loading and error states") {
    const auto idle = presentStatus(status(RobotProcessState::Idle), false);
    CHECK(idle.showOverlay);
    CHECK(idle.offerStart);

    const auto starting = presentStatus(status(RobotProcessState::Starting), false);
    CHECK(starting.tone == Tone::Info);
    CHECK(starting.showOverlay);
    CHECK_FALSE(starting.offerStart);

    const auto ok = presentStatus(status(RobotProcessState::Running, LinkState::Connected), true);
    CHECK(ok.tone == Tone::Success);
    CHECK_FALSE(ok.showOverlay);

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
    CHECK(crashed.offerStart);
    CHECK(crashed.chipText.contains(QStringLiteral("137")));

    RunStatus failed = status(RobotProcessState::FailedToStart);
    failed.detail = QStringLiteral("gradlew missing");
    const auto f = presentStatus(failed, false);
    CHECK(f.detail == QStringLiteral("gradlew missing"));
    CHECK(f.tone == Tone::Error);
}

TEST_CASE("real DS chip is neutral unless configured") {
    CHECK(realDsTone(RealDsState::NotConfigured) == Tone::Neutral);
    CHECK(realDsTone(RealDsState::Connected) == Tone::Success);
    CHECK(realDsTone(RealDsState::Disconnected) == Tone::Warning);
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
