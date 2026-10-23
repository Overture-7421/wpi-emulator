#include <doctest/doctest.h>

#include "console_model.h"
#include "run_control.h"

using namespace wpiemu::app::runview;
using wpiemu::app::HostLogLine;
using wpiemu::app::HostStatus;
using wpiemu::sdk::LinkState;
using wpiemu::sdk::LogLevel;
using wpiemu::sdk::LogStream;
using wpiemu::sdk::ProcessState;

namespace {
HostStatus status(ProcessState p, LinkState l = LinkState::Disconnected, int code = 0, bool started = false, bool simulating = false) {
    HostStatus s;
    s.process = p;
    s.link = l;
    s.exitCode = code;
    s.programStarted = started;
    s.simulating = simulating;
    return s;
}
}  // namespace

TEST_CASE("control state follows the robot and simulation lifecycle") {
    const auto idle = deriveControlState(status(ProcessState::Idle), false);
    CHECK(idle.canRestart);
    CHECK_FALSE(idle.simulating);

    const auto starting = deriveControlState(status(ProcessState::Starting, LinkState::Connecting), false);
    CHECK(starting.canRestart);
    CHECK_FALSE(starting.simulating);

    const auto armed = deriveControlState(status(ProcessState::Running, LinkState::Connected, 0, true), false);
    CHECK(armed.canRestart);
    CHECK_FALSE(armed.simulating);

    const auto running = deriveControlState(status(ProcessState::Running, LinkState::Connected, 0, true, true), false);
    CHECK(running.canRestart);
    CHECK(running.simulating);

    const auto paused = deriveControlState(status(ProcessState::Running, LinkState::Connected, 0, true, true), true);
    CHECK(paused.paused);

    for (auto p : {ProcessState::Exited, ProcessState::Crashed, ProcessState::FailedToStart}) CHECK(deriveControlState(status(p), false).canRestart);
    CHECK_FALSE(deriveControlState(status(ProcessState::Stopping), false).canRestart);
}

TEST_CASE("status presentation covers empty, loading and error states") {
    const auto idle = presentStatus(status(ProcessState::Idle), false);
    CHECK(idle.showOverlay);
    CHECK(idle.offerRestart);

    const auto starting = presentStatus(status(ProcessState::Starting), false);
    CHECK(starting.tone == Tone::Info);
    CHECK(starting.showOverlay);
    CHECK_FALSE(starting.offerRestart);

    const auto initializing = presentStatus(status(ProcessState::Running, LinkState::Connected), false);
    CHECK(initializing.tone == Tone::Info);
    CHECK(initializing.showOverlay);

    const auto armed = presentStatus(status(ProcessState::Running, LinkState::Connected, 0, true), false);
    CHECK(armed.tone == Tone::Success);
    CHECK_FALSE(armed.showOverlay);
    CHECK(armed.chipText.contains(QStringLiteral("ready")));

    const auto ok = presentStatus(status(ProcessState::Running, LinkState::Connected, 0, true, true), true);
    CHECK(ok.tone == Tone::Success);
    CHECK_FALSE(ok.showOverlay);
    CHECK(ok.chipText == QStringLiteral("Simulating"));

    const auto waiting = presentStatus(status(ProcessState::Running, LinkState::Connecting), false);
    CHECK(waiting.showOverlay);
    CHECK(waiting.tone == Tone::Info);

    const auto timeout = presentStatus(status(ProcessState::Running, LinkState::HandshakeTimeout), false);
    CHECK(timeout.tone == Tone::Error);
    CHECK(timeout.offerRestart);
    CHECK(timeout.offerShowConsole);

    const auto mismatch = presentStatus(status(ProcessState::Running, LinkState::ProtocolMismatch), false);
    CHECK(mismatch.tone == Tone::Error);
    CHECK(mismatch.offerRestart);

    const auto stalled = presentStatus(status(ProcessState::Running, LinkState::Stalled), true);
    CHECK(stalled.tone == Tone::Warning);
    CHECK_FALSE(stalled.showOverlay);

    const auto crashed = presentStatus(status(ProcessState::Crashed, LinkState::Disconnected, 137), false);
    CHECK(crashed.tone == Tone::Error);
    CHECK(crashed.offerRestart);
    CHECK(crashed.chipText.contains(QStringLiteral("137")));

    HostStatus failed = status(ProcessState::FailedToStart);
    failed.detail = QStringLiteral("gradlew missing");
    const auto f = presentStatus(failed, false);
    CHECK(f.detail == QStringLiteral("gradlew missing"));
    CHECK(f.tone == Tone::Error);
    CHECK(f.action == ProblemAction::None);   // an unknown failure keeps the raw detail and offers no fix
}

TEST_CASE("status presentation words a recognised launch failure and offers the fix that exists for it") {
    HostStatus stale = status(ProcessState::FailedToStart);
    stale.detail = QStringLiteral("raw gradle tail");
    stale.problem = QStringLiteral("stale_vc_runtime");
    stale.problemFields.insert(QStringLiteral("found"), QStringLiteral("14.36"));
    stale.problemFields.insert(QStringLiteral("expected"), QStringLiteral("14.40"));
    stale.problemFields.insert(QStringLiteral("runtimePath"), QStringLiteral("C:\\jdk\\bin\\msvcp140.dll"));
    const auto s = presentStatus(stale, false);
    CHECK(s.detail.contains(QStringLiteral("14.36")));
    CHECK(s.detail.contains(QStringLiteral("14.40")));
    CHECK(s.detail.contains(QStringLiteral("msvcp140.dll")));
    CHECK(!s.detail.contains(QStringLiteral("raw gradle tail")));
    CHECK(s.action == ProblemAction::FixSetup);
    CHECK(s.offerRestart);

    HostStatus setting = status(ProcessState::FailedToStart);
    setting.problem = QStringLiteral("java_setting_unusable");
    setting.detail = QStringLiteral("/nowhere does not exist.");
    const auto o = presentStatus(setting, false);
    CHECK(o.action == ProblemAction::OpenSettings);
    CHECK(o.detail == QStringLiteral("/nowhere does not exist."));   // the reasons stay visible

    HostStatus offline = status(ProcessState::Crashed, LinkState::Disconnected, 1);
    offline.problem = QStringLiteral("download_failed");
    const auto d = presentStatus(offline, false);
    CHECK(d.action == ProblemAction::None);
    CHECK(d.detail.contains(QStringLiteral("internet")));

    for (const auto* id : {"no_extension_for_season", "season_mismatch"}) {
        HostStatus season = status(ProcessState::FailedToStart);
        season.problem = QString::fromLatin1(id);
        season.detail = QStringLiteral("WPILib year 2027 does not match project year 2026");
        const auto p = presentStatus(season, false);
        CHECK(p.action == ProblemAction::FixSetup);
        CHECK(p.detail == season.detail);
        CHECK(p.offerShowConsole);
    }
    HostStatus loader = status(ProcessState::FailedToStart);
    loader.problem = QStringLiteral("extension_load_failed");
    const auto l = presentStatus(loader, false);
    CHECK(l.action == ProblemAction::FixSetup);
    CHECK(l.detail.contains(QStringLiteral("WPILib season")));
    CHECK(l.offerShowConsole);
}

TEST_CASE("console model is bounded and filterable") {
    ConsoleModel model(nullptr, 5);
    ConsoleFilterModel filter;
    filter.setSourceModel(&model);

    auto entry = [](LogLevel l, LogStream s, const QString& text) {
        HostLogLine e;
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
