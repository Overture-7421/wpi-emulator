#include "driver_station_model.h"

#include <algorithm>
#include <cmath>

#include "ui/qt_text.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::runview {
namespace {
constexpr ui::Tr tr{"DriverStation"};
}  // namespace

void MatchTimer::begin(std::vector<MatchSegment> segments) {
    segments_ = std::move(segments);
    index_ = 0;
    elapsed_ = 0.0;
    segmentElapsed_ = 0.0;
    finished_ = false;
    running_ = !segments_.empty();
}

void MatchTimer::stop() {
    running_ = false;
    finished_ = false;
}

bool MatchTimer::advance(double dt) {
    if (!running_ || dt <= 0.0) return false;
    bool changed = false;
    elapsed_ += dt;
    segmentElapsed_ += dt;
    while (running_ && segmentElapsed_ >= segments_[static_cast<std::size_t>(index_)].durationS) {
        segmentElapsed_ -= segments_[static_cast<std::size_t>(index_)].durationS;
        if (index_ + 1 < static_cast<int>(segments_.size())) {
            ++index_;
        } else {
            running_ = false;
            finished_ = true;
        }
        changed = true;
    }
    return changed;
}

double MatchTimer::remainingS() const {
    if (!running_) return 0.0;
    return std::max(0.0, segments_[static_cast<std::size_t>(index_)].durationS - segmentElapsed_);
}

bool equalDsState(const sdk::proto::DsState& a, const sdk::proto::DsState& b) {
    if (a.dsAttached != b.dsAttached || a.fmsAttached != b.fmsAttached || a.enabled != b.enabled || a.eStop != b.eStop ||
        a.mode != b.mode || a.station != b.station || a.matchTimeS != b.matchTimeS ||
        a.gameSpecificMessage != b.gameSpecificMessage || a.sessionId != b.sessionId || a.runEpoch != b.runEpoch)
        return false;
    for (std::size_t i = 0; i < a.joysticks.size(); ++i) {
        const auto& x = a.joysticks[i];
        const auto& y = b.joysticks[i];
        if (x.present != y.present || x.axes != y.axes || x.povs != y.povs || x.buttons != y.buttons ||
            x.buttonCount != y.buttonCount)
            return false;
    }
    return true;
}

DriverStationModel::DriverStationModel(QSettings* settings, QObject* parent) : QObject(parent), settings_(settings) { load(); }

void DriverStationModel::load() {
    if (settings_ == nullptr) return;
    alliance_ = settings_->value(QStringLiteral("runview/ds/alliance"), 0).toInt() == 1 ? sdk::Alliance::Blue : sdk::Alliance::Red;
    station_ = std::clamp(settings_->value(QStringLiteral("runview/ds/station"), 1).toInt(), 1, 3);
    gameMessage_ = settings_->value(QStringLiteral("runview/ds/gameMessage")).toString();
    matchSettings_.autoSeconds = std::clamp(settings_->value(QStringLiteral("runview/ds/autoSeconds"), 20.0).toDouble(), kAutoSecondsRange.min, kAutoSecondsRange.max);
    matchSettings_.teleopSeconds = std::clamp(settings_->value(QStringLiteral("runview/ds/teleopSeconds"), 140.0).toDouble(), kTeleopSecondsRange.min, kTeleopSecondsRange.max);
    matchSettings_.gapSeconds = std::clamp(settings_->value(QStringLiteral("runview/ds/gapSeconds"), 3.0).toDouble(), kGapSecondsRange.min, kGapSecondsRange.max);
}

void DriverStationModel::save() const {
    if (settings_ == nullptr) return;
    settings_->setValue(QStringLiteral("runview/ds/alliance"), alliance_ == sdk::Alliance::Blue ? 1 : 0);
    settings_->setValue(QStringLiteral("runview/ds/station"), station_);
    settings_->setValue(QStringLiteral("runview/ds/gameMessage"), gameMessage_);
    settings_->setValue(QStringLiteral("runview/ds/autoSeconds"), matchSettings_.autoSeconds);
    settings_->setValue(QStringLiteral("runview/ds/teleopSeconds"), matchSettings_.teleopSeconds);
    settings_->setValue(QStringLiteral("runview/ds/gapSeconds"), matchSettings_.gapSeconds);
}

// Between Practice runs the DS sits on Autonomous, ready for the next Enable.
sdk::proto::RobotMode DriverStationModel::mode() const {
    return practice_ && !match_.running() ? sdk::proto::RobotMode::Autonomous : mode_;
}

DsMode DriverStationModel::dsMode() const {
    if (practice_) return DsMode::Practice;
    switch (mode_) {
        case sdk::proto::RobotMode::Autonomous: return DsMode::Auto;
        case sdk::proto::RobotMode::Test: return DsMode::Test;
        case sdk::proto::RobotMode::Teleop: break;
    }
    return DsMode::Teleop;
}

QString DriverStationModel::matchPhaseText() const {
    const MatchSegment* seg = match_.current();
    if (seg == nullptr) return {};
    if (!seg->enabled) return tr("Transition");
    return seg->mode == sdk::proto::RobotMode::Autonomous ? tr("Autonomous") : tr("Teleoperated");
}

sdk::proto::DsState DriverStationModel::state() const {
    sdk::proto::DsState s;
    s.dsAttached = true;
    s.fmsAttached = false;
    s.enabled = enabled_ && !eStopped_;
    s.eStop = eStopped_;
    s.mode = mode();
    s.station = sdk::proto::stationOf(alliance_, station_);
    s.matchTimeS = match_.running() ? match_.remainingS() : -1.0;
    s.gameSpecificMessage = gameMessage_.toStdString();
    return s;
}

void DriverStationModel::setEnabled(bool enabled) {
    if (enabled) {
        if (!canEnable() || enabled_) return;
        if (practice_) {
            startMatch();
            return;
        }
        enabled_ = true;
    } else {
        if (!enabled_ && !match_.running()) return;
        enabled_ = false;
        match_.stop();
    }
    emit changed();
}

void DriverStationModel::eStop() {
    eStopped_ = true;
    enabled_ = false;
    match_.stop();
    emit changed();
}

void DriverStationModel::clearEStop() {
    if (!eStopped_) return;
    eStopped_ = false;
    emit changed();
}

void DriverStationModel::setMode(sdk::proto::RobotMode mode) {
    if (mode == mode_ && !practice_) return;
    // Like the real DS: the mode only changes while disabled.
    enabled_ = false;
    match_.stop();
    practice_ = false;
    mode_ = mode;
    emit changed();
}

void DriverStationModel::setDsMode(DsMode mode) {
    switch (mode) {
        case DsMode::Auto: setMode(sdk::proto::RobotMode::Autonomous); return;
        case DsMode::Teleop: setMode(sdk::proto::RobotMode::Teleop); return;
        case DsMode::Test: setMode(sdk::proto::RobotMode::Test); return;
        case DsMode::Practice: break;
    }
    if (practice_) return;
    enabled_ = false;
    match_.stop();
    practice_ = true;
    mode_ = sdk::proto::RobotMode::Autonomous;
    emit changed();
}

void DriverStationModel::setAlliance(sdk::Alliance alliance) {
    if (alliance == alliance_) return;
    alliance_ = alliance;
    save();
    emit changed();
}

void DriverStationModel::setStation(int station) {
    station = std::clamp(station, 1, 3);
    if (station == station_) return;
    station_ = station;
    save();
    emit changed();
}

void DriverStationModel::setGameMessage(const QString& message) {
    if (message == gameMessage_) return;
    gameMessage_ = message;
    save();
    emit changed();
}

void DriverStationModel::setMatchSettings(const MatchSettings& settings) {
    matchSettings_ = settings;
    save();
    emit changed();
}

bool DriverStationModel::startMatch() {
    if (!canEnable()) return false;
    std::vector<MatchSegment> segments;
    if (!practice_) {
        // Auto / Teleop / Test have no match clock (and so no game timing): Enable just enables until Disable.
        enabled_ = true;
        emit changed();
        return true;
    }
    segments.push_back({sdk::proto::RobotMode::Autonomous, true, matchSettings_.autoSeconds});
    if (matchSettings_.gapSeconds > 0.0) segments.push_back({sdk::proto::RobotMode::Teleop, false, matchSettings_.gapSeconds});
    segments.push_back({sdk::proto::RobotMode::Teleop, true, matchSettings_.teleopSeconds});
    match_.begin(std::move(segments));
    applySegment();
    emit changed();
    return true;
}

void DriverStationModel::stopMatch() {
    if (!match_.running()) return;
    match_.stop();
    enabled_ = false;
    emit changed();
}

void DriverStationModel::applySegment() {
    if (const MatchSegment* seg = match_.current()) {
        mode_ = seg->mode;
        enabled_ = seg->enabled && !eStopped_;
    }
}

void DriverStationModel::setRobotAvailable(bool available) {
    if (available == robotAvailable_) return;
    robotAvailable_ = available;
    if (!available) {
        enabled_ = false;
        match_.stop();
    } else {
        eStopped_ = false;   // a fresh robot session clears the latched e-stop
    }
    emit changed();
}

void DriverStationModel::onSimTime(std::int64_t simTimeUs) {
    const std::int64_t previous = lastSimUs_;
    lastSimUs_ = simTimeUs;
    if (previous < 0 || !match_.running()) return;
    const double dt = sdk::usToSeconds(simTimeUs - previous);
    if (dt <= 0.0 || dt > 5.0) return;   // reset or a long stall: do not jump the clock
    if (match_.advance(dt)) {
        if (match_.running()) applySegment();
        else enabled_ = false;           // match over
        emit changed();
    }
}

void DriverStationModel::onSimReset() {
    lastSimUs_ = -1;
    const bool wasActive = enabled_ || match_.running();
    enabled_ = false;
    match_.stop();
    if (wasActive) emit changed();
}

bool DriverStationModel::publish(const std::array<sdk::proto::JoystickState, 6>& joysticks, std::int64_t nowMs, bool force) {
    sdk::proto::DsState next = state();
    next.joysticks = joysticks;
    const bool due = !sentOnce_ || force || nowMs - lastSentMs_ >= kKeepAliveMs || !equalDsState(next, lastSent_);
    if (!due) return false;
    lastSent_ = next;
    lastSentMs_ = nowMs;
    sentOnce_ = true;
    emit stateReady(next);
    return true;
}

}  // namespace wpiemu::app::runview
