#pragma once

// Driver station state machine: enable/disable/e-stop, mode, alliance, match sequencing and the DsState
// the robot sees. Pure logic on top of QObject signals; the DS panel and the run page only bind to it.

#include <QObject>
#include <QSettings>
#include <QString>

#include <array>
#include <vector>

#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::app::runview {

// One period of a timed match. Plain data, no Qt: unit-tested directly.
struct MatchSegment {
    sdk::proto::RobotMode mode = sdk::proto::RobotMode::Teleop;
    bool enabled = true;
    double durationS = 0.0;
};

class MatchTimer {
public:
    void begin(std::vector<MatchSegment> segments);
    void stop();
    // Advances by dt seconds of sim time; returns true when the active segment (or running state) changed.
    bool advance(double dtSeconds);

    bool running() const { return running_; }
    bool finished() const { return finished_; }
    int segmentIndex() const { return index_; }
    const MatchSegment* current() const { return running_ ? &segments_[static_cast<std::size_t>(index_)] : nullptr; }
    double remainingS() const;   // time left in the active segment; 0 when not running
    double elapsedS() const { return elapsed_; }

private:
    std::vector<MatchSegment> segments_;
    int index_ = 0;
    double elapsed_ = 0.0;      // total
    double segmentElapsed_ = 0.0;
    bool running_ = false;
    bool finished_ = false;
};

struct MatchSettings {
    double autoSeconds = 20.0;
    double gapSeconds = 3.0;       // robot disabled between auto and teleop (Practice)
    double teleopSeconds = 140.0;  // 2:20
};

// What each practice-match segment length may be set to, in the setup panel and in the stored settings.
struct SecondsRange {
    double min;
    double max;
};
inline constexpr SecondsRange kAutoSecondsRange{1.0, 600.0};
inline constexpr SecondsRange kGapSecondsRange{0.0, 60.0};
inline constexpr SecondsRange kTeleopSecondsRange{1.0, 1200.0};

// What the user picks on the DS. Auto / Teleop / Test map to the wire RobotMode; Practice is a DS-only
// concept: Enable runs autonomous, a disabled gap, then teleop on the match clock.
enum class DsMode { Auto, Teleop, Practice, Test };

bool equalDsState(const sdk::proto::DsState& a, const sdk::proto::DsState& b);

class DriverStationModel : public QObject {
    Q_OBJECT
public:
    explicit DriverStationModel(QSettings* settings = nullptr, QObject* parent = nullptr);

    bool enabled() const { return enabled_; }
    bool eStopped() const { return eStopped_; }
    // The mode the robot sees right now (follows the segments of a running Practice match).
    sdk::proto::RobotMode mode() const;
    DsMode dsMode() const;
    bool practice() const { return practice_; }
    sdk::Alliance alliance() const { return alliance_; }
    int station() const { return station_; }
    const QString& gameMessage() const { return gameMessage_; }
    bool robotAvailable() const { return robotAvailable_; }
    bool canEnable() const { return robotAvailable_ && !eStopped_; }
    const MatchSettings& matchSettings() const { return matchSettings_; }
    const MatchTimer& match() const { return match_; }
    bool matchActive() const { return match_.running(); }
    // Human readable phase of the running match ("Auto", "Transition", "Teleop"), empty when none.
    QString matchPhaseText() const;

    // The robot-visible state without joysticks. matchTimeS is -1 outside a timed match.
    sdk::proto::DsState state() const;

    // Enabling needs an available robot and no latched e-stop; in Practice it starts the match sequence.
    // Disabling also ends a running match.
    void setEnabled(bool enabled);
    void disable() { setEnabled(false); }
    void eStop();
    void clearEStop();
    void setMode(sdk::proto::RobotMode mode);   // selects Auto / Teleop / Test (leaves Practice)
    void setDsMode(DsMode mode);
    void setAlliance(sdk::Alliance alliance);
    void setStation(int station);   // 1..3
    void setGameMessage(const QString& message);
    void setMatchSettings(const MatchSettings& settings);
    bool startMatch();
    void stopMatch();

    // Environment inputs.
    void setRobotAvailable(bool available);   // false => forced disable
    void onSimTime(std::int64_t simTimeUs);   // advances the match with sim time (pause / time scale aware)
    void onSimReset();

    // Emits stateReady when the composed state changed or `keepAliveMs` passed since the last send.
    bool publish(const std::array<sdk::proto::JoystickState, 6>& joysticks, std::int64_t nowMs, bool force = false);

    static constexpr int kKeepAliveMs = 500;

signals:
    void changed();
    void stateReady(const wpiemu::sdk::proto::DsState& state);

private:
    void applySegment();
    void load();
    void save() const;

    QSettings* settings_;
    bool enabled_ = false;
    bool eStopped_ = false;
    bool robotAvailable_ = false;
    sdk::proto::RobotMode mode_ = sdk::proto::RobotMode::Teleop;
    bool practice_ = false;
    sdk::Alliance alliance_ = sdk::Alliance::Red;
    int station_ = 1;
    QString gameMessage_;
    MatchSettings matchSettings_;
    MatchTimer match_;
    std::int64_t lastSimUs_ = -1;

    sdk::proto::DsState lastSent_;
    std::int64_t lastSentMs_ = -1;
    bool sentOnce_ = false;
};

}  // namespace wpiemu::app::runview
