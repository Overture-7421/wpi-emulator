#pragma once

#include <QObject>
#include <QSettings>
#include <QString>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "sound_output.h"
#include "wpiemu/sdk/game/sound_cue.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// Turns the sound cues a game raises (snapshots) into playback, without knowing any game: each cue sequence is played
// once, in order, and only while it is fresh. Volume belongs to the operating system; this model only has an on/off
// preference (default on).
class SoundCueModel : public QObject {
    Q_OBJECT
public:
    // A cue older than this (sim time) when first seen, e.g. after the UI stalled, is skipped.
    static constexpr sdk::SimTimeUs kMaxCueAgeUs = 1'000'000;

    // `output` is borrowed (null = silent). `settings` is borrowed and may be null (the preference is then not saved).
    SoundCueModel(ISoundOutput* output, QSettings* settings, QObject* parent = nullptr);

    bool enabled() const { return enabled_; }
    void setEnabled(bool enabled);

    // The cues of the open project's game; reloads the audio only when the list changed.
    void setCatalog(std::vector<sdk::SoundCueInfo> cues);

    // UI thread, every poll. The first snapshot after construction, suspend() or a restart of the sim clock only
    // learns which cues already happened: it never plays them.
    void onSnapshot(const sdk::ISimSnapshot* snapshot);
    // The page is hidden or the simulation is not running: stop playing and forget what was seen.
    void suspend();

signals:
    void enabledChanged(bool enabled);
    void problem(const QString& message);

private:
    ISoundOutput* output_;
    QSettings* settings_;
    bool enabled_ = true;
    std::vector<sdk::SoundCueInfo> catalogList_;
    std::map<std::string, sdk::SoundCueInfo> catalog_;
    std::set<std::string> unknownReported_;

    bool primed_ = false;
    std::uint64_t lastSequence_ = 0;
    std::uint64_t lastTick_ = 0;
    sdk::SimTimeUs lastSimUs_ = 0;
};

}  // namespace wpiemu::app::runview
