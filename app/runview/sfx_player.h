#pragma once

// Plays sound cues for the run view. Cues come from the Driver Station model (Practice) and from the host (the game
// plugin); the shell sets the user's preferences. Qt-only logic over an ISfxOutput, so it is tested without audio.

#include <QObject>
#include <QSet>
#include <QString>

#include <memory>
#include <vector>

#include "sfx_catalog.h"
#include "sfx_output.h"
#include "wpiemu/sdk/game/sound_cue.h"

namespace wpiemu::app::runview {

class SfxPlayer : public QObject {
    Q_OBJECT
public:
    // `output` is borrowed (and may be null: silent); it must outlive the player.
    explicit SfxPlayer(ISfxOutput* output, QObject* parent = nullptr);

    // The sounds this player can play: the common cues plus the game's own (see SfxCatalog). Preloads them.
    void setCatalog(const QString& gameId, const std::vector<sdk::SoundCueInfo>& gameCues);

    bool enabled() const { return enabled_; }
    int volumePercent() const { return volumePercent_; }
    bool active() const { return active_; }
    void setEnabled(bool enabled);
    void setVolumePercent(int percent);   // clamped to 0..100
    // False while the page is not showing: nothing plays and what is playing stops.
    void setActive(bool active);

public slots:
    // Plays the cue; unknown ids are reported once through problem().
    void play(const QString& cueId);

signals:
    // Something a user should know (missing file, no audio device, unknown cue): shown in the console.
    void problem(const QString& message);

private:
    void reportOnce(const QString& key, const QString& message);

    ISfxOutput* output_;
    SfxCatalog catalog_;
    bool enabled_ = true;
    int volumePercent_ = 70;
    bool active_ = true;
    QSet<QString> reported_;
};

}  // namespace wpiemu::app::runview
