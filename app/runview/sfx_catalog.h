#pragma once

// Turns sound cue ids into resource URLs. Common cues (sdk::commonSoundCues) play files embedded from
// app/runview/assets/sfx; a game's own cues play files its plugin ships in plugins/<gameId>/assets/sfx, embedded under
// /wpiemu/plugins/<gameId>/sfx. The catalog knows no season: it only applies those two rules to the descriptors.

#include <QString>
#include <QStringList>
#include <QUrl>

#include <map>
#include <optional>
#include <vector>

#include "wpiemu/sdk/game/sound_cue.h"

namespace wpiemu::app::runview {

QUrl commonSoundUrl(const QString& asset);
QUrl pluginSoundUrl(const QString& gameId, const QString& asset);

class SfxCatalog {
public:
    // Replaces the catalog: the common cues plus `gameCues` owned by plugin `gameId`. Returns what was wrong (a cue id
    // without the plugin's prefix, or a sound file that is not embedded); such cues are left out and the rest still work.
    QStringList set(const QString& gameId, const std::vector<sdk::SoundCueInfo>& gameCues);

    std::optional<QUrl> resolve(const QString& cueId) const;
    // Every distinct sound the catalog can play.
    std::vector<QUrl> urls() const;

private:
    std::map<QString, QUrl> cues_;
};

}  // namespace wpiemu::app::runview
