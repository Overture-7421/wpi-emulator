#include "sfx_player.h"

#include <algorithm>

#include "ui/qt_text.h"

namespace wpiemu::app::runview {
namespace {
constexpr ui::Tr tr{"Sound"};
}  // namespace

SfxPlayer::SfxPlayer(ISfxOutput* output, QObject* parent) : QObject(parent), output_(output) {}

void SfxPlayer::setCatalog(const QString& gameId, const std::vector<sdk::SoundCueInfo>& gameCues) {
    for (const QString& message : catalog_.set(gameId, gameCues)) reportOnce(message, message);
    if (output_ == nullptr || !output_->available()) return;
    for (const QUrl& url : catalog_.urls()) output_->preload(url);
}

void SfxPlayer::setEnabled(bool enabled) {
    if (enabled == enabled_) return;
    enabled_ = enabled;
    if (!enabled_ && output_ != nullptr) output_->stopAll();
}

void SfxPlayer::setVolumePercent(int percent) { volumePercent_ = std::clamp(percent, 0, 100); }

void SfxPlayer::setActive(bool active) {
    if (active == active_) return;
    active_ = active;
    if (!active_ && output_ != nullptr) output_->stopAll();
}

void SfxPlayer::play(const QString& cueId) {
    if (!enabled_ || !active_ || output_ == nullptr || volumePercent_ == 0) return;
    const std::optional<QUrl> url = catalog_.resolve(cueId);
    if (!url) {
        reportOnce(QStringLiteral("unknown:") + cueId, tr("Sound cue %1 has no sound").arg(cueId));
        return;
    }
    if (!output_->available()) {
        reportOnce(QStringLiteral("no-device"), tr("No audio output device was found: sound effects are silent"));
        return;
    }
    output_->play(*url, volumePercent_ / 100.0);
}

void SfxPlayer::reportOnce(const QString& key, const QString& message) {
    if (reported_.contains(key)) return;
    reported_.insert(key);
    emit problem(message);
}

}  // namespace wpiemu::app::runview
