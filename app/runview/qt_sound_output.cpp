#include <QAudioDevice>
#include <QMediaDevices>
#include <QSoundEffect>
#include <QString>
#include <QUrl>

#include <map>
#include <memory>
#include <set>

#include "sound_output.h"

namespace wpiemu::app::runview {
namespace {

// Plays each cue through its own QSoundEffect (uncompressed WAV from the embedded plugin resources), so cues overlap.
class QtSoundOutput final : public ISoundOutput {
public:
    void setCatalog(const std::vector<sdk::SoundCueInfo>& cues) override {
        effects_.clear();
        reported_.clear();
        if (cues.empty()) return;
        // Without an output device Qt would log a critical message per cue: report once and stay silent instead.
        if (QMediaDevices::audioOutputs().isEmpty()) {
            reportOnce("no audio output device", "No audio output device: match sounds are off.");
            return;
        }
        for (const sdk::SoundCueInfo& cue : cues) {
            auto effect = std::make_unique<QSoundEffect>();
            QSoundEffect* raw = effect.get();
            raw->setVolume(static_cast<qreal>(cue.gain));
            QObject::connect(raw, &QSoundEffect::statusChanged, raw, [this, raw, id = cue.id] {
                if (raw->status() == QSoundEffect::Error)
                    reportOnce(id, "Could not load the sound for " + id + ".");
            });
            raw->setSource(QUrl(QStringLiteral("qrc:/wpiemu/plugins/") + QString::fromStdString(cue.asset)));
            effects_[cue.id] = std::move(effect);
        }
    }

    void play(const std::string& cueId) override {
        const auto it = effects_.find(cueId);
        if (it == effects_.end()) return;
        it->second->stop();
        it->second->play();
    }

    void stopAll() override {
        for (auto& [id, effect] : effects_) effect->stop();
    }

private:
    void reportOnce(const std::string& key, const std::string& message) {
        if (reported_.insert(key).second) reportProblem(message);
    }

    std::map<std::string, std::unique_ptr<QSoundEffect>> effects_;
    std::set<std::string> reported_;
};

}  // namespace

std::unique_ptr<ISoundOutput> createDefaultSoundOutput() { return std::make_unique<QtSoundOutput>(); }

}  // namespace wpiemu::app::runview
