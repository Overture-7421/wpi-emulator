#include <QAudioDevice>
#include <QGuiApplication>
#include <QMediaDevices>
#include <QSoundEffect>

#include <map>
#include <memory>

#include "sfx_output.h"

namespace wpiemu::app::runview {
namespace {

// One QSoundEffect per sound file, created on first use. Audio devices are probed once, so a machine without one never
// constructs an effect. The offscreen platform is the headless one (tests, CI): it has nothing to say and plays nothing.
class QtSfxOutput final : public ISfxOutput {
public:
    bool available() const override {
        if (headless()) return true;
        if (!probed_) {
            probed_ = true;
            hasDevice_ = !QMediaDevices::audioOutputs().isEmpty();
        }
        return hasDevice_;
    }

    void preload(const QUrl& url) override {
        if (!headless() && available()) (void)effect(url);
    }

    void play(const QUrl& url, double volume) override {
        if (headless() || !available()) return;
        QSoundEffect* e = effect(url);
        e->setVolume(static_cast<float>(volume));
        if (e->isPlaying()) e->stop();
        e->play();
    }

    void stopAll() override {
        for (auto& [url, e] : effects_) e->stop();
    }

private:
    static bool headless() { return QGuiApplication::platformName() == QLatin1String("offscreen"); }
    QSoundEffect* effect(const QUrl& url) {
        auto it = effects_.find(url);
        if (it == effects_.end()) {
            auto e = std::make_unique<QSoundEffect>();
            e->setSource(url);
            it = effects_.emplace(url, std::move(e)).first;
        }
        return it->second.get();
    }

    mutable bool probed_ = false;
    mutable bool hasDevice_ = false;
    std::map<QUrl, std::unique_ptr<QSoundEffect>> effects_;
};

}  // namespace

std::unique_ptr<ISfxOutput> createDefaultSfxOutput() { return std::make_unique<QtSfxOutput>(); }

}  // namespace wpiemu::app::runview
