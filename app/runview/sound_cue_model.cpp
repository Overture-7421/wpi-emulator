#include "sound_cue_model.h"

#include <algorithm>

#include "ui/qt_text.h"

namespace wpiemu::app::runview {
namespace {

constexpr ui::Tr tr{"RunView"};
const QString kEnabledKey = QStringLiteral("runview/sound/enabled");

bool sameCues(const std::vector<sdk::SoundCueInfo>& a, const std::vector<sdk::SoundCueInfo>& b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [](const sdk::SoundCueInfo& x, const sdk::SoundCueInfo& y) {
        return x.id == y.id && x.asset == y.asset && x.gain == y.gain && x.interruptsOthers == y.interruptsOthers;
    });
}

}  // namespace

SoundCueModel::SoundCueModel(ISoundOutput* output, QSettings* settings, QObject* parent)
    : QObject(parent), output_(output), settings_(settings) {
    if (settings_ != nullptr) enabled_ = settings_->value(kEnabledKey, true).toBool();
    if (output_ != nullptr)
        output_->setProblemHandler([this](const std::string& message) { emit problem(QString::fromStdString(message)); });
}

void SoundCueModel::setEnabled(bool enabled) {
    if (enabled == enabled_) return;
    enabled_ = enabled;
    if (settings_ != nullptr) settings_->setValue(kEnabledKey, enabled_);
    if (!enabled_ && output_ != nullptr) output_->stopAll();
    emit enabledChanged(enabled_);
}

void SoundCueModel::setCatalog(std::vector<sdk::SoundCueInfo> cues) {
    if (sameCues(cues, catalogList_)) return;
    catalogList_ = std::move(cues);
    catalog_.clear();
    for (const sdk::SoundCueInfo& cue : catalogList_) catalog_[cue.id] = cue;
    unknownReported_.clear();
    if (output_ != nullptr) output_->setCatalog(catalogList_);
}

void SoundCueModel::suspend() {
    const bool wasPlaying = primed_;
    primed_ = false;
    if (wasPlaying && output_ != nullptr) output_->stopAll();
}

void SoundCueModel::onSnapshot(const sdk::ISimSnapshot* snapshot) {
    if (snapshot == nullptr) {
        suspend();
        return;
    }
    // Tick numbers and sim time restart with a Reset or a new robot session: nothing from before it is still playing.
    if (primed_ && (snapshot->tick() < lastTick_ || snapshot->simTimeUs() < lastSimUs_)) suspend();
    lastTick_ = snapshot->tick();
    lastSimUs_ = snapshot->simTimeUs();

    const auto cues = snapshot->soundCues();
    if (!primed_) {
        lastSequence_ = 0;
        for (const sdk::SoundCueEvent& cue : cues) lastSequence_ = std::max(lastSequence_, cue.sequence);
        primed_ = true;
        return;
    }
    for (const sdk::SoundCueEvent& event : cues) {
        if (event.sequence <= lastSequence_) continue;
        lastSequence_ = event.sequence;
        if (!enabled_ || output_ == nullptr) continue;
        if (snapshot->simTimeUs() - event.simTimeUs > kMaxCueAgeUs) continue;
        const auto known = catalog_.find(event.cueId);
        if (known == catalog_.end()) {
            if (unknownReported_.insert(event.cueId).second)
                emit problem(tr("The game raised the unknown sound \"%1\".").arg(QString::fromStdString(event.cueId)));
            continue;
        }
        if (known->second.interruptsOthers) output_->stopAll();
        output_->play(event.cueId);
    }
}

}  // namespace wpiemu::app::runview
