#include "sfx_catalog.h"

#include <QFile>

#include <algorithm>

#include "ui/qt_text.h"

namespace wpiemu::app::runview {
namespace {
constexpr ui::Tr tr{"Sound"};

QString resourcePath(const QUrl& url) { return QLatin1Char(':') + url.path(); }
}  // namespace

QUrl commonSoundUrl(const QString& asset) {
    return QUrl(QStringLiteral("qrc:/wpiemu/runview/assets/sfx/%1.wav").arg(asset));
}

QUrl pluginSoundUrl(const QString& gameId, const QString& asset) {
    return QUrl(QStringLiteral("qrc:/wpiemu/plugins/%1/sfx/%2.wav").arg(gameId, asset));
}

QStringList SfxCatalog::set(const QString& gameId, const std::vector<sdk::SoundCueInfo>& gameCues) {
    QStringList problems;
    cues_.clear();
    for (const sdk::SoundCueInfo& cue : sdk::commonSoundCues()) {
        const QUrl url = commonSoundUrl(QString::fromStdString(cue.asset));
        if (!QFile::exists(resourcePath(url)))
            problems << tr("Sound cue %1: the file %2.wav is not embedded in the app").arg(QString::fromStdString(cue.id), QString::fromStdString(cue.asset));
        else
            cues_[QString::fromStdString(cue.id)] = url;
    }
    const QString prefix = gameId + QLatin1Char('.');
    for (const sdk::SoundCueInfo& cue : gameCues) {
        const QString id = QString::fromStdString(cue.id);
        if (gameId.isEmpty() || !id.startsWith(prefix)) {
            problems << tr("Sound cue %1 is ignored: a game's cue ids must start with \"%2\"").arg(id, prefix);
            continue;
        }
        const QUrl url = pluginSoundUrl(gameId, QString::fromStdString(cue.asset));
        if (!QFile::exists(resourcePath(url)))
            problems << tr("Sound cue %1: the file %2.wav is not shipped with game %3").arg(id, QString::fromStdString(cue.asset), gameId);
        else
            cues_[id] = url;
    }
    return problems;
}

std::optional<QUrl> SfxCatalog::resolve(const QString& cueId) const {
    const auto it = cues_.find(cueId);
    if (it == cues_.end()) return std::nullopt;
    return it->second;
}

std::vector<QUrl> SfxCatalog::urls() const {
    std::vector<QUrl> out;
    for (const auto& [id, url] : cues_)
        if (std::find(out.begin(), out.end(), url) == out.end()) out.push_back(url);
    return out;
}

}  // namespace wpiemu::app::runview
