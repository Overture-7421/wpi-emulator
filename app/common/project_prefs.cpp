#include "common/project_prefs.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>

#include <algorithm>

namespace wpiemu::app {

QString ProjectPrefsStore::keyFor(const QString& projectPath, const char* field) {
    QString p = QDir::cleanPath(QFileInfo(projectPath).absoluteFilePath());
#ifdef Q_OS_WIN
    p = p.toLower();
#endif
    const QString hash = QString::fromLatin1(QCryptographicHash::hash(p.toUtf8(), QCryptographicHash::Sha1).toHex().left(10));
    return QStringLiteral("projectPrefs/%1/%2").arg(hash, QLatin1String(field));
}

ProjectPrefs ProjectPrefsStore::load(const QString& projectPath) const {
    ProjectPrefs prefs;
    prefs.alliance = store_.value(keyFor(projectPath, "alliance")).toString() == QLatin1String(sdk::allianceKey(sdk::Alliance::Blue))
                         ? sdk::Alliance::Blue
                         : sdk::Alliance::Red;
    prefs.station = std::clamp(store_.value(keyFor(projectPath, "station"), 2).toInt(), 1, 3);
    return prefs;
}

void ProjectPrefsStore::save(const QString& projectPath, const ProjectPrefs& prefs) {
    store_.setValue(keyFor(projectPath, "alliance"), QString::fromLatin1(sdk::allianceKey(prefs.alliance)));
    store_.setValue(keyFor(projectPath, "station"), std::clamp(prefs.station, 1, 3));
    store_.sync();
}

}  // namespace wpiemu::app
