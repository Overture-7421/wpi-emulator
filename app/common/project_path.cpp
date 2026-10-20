#include "common/project_path.h"

#include <QDir>
#include <QFileInfo>

namespace wpiemu::app {

namespace {
#ifdef Q_OS_WIN
constexpr Qt::CaseSensitivity kPathCase = Qt::CaseInsensitive;
#else
constexpr Qt::CaseSensitivity kPathCase = Qt::CaseSensitive;
#endif
}  // namespace

QString normalizedProjectPath(const QString& path) { return QDir::cleanPath(QFileInfo(path).absoluteFilePath()); }

bool sameProjectPath(const QString& a, const QString& b) { return a.compare(b, kPathCase) == 0; }

QString projectPathKeyText(const QString& path) {
    const QString normalized = normalizedProjectPath(path);
    return kPathCase == Qt::CaseInsensitive ? normalized.toLower() : normalized;
}

}  // namespace wpiemu::app
