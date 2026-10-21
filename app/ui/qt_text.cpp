#include "ui/qt_text.h"

#include <QDateTime>

namespace wpiemu::app::ui {

QString clockText(qint64 msSinceEpoch) {
    return QDateTime::fromMSecsSinceEpoch(msSinceEpoch).toString(QStringLiteral("HH:mm:ss"));
}

QString clockTextMs(qint64 msSinceEpoch) {
    return QDateTime::fromMSecsSinceEpoch(msSinceEpoch).toString(QStringLiteral("HH:mm:ss.zzz"));
}

}  // namespace wpiemu::app::ui
