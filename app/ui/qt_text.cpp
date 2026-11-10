#include "ui/qt_text.h"

#include <QDateTime>

namespace wpiemu::app::ui {

QString clockText(qint64 msSinceEpoch) {
    return QDateTime::fromMSecsSinceEpoch(msSinceEpoch).toString(QStringLiteral("HH:mm:ss"));
}

QString clockTextMs(qint64 msSinceEpoch) {
    return QDateTime::fromMSecsSinceEpoch(msSinceEpoch).toString(QStringLiteral("HH:mm:ss.zzz"));
}

QString stationText(sdk::proto::AllianceStation station) {
    constexpr Tr tr{"Station"};
    const int number = sdk::proto::stationNumberOf(station);
    return (sdk::proto::allianceOf(station) == sdk::Alliance::Red ? tr("Red %1") : tr("Blue %1")).arg(number);
}

}  // namespace wpiemu::app::ui
