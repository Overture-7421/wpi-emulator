#pragma once

// Text plumbing every Qt layer of the app repeats: SDK string conversion, translation contexts and clock stamps.
// Qt Core only.

#include <QCoreApplication>
#include <QString>

#include <string>

namespace wpiemu::app::ui {

// SDK names, labels and messages are UTF-8 std::string.
inline QString qs(const std::string& s) { return QString::fromStdString(s); }

// A translation context bound once, for code outside a QObject: `constexpr ui::Tr tr{"Hud"};` then tr("text").
// Context names key the translation files; never rename one.
struct Tr {
    const char* context;
    QString operator()(const char* text) const { return QCoreApplication::translate(context, text); }
};

// Local wall-clock time of a Unix time in milliseconds: "HH:mm:ss" and "HH:mm:ss.zzz".
QString clockText(qint64 msSinceEpoch);
QString clockTextMs(qint64 msSinceEpoch);

}  // namespace wpiemu::app::ui
