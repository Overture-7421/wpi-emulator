#pragma once

#include <QColor>
#include <QObject>
#include <QPalette>
#include <QString>

#include "common/app_settings.h"

class QApplication;

namespace wpiemu::app {

// Semantic colours of one theme. Widgets read these (never hard-coded colours) so light and dark
// stay consistent; the style sheet and the palette are both generated from them.
struct ThemeTokens {
    bool dark = false;
    QColor window, base, altBase, card, text, mutedText, border, hover;
    QColor accent, accentHover, accentText;
    QColor success, warning, error;
};

ThemeTokens lightTokens();
ThemeTokens darkTokens();
QPalette makePalette(const ThemeTokens& tokens);
QString buildStyleSheet(const ThemeTokens& tokens);

// Fusion style + generated palette and style sheet. Follows the OS colour scheme unless the user
// forces light or dark in Settings.
class ThemeManager : public QObject {
    Q_OBJECT
public:
    ThemeManager(QApplication& app, AppSettings& settings, QObject* parent = nullptr);

    const ThemeTokens& tokens() const { return tokens_; }
    bool isDark() const { return tokens_.dark; }

signals:
    void themeChanged();

private:
    void apply();
    bool resolveDark() const;

    QApplication& app_;
    AppSettings& settings_;
    ThemeTokens tokens_;
    ThemeMode appliedMode_ = ThemeMode::System;
};

}  // namespace wpiemu::app
