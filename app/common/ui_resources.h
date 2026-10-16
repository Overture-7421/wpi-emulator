#pragma once

#include <QColor>
#include <QIcon>
#include <QObject>
#include <QString>

#include "common/icon_provider.h"
#include "common/theme.h"

namespace wpiemu::app {

enum class IconTone { Text, Muted, Accent, OnAccent, Success, Warning, Error };

// What views use to look up theme colours and icons; forwards theme changes so they can refresh.
// Owned by the composition root and passed to views by reference.
class UiResources : public QObject {
    Q_OBJECT
public:
    explicit UiResources(ThemeManager& theme, QObject* parent = nullptr);

    const ThemeTokens& tokens() const { return theme_.tokens(); }
    QColor color(IconTone tone) const;
    QIcon icon(const QString& name, IconTone tone = IconTone::Text) const;
    QIcon logo() const { return icons_.logo(); }

signals:
    void themeChanged();

private:
    ThemeManager& theme_;
    IconProvider icons_;
};

}  // namespace wpiemu::app
