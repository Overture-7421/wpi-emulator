#pragma once

#include <QColor>
#include <QHash>
#include <QIcon>
#include <QString>

namespace wpiemu::app {

// Renders the bundled monochrome SVG icons (stroke = currentColor) in any colour, at any scale.
// Instances are owned by whoever needs icons (no global cache); copies of QIcon are cheap.
class IconProvider {
public:
    // `name` is the file stem under :/wpiemu/icons/, e.g. "check-circle".
    QIcon icon(const QString& name, const QColor& color) const;
    // Full-colour icons (the application logo): rendered as stored.
    QIcon logo() const;

    // Raw SVG text with currentColor replaced, empty when the icon does not exist (exposed for tests).
    static QString coloredSvg(const QString& name, const QColor& color);

private:
    mutable QHash<QString, QIcon> cache_;
};

}  // namespace wpiemu::app
