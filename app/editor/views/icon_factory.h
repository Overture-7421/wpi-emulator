#pragma once

// Small painted icons (no image assets): status dots for slots/devices and glyph tiles for tree
// nodes and palette templates. One factory per page; icons are cached per key.

#include <QColor>
#include <QHash>
#include <QIcon>
#include <QString>

#include "viewmodel/project_tree_model.h"

namespace wpiemu::app::editor {

class IconFactory {
public:
    QIcon status(SlotState state) const;
    QIcon deviceStatus(bool live, bool mapped, int aliasState) const;
    QIcon node(NodeKind kind) const;
    QIcon glyph(const QString& text, const QColor& color) const;
    QIcon severity(int severity) const;   // sdk::Severity
    static QColor colorForCategory(const QString& category);
    static QColor stateColor(SlotState state);

private:
    QIcon dot(const QColor& color, bool hollow) const;
    mutable QHash<QString, QIcon> cache_;
};

}  // namespace wpiemu::app::editor
