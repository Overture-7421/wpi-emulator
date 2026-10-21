#pragma once

// List model feeding the Qt Quick 3D scene: one row per primitive with everything QML needs already
// converted (Z-up world, cylinder axis, colours, selection tint). QML never sees the robot config.

#include <QAbstractListModel>
#include <QColor>

#include <vector>

#include "model/scene_builder.h"

namespace wpiemu::app::editor {

class SceneModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role {
        IdRole = Qt::UserRole + 1,
        SourceRole,
        PosRole,
        RotRole,
        SclRole,
        ColorRole,
        OpacityRole,
        PickableRole,
        SelectedRole,
    };

    explicit SceneModel(QObject* parent = nullptr) : QAbstractListModel(parent) {}

    void setContent(const SceneContent& content, const QColor& accent);
    // `target` highlights items whose ref or group equals it.
    void setHighlight(const SelectionRef& target, const QColor& accent);
    const SceneItem* itemById(const std::string& id) const;
    const std::vector<SceneItem>& items() const { return items_; }
    bool isHighlighted(const SceneItem& item) const;

    int rowCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : static_cast<int>(items_.size()); }
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

private:
    QColor shade(const SceneItem& item) const;

    std::vector<SceneItem> items_;
    SelectionRef highlight_;
    QColor accent_{"#4da3ff"};
};

}  // namespace wpiemu::app::editor
