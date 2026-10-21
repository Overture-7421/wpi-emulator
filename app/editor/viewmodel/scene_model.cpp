#include "viewmodel/scene_model.h"

#include <cmath>

#include "ui/scene_support.h"

namespace wpiemu::app::editor {

void SceneModel::setContent(const SceneContent& content, const QColor& accent) {
    accent_ = accent;
    bool sameIds = content.items.size() == items_.size();
    for (std::size_t i = 0; sameIds && i < items_.size(); ++i) sameIds = items_[i].id == content.items[i].id;
    if (sameIds) {
        items_ = content.items;
        if (!items_.empty()) Q_EMIT dataChanged(index(0), index(static_cast<int>(items_.size()) - 1));
        return;
    }
    beginResetModel();
    items_ = content.items;
    endResetModel();
}

void SceneModel::setHighlight(const SelectionRef& target, const QColor& accent) {
    highlight_ = target;
    accent_ = accent;
    if (!items_.empty()) Q_EMIT dataChanged(index(0), index(static_cast<int>(items_.size()) - 1), {ColorRole, SelectedRole});
}

const SceneItem* SceneModel::itemById(const std::string& id) const {
    for (const auto& i : items_)
        if (i.id == id) return &i;
    return nullptr;
}

bool SceneModel::isHighlighted(const SceneItem& item) const {
    if (highlight_.isNone()) return false;
    if (item.ref == highlight_) return true;
    return !item.group.isNone() && item.group == highlight_;
}

QColor SceneModel::shade(const SceneItem& item) const {
    if (!isHighlighted(item)) return item.color;
    const double k = item.role == ItemRole::Overlay ? 0.5 : 0.42;
    return QColor::fromRgbF(item.color.redF() * (1 - k) + accent_.redF() * k, item.color.greenF() * (1 - k) + accent_.greenF() * k,
                            item.color.blueF() * (1 - k) + accent_.blueF() * k);
}

QHash<int, QByteArray> SceneModel::roleNames() const {
    return {{IdRole, "itemId"},   {SourceRole, "source3d"}, {PosRole, "pos"},          {RotRole, "rot"},
            {SclRole, "scl"},     {ColorRole, "tint"},      {OpacityRole, "alpha"},    {PickableRole, "isPickable"},
            {SelectedRole, "isSelected"}};
}

QVariant SceneModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() >= static_cast<int>(items_.size())) return {};
    const SceneItem& it = items_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case IdRole: return QString::fromStdString(it.id);
        case SourceRole: return it.shape == sdk::PrimitiveKind::Box ? QStringLiteral("#Cube") : QStringLiteral("#Cylinder");
        case PosRole: return ui::toQt(it.world.position);
        case RotRole: return ui::meshRotation(it.shape, it.world.rotation);
        case SclRole: return ui::meshScale(it.shape, it.size);
        case ColorRole: return shade(it);
        case OpacityRole: return static_cast<double>(it.opacity);
        case PickableRole: return it.pickable;
        case SelectedRole: return isHighlighted(it);
        default: return {};
    }
}

}  // namespace wpiemu::app::editor
