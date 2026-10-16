#include "mechanism_overlay_model.h"

#include <cmath>

#include "scene_parts.h"

namespace wpiemu::app::runview {

MechanismOverlayModel::MechanismOverlayModel(QObject* parent) : QAbstractListModel(parent) {}

QString MechanismOverlayModel::formatValue(const QString& name, double value, const QString& unit) {
    const double a = std::fabs(value);
    const int decimals = a >= 1000.0 ? 0 : (a >= 100.0 ? 1 : 2);
    QString text = QStringLiteral("%1  %2").arg(name, QString::number(value, 'f', decimals));
    if (!unit.isEmpty()) text += QLatin1Char(' ') + unit;
    return text;
}

void MechanismOverlayModel::update(const sdk::ISimSnapshot* snapshot) {
    std::vector<Row> next;
    if (snapshot != nullptr) {
        for (const auto& m : snapshot->mechanisms()) {
            Row r;
            r.id = QString::fromStdString(m.id);
            r.type = QString::fromStdString(m.type);
            r.label = r.id;
            if (config_)
                if (const sdk::MechanismConfig* mc = config_->findMechanism(m.id); mc != nullptr && !mc->label.empty())
                    r.label = QString::fromStdString(mc->label);
            r.active = m.active;
            QStringList lines;
            for (const auto& v : m.values) {
                const DisplayValue d = toDisplayValue(v);
                lines << formatValue(QString::fromStdString(d.name), d.value, QString::fromStdString(d.unit));
            }
            r.values = lines.join(QLatin1Char('\n'));
            next.push_back(std::move(r));
        }
    }
    if (next.size() != rows_.size()) {
        beginResetModel();
        rows_ = std::move(next);
        endResetModel();
        return;
    }
    for (std::size_t i = 0; i < next.size(); ++i) {
        if (next[i] == rows_[i]) continue;
        const bool sameId = next[i].id == rows_[i].id;
        rows_[i] = std::move(next[i]);
        const QModelIndex idx = index(static_cast<int>(i));
        if (sameId) emit dataChanged(idx, idx, {ActiveRole, ValuesRole, LabelRole});
        else emit dataChanged(idx, idx);
    }
}

int MechanismOverlayModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : static_cast<int>(rows_.size());
}

QVariant MechanismOverlayModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const Row& r = rows_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case IdRole: return r.id;
        case LabelRole: return r.label;
        case TypeRole: return r.type;
        case ActiveRole: return r.active;
        case ValuesRole: return r.values;
        default: return {};
    }
}

QHash<int, QByteArray> MechanismOverlayModel::roleNames() const {
    return {{IdRole, "mechId"}, {LabelRole, "label"}, {TypeRole, "mechType"}, {ActiveRole, "active"}, {ValuesRole, "values"}};
}

}  // namespace wpiemu::app::runview
