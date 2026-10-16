#include "viewmodel/detected_devices_model.h"

#include <QMimeData>
#include <QTimer>

#include <algorithm>
#include <tuple>

#include "model/config_ops.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

DetectedDevicesModel::DetectedDevicesModel(const DeviceDirectory& devices, EditorDocument& document, QObject* parent)
    : QAbstractTableModel(parent), document_(document), devices_(devices) {
    refresh();
}

void DetectedDevicesModel::refresh() {
    beginResetModel();
    rows_.clear();
    for (const auto& d : devices_.all()) rows_.push_back({d, deviceStatus(document_.config(), d)});
    std::stable_sort(rows_.begin(), rows_.end(), [](const Row& a, const Row& b) {
        const auto& x = a.device.descriptor;
        const auto& y = b.device.descriptor;
        return std::tie(x.family, x.type, x.id, x.channel) < std::tie(y.family, y.type, y.id, y.channel);
    });
    endResetModel();
}

void DetectedDevicesModel::markRecent(const sdk::DiscoveredDevice& device) {
    const QString key = QString::fromStdString(device.descriptor.physicalId().toString());
    recent_.insert(key);
    refresh();
    QTimer::singleShot(4500, this, [this, key] {
        recent_.remove(key);
        if (!rows_.empty()) Q_EMIT dataChanged(index(0, 0), index(rowCount() - 1, ColumnCount - 1));
    });
}

const sdk::DiscoveredDevice* DetectedDevicesModel::deviceAt(const QModelIndex& index) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= static_cast<int>(rows_.size())) return nullptr;
    return &rows_[static_cast<std::size_t>(index.row())].device;
}

QModelIndex DetectedDevicesModel::indexOf(const std::string& physicalKey) const {
    for (std::size_t i = 0; i < rows_.size(); ++i)
        if (rows_[i].device.descriptor.physicalId().toString() == physicalKey) return index(static_cast<int>(i), 0);
    return {};
}

int DetectedDevicesModel::unmappedLiveCount() const {
    return static_cast<int>(std::count_if(rows_.begin(), rows_.end(), [](const Row& r) { return r.device.live && !r.status.mapped(); }));
}

QVariant DetectedDevicesModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return {};
    switch (section) {
        case ColDevice: return ETR("Device");
        case ColType: return ETR("Type");
        case ColId: return ETR("ID");
        case ColBus: return ETR("Bus");
        case ColStatus: return ETR("Status");
        default: return {};
    }
}

QVariant DetectedDevicesModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() >= static_cast<int>(rows_.size())) return {};
    const Row& row = rows_[static_cast<std::size_t>(index.row())];
    const sdk::DeviceDescriptor& d = row.device.descriptor;
    switch (role) {
        case PhysicalKeyRole: return QString::fromStdString(d.physicalId().toString());
        case MappedRole: return row.status.mapped();
        case LiveRole: return row.device.live;
        case AliasRole: return static_cast<int>(row.status.alias);
        case RecentRole: return recent_.contains(QString::fromStdString(d.physicalId().toString()));
        case Qt::ToolTipRole: {
            QString tip = QString::fromStdString(d.family + "/" + d.type) + (d.vendor.empty() ? QString() : QStringLiteral(" (%1)").arg(QString::fromStdString(d.vendor)));
            if (!d.nativeName.empty()) tip += QStringLiteral("\n") + QString::fromStdString(d.nativeName);
            if (row.status.alias != sdk::AliasState::None) {
                tip += row.status.alias == sdk::AliasState::Conflict
                           ? ETR("\nAlias conflict: the same physical device is addressed through different buses and mapped independently.")
                           : ETR("\nAliased: the robot code addresses this device through several buses. Harmless while one slot uses it.");
                for (const auto& a : row.device.aliases) tip += QStringLiteral("\n  \xE2\x80\xA2 %1").arg(QString::fromStdString(a.toString()));
            }
            tip += ETR("\nDrag onto a mechanism slot to map it.");
            return tip;
        }
        case Qt::DisplayRole:
            switch (index.column()) {
                case ColDevice: return deviceLabel(d);
                case ColType: return QString::fromStdString(d.type);
                case ColId: return d.id >= 0 ? QVariant(d.id) : QVariant(QStringLiteral("\xE2\x80\x94"));
                case ColBus: return busLabel(row.device.observedBus.empty() ? d.bus : row.device.observedBus);
                case ColStatus: {
                    QString s;
                    if (!row.device.live) s = ETR("gone");
                    else if (row.status.mapped()) {
                        const auto& first = row.status.usedBy.front();
                        s = QStringLiteral("\xE2\x86\x92 %1").arg(QString::fromStdString(ops::slotDisplayName(document_.config(), document_.catalog(), first)));
                        if (row.status.usedBy.size() > 1) s += QStringLiteral(" +%1").arg(row.status.usedBy.size() - 1);
                    } else s = ETR("unmapped (loopback)");
                    if (row.status.alias == sdk::AliasState::Conflict) s = ETR("\xE2\x9A\xA0 alias conflict \xC2\xB7 ") + s;
                    else if (row.status.alias == sdk::AliasState::Aliased) s += ETR(" \xC2\xB7 aliased");
                    return s;
                }
                default: return {};
            }
        default: return {};
    }
}

Qt::ItemFlags DetectedDevicesModel::flags(const QModelIndex& index) const {
    Qt::ItemFlags f = QAbstractTableModel::flags(index);
    if (index.isValid()) f |= Qt::ItemIsDragEnabled;
    return f;
}

QMimeData* DetectedDevicesModel::mimeData(const QModelIndexList& indexes) const {
    for (const QModelIndex& i : indexes)
        if (const auto* d = deviceAt(i)) {
            auto* mime = new QMimeData;
            mime->setData(QString::fromLatin1(kDeviceMime), encodeDevice(*d));
            mime->setText(deviceLabel(d->descriptor));
            return mime;
        }
    return nullptr;
}

}  // namespace wpiemu::app::editor
