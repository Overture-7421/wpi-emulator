#pragma once

// Table of detected devices: type / id / bus / status. Rows are drag sources (device mime) for the
// tree and slot rows. Newly announced devices are flagged for a few seconds so the view can
// highlight them.

#include <QAbstractTableModel>
#include <QSet>
#include <QString>

#include <vector>

#include "model/device_directory.h"
#include "model/editor_document.h"
#include "model/slot_status.h"

namespace wpiemu::app::editor {

class DetectedDevicesModel : public QAbstractTableModel {
    Q_OBJECT
public:
    enum Column { ColDevice, ColType, ColId, ColBus, ColStatus, ColumnCount };
    enum Role {
        PhysicalKeyRole = Qt::UserRole + 1,   // QString, PhysicalDeviceId::toString()
        MappedRole,                           // bool
        LiveRole,                             // bool
        AliasRole,                            // int sdk::AliasState
        RecentRole,                           // bool: announced in the last few seconds
        SortRole,
    };

    DetectedDevicesModel(const DeviceDirectory& devices, EditorDocument& document, QObject* parent = nullptr);

    void refresh();
    void markRecent(const sdk::DiscoveredDevice& device);
    const sdk::DiscoveredDevice* deviceAt(const QModelIndex& index) const;
    QModelIndex indexOf(const std::string& physicalKey) const;
    int unmappedLiveCount() const;

    int rowCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : static_cast<int>(rows_.size()); }
    int columnCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : ColumnCount; }
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QStringList mimeTypes() const override { return {QString::fromLatin1(kDeviceMime)}; }
    QMimeData* mimeData(const QModelIndexList& indexes) const override;

private:
    struct Row {
        sdk::DiscoveredDevice device;
        DeviceStatus status;
    };
    EditorDocument& document_;
    const DeviceDirectory& devices_;
    std::vector<Row> rows_;
    QSet<QString> recent_;
};

}  // namespace wpiemu::app::editor
