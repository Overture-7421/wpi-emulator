#pragma once

// One device slot in the inspector: status dot, label + type, a device picker (compatible devices
// enabled, incompatible greyed with the reason) and a clear button. The whole row is a drop target
// for devices dragged from the "Detected devices" list.

#include <QComboBox>
#include <QFrame>
#include <QLabel>
#include <QToolButton>

#include "model/device_directory.h"
#include "model/slot_status.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class SlotRowWidget : public QFrame {
    Q_OBJECT
public:
    SlotRowWidget(const IconFactory& icons, QWidget* parent = nullptr);

    // Updates the row in place; the picker is only repopulated when its contents changed.
    void setInfo(const SlotInfo& info, const DeviceDirectory& devices);
    const sdk::SlotAddress& address() const { return info_.address; }
    void flash();
    QComboBox* picker() const { return picker_; }

Q_SIGNALS:
    void deviceChosen(const wpiemu::sdk::SlotAddress& slot, const wpiemu::sdk::DeviceKey& device, quint32 capabilityBits);
    void clearRequested(const wpiemu::sdk::SlotAddress& slot);

protected:
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dragLeaveEvent(QDragLeaveEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private:
    void setDropHighlight(int state);   // 0 none, 1 accept, 2 reject
    void populate(const DeviceDirectory& devices);

    const IconFactory& icons_;
    SlotInfo info_;
    QLabel* dot_;
    QLabel* title_;
    QLabel* detail_;
    QComboBox* picker_;
    QToolButton* clear_;
    QString signature_;
    bool populating_ = false;
};

}  // namespace wpiemu::app::editor
