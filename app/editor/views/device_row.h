#pragma once

// One mapped (or empty) device slot of a part as a compact row: status dot, slot label, the mapped
// controller (or what an empty slot does), a Reversed check box for motors and an unmap button.
// Devices are never picked here: they are dragged onto the project tree.

#include <QCheckBox>
#include <QFrame>
#include <QLabel>
#include <QToolButton>

#include "model/slot_status.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class DeviceRowWidget : public QFrame {
    Q_OBJECT
public:
    DeviceRowWidget(const IconFactory& icons, QWidget* parent = nullptr);

    void setInfo(const SlotInfo& info);
    const sdk::SlotAddress& address() const { return info_.address; }
    const SlotInfo& info() const { return info_; }
    QCheckBox* reversedBox() const { return reversed_; }
    QToolButton* unmapButton() const { return clear_; }
    QString deviceText() const { return device_->text(); }
    QString titleText() const { return title_->text(); }
    // Shown instead of the slot's own label (it survives setInfo refreshes).
    void setLabel(const QString& label) { label_ = label; title_->setText(label_.isEmpty() ? slotLabel() : label_); }
    void flash();

Q_SIGNALS:
    void clearRequested(const wpiemu::sdk::SlotAddress& slot);
    void reversedToggled(const wpiemu::sdk::SlotAddress& slot, bool reversed);

private:
    const IconFactory& icons_;
    QString slotLabel() const;
    SlotInfo info_;
    QString label_;
    QLabel* dot_;
    QLabel* title_;
    QLabel* device_;
    QCheckBox* reversed_;
    QToolButton* clear_;
};

}  // namespace wpiemu::app::editor
