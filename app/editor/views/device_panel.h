#pragma once

// "Detected devices": the live list of what the robot code declared (type / id / bus / status), the
// Detect devices button and the empty-state coaching. Rows are drag sources for slots.

#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QStyledItemDelegate>
#include <QTableView>
#include <QWidget>

#include "viewmodel/detected_devices_model.h"
#include "viewmodel/editor_controller.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class DeviceRowDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    DeviceRowDelegate(const IconFactory& icons, QObject* parent = nullptr) : QStyledItemDelegate(parent), icons_(icons) {}
    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const override;

private:
    const IconFactory& icons_;
};

class DevicePanel : public QWidget {
    Q_OBJECT
public:
    DevicePanel(EditorController& controller, DetectedDevicesModel& model, const IconFactory& icons, QWidget* parent = nullptr);

    QPushButton* detectButton() const { return detect_; }
    QTableView* table() const { return table_; }
    void selectDevice(const std::string& physicalKey);

private:
    void updateState();

    EditorController& controller_;
    DetectedDevicesModel& model_;
    QLabel* title_;
    QPushButton* detect_;
    QLabel* status_;
    QStackedWidget* stack_;
    QTableView* table_;
    QLabel* empty_;
    QLabel* summary_;
    bool syncing_ = false;
};

}  // namespace wpiemu::app::editor
