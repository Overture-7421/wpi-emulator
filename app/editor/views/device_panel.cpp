#include "views/device_panel.h"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QPainter>
#include <QVBoxLayout>

#include "model/tr.h"

namespace wpiemu::app::editor {

QSize DeviceRowDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const {
    QSize s = QStyledItemDelegate::sizeHint(option, index);
    if (index.column() == DetectedDevicesModel::ColDevice) s.setWidth(s.width() + 24);
    return s;
}

void DeviceRowDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const {
    QStyleOptionViewItem opt = option;
    initStyleOption(&opt, index);
    const bool recent = index.data(DetectedDevicesModel::RecentRole).toBool();
    if (recent && !(opt.state & QStyle::State_Selected)) painter->fillRect(opt.rect, QColor(88, 166, 255, 55));

    const int alias = index.data(DetectedDevicesModel::AliasRole).toInt();
    const bool mapped = index.data(DetectedDevicesModel::MappedRole).toBool();
    const bool live = index.data(DetectedDevicesModel::LiveRole).toBool();
    if (index.column() == DetectedDevicesModel::ColDevice) {
        opt.icon = icons_.deviceStatus(live, mapped, alias);
        opt.decorationSize = QSize(16, 16);
        opt.features |= QStyleOptionViewItem::HasDecoration;
    }
    if (index.column() == DetectedDevicesModel::ColStatus) {
        const QColor c = alias == 2 ? QColor("#f85149") : !mapped && live ? QColor("#d29922") : opt.palette.color(QPalette::Text);
        opt.palette.setColor(QPalette::Text, c);
        opt.palette.setColor(QPalette::HighlightedText, alias == 2 ? QColor("#ffb4af") : opt.palette.color(QPalette::HighlightedText));
        if (alias == 2) opt.font.setBold(true);
    }
    QStyledItemDelegate::paint(painter, opt, index);
}

DevicePanel::DevicePanel(EditorController& controller, DetectedDevicesModel& model, const IconFactory& icons, QWidget* parent)
    : QWidget(parent), controller_(controller), model_(model), title_(new QLabel(this)), detect_(new QPushButton(this)),
      status_(new QLabel(this)), stack_(new QStackedWidget(this)), table_(new QTableView(this)), empty_(new QLabel(this)),
      summary_(new QLabel(this)) {
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(4);

    auto* head = new QHBoxLayout;
    head->setContentsMargins(8, 6, 8, 0);
    QFont f = title_->font();
    f.setBold(true);
    title_->setFont(f);
    head->addWidget(title_, 1);
    detect_->setText(ETR("Detect devices"));
    detect_->setToolTip(ETR("Run the robot in discovery mode: devices appear here as your robot code creates them."));
    detect_->setCursor(Qt::PointingHandCursor);
    head->addWidget(detect_);
    v->addLayout(head);
    status_->setContentsMargins(8, 0, 8, 0);
    status_->setWordWrap(true);
    status_->setEnabled(false);
    v->addWidget(status_);

    table_->setModel(&model_);
    table_->setItemDelegate(new DeviceRowDelegate(icons, table_));
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setDragEnabled(true);
    table_->setDragDropMode(QAbstractItemView::DragOnly);
    table_->setDefaultDropAction(Qt::CopyAction);
    table_->setShowGrid(false);
    table_->setAlternatingRowColors(true);
    table_->verticalHeader()->hide();
    table_->verticalHeader()->setDefaultSectionSize(24);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setSectionResizeMode(DetectedDevicesModel::ColDevice, QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(DetectedDevicesModel::ColType, QHeaderView::Interactive);
    table_->setColumnHidden(DetectedDevicesModel::ColType, true);
    table_->setColumnHidden(DetectedDevicesModel::ColId, true);
    table_->setColumnWidth(DetectedDevicesModel::ColId, 36);
    table_->setColumnWidth(DetectedDevicesModel::ColBus, 70);
    table_->horizontalHeader()->setSectionResizeMode(DetectedDevicesModel::ColId, QHeaderView::Fixed);
    table_->horizontalHeader()->setSectionResizeMode(DetectedDevicesModel::ColBus, QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(DetectedDevicesModel::ColStatus, QHeaderView::Stretch);
    table_->setFrameShape(QFrame::NoFrame);

    empty_->setWordWrap(true);
    empty_->setAlignment(Qt::AlignCenter);
    empty_->setMargin(14);
    empty_->setEnabled(false);
    empty_->setText(ETR("No devices yet.\n\nClick \"Detect devices\", then run your robot code. Every motor controller, encoder and sensor it creates shows up here, ready to drag onto a mechanism slot."));
    stack_->addWidget(empty_);
    stack_->addWidget(table_);
    v->addWidget(stack_, 1);

    summary_->setContentsMargins(8, 0, 8, 6);
    v->addWidget(summary_);

    connect(detect_, &QPushButton::clicked, this, [this] { controller_.toggleDetection(); });
    connect(&controller_, &EditorController::detectionChanged, this, [this] { updateState(); });
    connect(&controller_, &EditorController::devicesUpdated, this, [this] {
        model_.refresh();
        updateState();
    });
    connect(&controller_.document(), &EditorDocument::changed, this, [this] {
        model_.refresh();
        updateState();
    });
    connect(&controller_, &EditorController::deviceToast, this, [this](const QString&, const QString&) {});
    connect(table_->selectionModel(), &QItemSelectionModel::currentRowChanged, this, [this](const QModelIndex& current) {
        if (syncing_) return;
        if (const auto* d = model_.deviceAt(current)) controller_.selection().set(SelectionRef::device(d->descriptor.physicalId().toString()));
    });
    connect(table_, &QTableView::doubleClicked, this, [this](const QModelIndex& index) {
        const auto* d = model_.deviceAt(index);
        if (d == nullptr) return;
        const QString err = controller_.mapDevice(controller_.selection().current(), proposedKey(*d), d->descriptor.capabilities);
        if (!err.isEmpty()) Q_EMIT controller_.statusMessage(err, 1);
    });
    connect(&controller_.selection(), &SelectionState::changed, this, [this](const SelectionRef& ref) {
        if (ref.kind == SelKind::Device) selectDevice(ref.owner);
    });
    updateState();
}

void DevicePanel::selectDevice(const std::string& physicalKey) {
    const QModelIndex idx = model_.indexOf(physicalKey);
    syncing_ = true;
    if (idx.isValid()) {
        table_->selectRow(idx.row());
        table_->scrollTo(idx);
    }
    syncing_ = false;
}

void DevicePanel::updateState() {
    const int total = model_.rowCount();
    const bool detecting = controller_.detecting();
    title_->setText(total > 0 ? ETR("Detected devices (%1)").arg(total) : ETR("Detected devices"));
    detect_->setText(detecting ? ETR("Stop detecting") : ETR("Detect devices"));
    detect_->setDefault(!detecting && total == 0);
    status_->setVisible(detecting);
    status_->setText(ETR("Listening... run your robot code. New devices are added as they announce themselves."));
    stack_->setCurrentWidget(total > 0 ? static_cast<QWidget*>(table_) : static_cast<QWidget*>(empty_));
    if (total == 0) {
        summary_->setText({});
        return;
    }
    const int unmapped = model_.unmappedLiveCount();
    if (unmapped > 0)
        summary_->setText(QStringLiteral("<span style=\"color:#d29922\">%1</span>")
                              .arg(QCoreApplication::translate("Editor", "%n unmapped device(s) run in loopback.", nullptr, unmapped)));
    else
        summary_->setText(ETR("All detected devices are mapped."));
}

}  // namespace wpiemu::app::editor
