#include "views/slot_row.h"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QHBoxLayout>
#include <QStandardItemModel>
#include <QTimer>
#include <QVBoxLayout>

#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {
QString qs(const std::string& s) { return QString::fromStdString(s); }
}  // namespace

SlotRowWidget::SlotRowWidget(const IconFactory& icons, QWidget* parent)
    : QFrame(parent), icons_(icons), dot_(new QLabel(this)), title_(new QLabel(this)), detail_(new QLabel(this)), picker_(new QComboBox(this)),
      clear_(new QToolButton(this)) {
    setAcceptDrops(true);
    setFrameShape(QFrame::NoFrame);
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(6, 4, 6, 4);
    v->setSpacing(2);
    auto* top = new QHBoxLayout;
    top->setSpacing(6);
    dot_->setFixedSize(16, 16);
    top->addWidget(dot_);
    top->addWidget(title_, 1);
    v->addLayout(top);
    auto* bottom = new QHBoxLayout;
    bottom->setSpacing(4);
    bottom->setContentsMargins(22, 0, 0, 0);
    picker_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    picker_->setMinimumContentsLength(14);
    picker_->setFocusPolicy(Qt::StrongFocus);
    bottom->addWidget(picker_, 1);
    clear_->setText(QStringLiteral("\xC3\x97"));
    clear_->setToolTip(ETR("Unmap this slot"));
    clear_->setAutoRaise(true);
    bottom->addWidget(clear_);
    v->addLayout(bottom);
    detail_->setWordWrap(true);
    detail_->setContentsMargins(22, 0, 0, 0);
    QFont f = detail_->font();
    f.setPointSizeF(f.pointSizeF() * 0.9);
    detail_->setFont(f);
    v->addWidget(detail_);

    connect(picker_, &QComboBox::activated, this, [this](int index) {
        if (populating_) return;
        const QString data = picker_->itemData(index).toString();
        if (data.isEmpty()) {
            Q_EMIT clearRequested(info_.address);
            return;
        }
        if (const auto payload = decodeDevice(data.toUtf8())) Q_EMIT deviceChosen(info_.address, payload->key, payload->capabilities.bits);
    });
    connect(clear_, &QToolButton::clicked, this, [this] { Q_EMIT clearRequested(info_.address); });
}

void SlotRowWidget::populate(const DeviceDirectory& devices) {
    populating_ = true;
    const QSignalBlocker block(picker_);
    picker_->clear();
    picker_->addItem(info_.spec.required ? ETR("Not mapped (pick a device)") : ETR("Not mapped (optional)"), QString());
    auto* model = qobject_cast<QStandardItemModel*>(picker_->model());
    int current = 0;
    bool mappedListed = false;
    for (const sdk::DiscoveredDevice& d : devices.all()) {
        DevicePayload payload{proposedKey(d), d.descriptor.capabilities};
        const bool ok = slotAccepts(info_.spec, d.descriptor.capabilities);
        QString text = deviceLabel(d.descriptor);
        const QString bus = d.observedBus.empty() ? (d.descriptor.bus.empty() ? QString() : qs(d.descriptor.bus)) : qs(d.observedBus);
        if (!bus.isEmpty()) text += QStringLiteral(" \xC2\xB7 ") + bus;
        if (!ok) text += ETR("  (not a %1)").arg(slotTypeLabel(info_.spec.type).toLower());
        if (!d.live) text += ETR("  (gone)");
        picker_->addItem(text, QString::fromUtf8(encodePayload(payload)));
        const int idx = picker_->count() - 1;
        if (model != nullptr && model->item(idx) != nullptr && !ok) model->item(idx)->setEnabled(false);
        if (info_.mapped && info_.mapped->physical() == d.descriptor.physicalId()) {
            current = idx;
            mappedListed = true;
        }
    }
    if (info_.mapped && !mappedListed) {
        DevicePayload payload{*info_.mapped, sdk::requiredCapabilities(info_.spec.type)};
        picker_->addItem(deviceLabel(*info_.mapped) + ETR("  (not detected)"), QString::fromUtf8(encodePayload(payload)));
        current = picker_->count() - 1;
    }
    picker_->setCurrentIndex(current);
    populating_ = false;
}

void SlotRowWidget::setInfo(const SlotInfo& info, const DeviceDirectory& devices) {
    info_ = info;
    dot_->setPixmap(icons_.status(info.state).pixmap(16, 16));
    title_->setText(QStringLiteral("<b>%1</b>  <span style=\"color:gray\">%2</span>")
                        .arg(qs(info.spec.label.empty() ? info.spec.name : info.spec.label).toHtmlEscaped(),
                             (slotTypeLabel(info.spec.type) + (info.spec.required ? QString() : ETR(" \xC2\xB7 optional"))).toHtmlEscaped()));
    title_->setToolTip(qs(info.spec.description));
    clear_->setVisible(info.mapped.has_value());

    QStringList sig;
    sig << QString::number(static_cast<int>(info.spec.type)) << (info.mapped ? deviceLabel(*info.mapped) : QString());
    for (const auto& d : devices.all()) sig << QString::fromStdString(d.descriptor.physicalId().toString()) << (d.live ? "1" : "0");
    const QString signature = sig.join('|');
    if (signature != signature_) {
        signature_ = signature;
        populate(devices);
    }

    const bool attention = info.state == SlotState::Unmapped || info.state == SlotState::Incompatible ||
                           info.state == SlotState::AliasConflict || info.state == SlotState::NotDetected;
    detail_->setVisible(attention);
    detail_->setText(info.detail);
    detail_->setStyleSheet(QStringLiteral("color: %1;").arg(IconFactory::stateColor(info.state).name()));
}

void SlotRowWidget::flash() {
    setStyleSheet(QStringLiteral("SlotRowWidget { background: rgba(255, 190, 0, 70); border-radius: 4px; }"));
    QTimer::singleShot(1400, this, [this] { setStyleSheet(QString()); });
}

void SlotRowWidget::setDropHighlight(int state) {
    if (state == 0) setStyleSheet(QString());
    else setStyleSheet(QStringLiteral("SlotRowWidget { background: rgba(%1, 60); border: 1px solid rgb(%1); border-radius: 4px; }")
                           .arg(state == 1 ? QStringLiteral("63, 185, 80") : QStringLiteral("248, 81, 73")));
}

void SlotRowWidget::dragEnterEvent(QDragEnterEvent* event) {
    const auto payload = devicePayload(event->mimeData());
    if (!payload) return;
    const bool ok = slotAccepts(info_.spec, payload->capabilities);
    setDropHighlight(ok ? 1 : 2);
    if (ok) event->acceptProposedAction();
}

void SlotRowWidget::dragLeaveEvent(QDragLeaveEvent*) { setDropHighlight(0); }

void SlotRowWidget::dropEvent(QDropEvent* event) {
    setDropHighlight(0);
    const auto payload = devicePayload(event->mimeData());
    if (!payload || !slotAccepts(info_.spec, payload->capabilities)) return;
    event->acceptProposedAction();
    Q_EMIT deviceChosen(info_.address, payload->key, payload->capabilities.bits);
}

}  // namespace wpiemu::app::editor
