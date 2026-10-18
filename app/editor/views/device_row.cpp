#include "views/device_row.h"

#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {
QString qs(const std::string& s) { return QString::fromStdString(s); }
}  // namespace

DeviceRowWidget::DeviceRowWidget(const IconFactory& icons, QWidget* parent)
    : QFrame(parent), icons_(icons), dot_(new QLabel(this)), title_(new QLabel(this)), device_(new QLabel(this)),
      reversed_(new QCheckBox(ETR("Reversed"), this)), clear_(new QToolButton(this)) {
    setObjectName(QStringLiteral("deviceRow"));
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(4, 3, 4, 3);
    h->setSpacing(8);
    dot_->setFixedSize(16, 16);
    h->addWidget(dot_, 0, Qt::AlignTop);
    auto* texts = new QVBoxLayout;
    texts->setSpacing(0);
    QFont bold = title_->font();
    bold.setBold(true);
    title_->setFont(bold);
    device_->setWordWrap(true);
    device_->setEnabled(false);
    texts->addWidget(title_);
    texts->addWidget(device_);
    h->addLayout(texts, 1);
    reversed_->setObjectName(QStringLiteral("slot.reversed"));
    reversed_->setToolTip(ETR("Check when this motor spins against the mechanism's positive direction (mounted mirrored, or inverted in "
                              "robot code). The simulator then flips its command and feedback."));
    h->addWidget(reversed_);
    clear_->setText(QStringLiteral("\xC3\x97"));
    clear_->setToolTip(ETR("Unmap this device"));
    clear_->setAutoRaise(true);
    h->addWidget(clear_);

    connect(clear_, &QToolButton::clicked, this, [this] { Q_EMIT clearRequested(info_.address); });
    connect(reversed_, &QCheckBox::toggled, this, [this](bool on) { Q_EMIT reversedToggled(info_.address, on); });
}

QString DeviceRowWidget::slotLabel() const { return qs(info_.spec.label.empty() ? info_.spec.name : info_.spec.label); }

void DeviceRowWidget::setInfo(const SlotInfo& info) {
    info_ = info;
    dot_->setPixmap(icons_.status(info.state).pixmap(16, 16));
    title_->setText(label_.isEmpty() ? slotLabel() : label_);
    title_->setToolTip(qs(info.spec.description));

    QString text;
    if (info.mapped) {
        text = mappedLabel(info);
    } else if (info.spec.type == sdk::SlotType::Motor && info.spec.ordinal > 1) {
        text = ETR("Not mapped \xE2\x80\x93 mirrors motor 1");
    } else if (info.spec.required) {
        text = ETR("Not mapped \xE2\x80\x93 drag a controller onto this part in the tree");
    } else {
        text = ETR("Not mapped");
    }
    device_->setText(text);
    device_->setToolTip(info.detail);
    const bool attention = info.state == SlotState::Unmapped || info.state == SlotState::Incompatible ||
                           info.state == SlotState::AliasConflict || info.state == SlotState::NotDetected;
    device_->setEnabled(!attention);
    device_->setStyleSheet(attention ? QStringLiteral("color: %1;").arg(IconFactory::stateColor(info.state).name()) : QString());

    clear_->setVisible(info.mapped.has_value());
    reversed_->setVisible(info.mapped.has_value() && info.spec.type == sdk::SlotType::Motor);
    const QSignalBlocker block(reversed_);
    reversed_->setChecked(info.reversed);
}

void DeviceRowWidget::flash() {
    setStyleSheet(QStringLiteral("QFrame#deviceRow { background: rgba(255, 190, 0, 70); border-radius: 4px; }"));
    QTimer::singleShot(1400, this, [this] { setStyleSheet(QString()); });
}

}  // namespace wpiemu::app::editor
