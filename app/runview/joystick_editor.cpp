#include "joystick_editor.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "ui/widget_support.h"

namespace wpiemu::app::runview {
namespace {

using Target = JoystickManager::LearnTarget;
using Kind = Target::Kind;

joy::Input inputFor(const joy::JoystickProfile& p, const Target& t) {
    const auto idx = static_cast<std::size_t>(std::max(0, t.index));
    switch (t.kind) {
        case Kind::AxisFirst: return idx < p.axes.size() ? p.axes[idx].first : joy::Input{};
        case Kind::AxisSecond: return idx < p.axes.size() ? p.axes[idx].second : joy::Input{};
        case Kind::Button: return idx < p.buttons.size() ? p.buttons[idx] : joy::Input{};
        case Kind::PovUp: return p.pov.up;
        case Kind::PovRight: return p.pov.right;
        case Kind::PovDown: return p.pov.down;
        case Kind::PovLeft: return p.pov.left;
        case Kind::PovPad: return p.pov.pad;
    }
    return {};
}

}  // namespace

ValueBar::ValueBar(bool led, QWidget* parent) : QWidget(parent), led_(led) {
    setSizePolicy(led ? QSizePolicy::Fixed : QSizePolicy::Expanding, QSizePolicy::Fixed);
}

QSize ValueBar::sizeHint() const { return led_ ? QSize(14, 14) : QSize(60, 14); }

void ValueBar::setValue(float value) {
    if (value == value_) return;
    value_ = value;
    update();
}

void ValueBar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QColor groove = palette().color(QPalette::Mid);
    const QColor fill = palette().color(QPalette::Highlight);
    if (led_) {
        const QRectF r = QRectF(rect()).adjusted(2, 2, -2, -2);
        p.setPen(Qt::NoPen);
        p.setBrush(value_ > 0.5f ? fill : groove);
        p.drawEllipse(r);
        return;
    }
    const QRectF r = QRectF(rect()).adjusted(0.5, 3, -0.5, -3);
    p.setPen(Qt::NoPen);
    p.setBrush(groove);
    p.drawRoundedRect(r, 3, 3);
    const double mid = r.center().x();
    const double half = r.width() / 2.0;
    const double v = std::clamp(static_cast<double>(value_), -1.0, 1.0);
    QRectF bar = r;
    bar.setLeft(std::min(mid, mid + v * half));
    bar.setRight(std::max(mid, mid + v * half));
    p.setBrush(fill);
    p.drawRoundedRect(bar, 3, 3);
    p.setPen(palette().color(QPalette::WindowText));
    p.drawLine(QPointF(mid, r.top() - 2), QPointF(mid, r.bottom() + 2));
}

JoystickEditor::JoystickEditor(JoystickManager& manager, QWidget* parent) : QWidget(parent), manager_(manager) {
    setObjectName(QStringLiteral("runview.joystickEditor"));

    slot_ = new QComboBox(this);
    slot_->setAccessibleName(tr("Joystick port"));
    slot_->setToolTip(tr("Driver Station joystick port the robot code reads (0 = first)"));
    device_ = new QComboBox(this);
    device_->setAccessibleName(tr("Input device"));
    device_->setToolTip(tr("Keyboard or gamepad that drives this joystick port"));
    notice_ = new QLabel(this);
    notice_->setWordWrap(true);
    notice_->setVisible(false);
    hint_ = ui::mutedLabel(tr("Click a binding, then press the key or move the control you want to use. Esc cancels."), this);

    auto* header = new QHBoxLayout;
    header->addWidget(new QLabel(tr("Port"), this));
    header->addWidget(slot_);
    header->addWidget(new QLabel(tr("Device"), this));
    header->addWidget(device_, 1);

    auto* content = new QWidget;
    auto* grid = new QGridLayout(content);
    grid->setContentsMargins(0, 0, 4, 0);
    grid->setHorizontalSpacing(6);
    grid->setVerticalSpacing(4);
    int row = 0;
    auto section = [&](const QString& title) {
        auto* label = new QLabel(title, content);
        QFont f = label->font();
        f.setBold(true);
        label->setFont(f);
        grid->addWidget(label, row++, 0, 1, 6);
    };

    section(tr("Axes"));
    for (int a = 0; a < joy::kAxisCount; ++a, ++row) {
        const auto idx = static_cast<std::size_t>(a);
        grid->addWidget(new QLabel(joy::axisName(a), content), row, 0);
        axisFirst_[idx] = makeBindButton({Kind::AxisFirst, a});
        axisSecond_[idx] = makeBindButton({Kind::AxisSecond, a});
        axisFirst_[idx]->setObjectName(QStringLiteral("axisFirst%1").arg(a));
        axisInvert_[idx] = new QCheckBox(tr("Invert"), content);
        axisBar_[idx] = new ValueBar(false, content);
        grid->addWidget(axisFirst_[idx], row, 1);
        grid->addWidget(axisSecond_[idx], row, 2);
        grid->addWidget(axisInvert_[idx], row, 3);
        grid->addWidget(axisBar_[idx], row, 4, 1, 2);
        connect(axisInvert_[idx], &QCheckBox::toggled, this, [this, a](bool on) {
            if (updating_) return;
            joy::JoystickProfile p = manager_.profile(currentSlot());
            p.axes[static_cast<std::size_t>(a)].invert = on;
            manager_.setProfile(currentSlot(), p);
        });
    }

    section(tr("Buttons"));
    for (int b = 0; b < joy::kButtonCount; ++b) {
        const auto idx = static_cast<std::size_t>(b);
        const int r = row + b % 6;
        const int c = (b / 6) * 3;
        auto* name = new QLabel(QString::number(b + 1), content);
        name->setToolTip(joy::buttonName(b));
        name->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        buttonBind_[idx] = makeBindButton({Kind::Button, b});
        buttonBind_[idx]->setObjectName(QStringLiteral("button%1").arg(b));
        buttonBind_[idx]->setToolTip(tr("Button %1 — %2").arg(b + 1).arg(joy::buttonName(b)));
        buttonLed_[idx] = new ValueBar(true, content);
        // Column layout: [name][bind][led] x2 -> columns 0..2 and 3..5 (reusing the axis columns).
        grid->addWidget(name, r, c);
        grid->addWidget(buttonBind_[idx], r, c + 1);
        grid->addWidget(buttonLed_[idx], r, c + 2);
    }
    row += 6;

    section(tr("POV (D-pad)"));
    povUp_ = makeBindButton({Kind::PovUp, 0});
    povRight_ = makeBindButton({Kind::PovRight, 0});
    povDown_ = makeBindButton({Kind::PovDown, 0});
    povLeft_ = makeBindButton({Kind::PovLeft, 0});
    povPad_ = makeBindButton({Kind::PovPad, 0});
    povUp_->setToolTip(tr("POV up"));
    povRight_->setToolTip(tr("POV right"));
    povDown_->setToolTip(tr("POV down"));
    povLeft_->setToolTip(tr("POV left"));
    povValue_ = new QLabel(content);
    auto* povRow = new QHBoxLayout;
    povRow->addWidget(povLeft_);
    povRow->addWidget(povUp_);
    povRow->addWidget(povDown_);
    povRow->addWidget(povRight_);
    povRow->addWidget(povPad_);
    povRow->addWidget(povValue_);
    povRow->addStretch(1);
    grid->addLayout(povRow, row++, 0, 1, 6);
    grid->setRowStretch(row, 1);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(4, 1);

    auto* scroll = new QScrollArea(this);
    scroll->setWidget(content);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);

    auto* reset = new QPushButton(tr("Reset to defaults"), this);
    reset->setToolTip(tr("Restore the default bindings for the selected device"));
    connect(reset, &QPushButton::clicked, this, [this] { manager_.resetProfile(currentSlot()); });

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addLayout(header);
    layout->addWidget(notice_);
    layout->addWidget(scroll, 1);
    layout->addWidget(hint_);
    layout->addWidget(reset, 0, Qt::AlignRight);

    for (int i = 0; i < joy::kSlotCount; ++i) slot_->addItem(QString());
    rebuildDevices();

    connect(slot_, &QComboBox::currentIndexChanged, this, [this] { rebuildDevices(); });
    connect(device_, &QComboBox::activated, this, [this](int i) {
        if (updating_) return;
        manager_.setSourceId(currentSlot(), device_->itemData(i).toString());
    });
    connect(&manager_, &JoystickManager::slotChanged, this, [this] { rebuildDevices(); });
    connect(&manager_, &JoystickManager::sourcesChanged, this, [this] { rebuildDevices(); });
    connect(&manager_, &JoystickManager::learningChanged, this, [this] { refreshBindings(); });
    connect(&manager_, &JoystickManager::gamepadConnected, this, [this](const JoystickSourceInfo& info, int assigned) {
        notice_->setText(tr("Gamepad connected: %1 — assigned to port %2.").arg(info.name).arg(assigned));
        notice_->setVisible(true);
    });

    previewTimer_ = new QTimer(this);
    previewTimer_->setInterval(33);
    connect(previewTimer_, &QTimer::timeout, this, &JoystickEditor::refreshPreview);
}

QPushButton* JoystickEditor::makeBindButton(Target target) {
    auto* b = new QPushButton(this);
    b->setCheckable(true);
    b->setMinimumWidth(44);
    b->setFocusPolicy(Qt::TabFocus);
    connect(b, &QPushButton::clicked, this, [this, target, b] { toggleLearn(target, b); });
    binds_.push_back({b, target});
    return b;
}

int JoystickEditor::currentSlot() const { return std::max(0, slot_->currentIndex()); }

void JoystickEditor::setCurrentSlot(int slot) { slot_->setCurrentIndex(std::clamp(slot, 0, joy::kSlotCount - 1)); }

void JoystickEditor::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    previewTimer_->start();
}

void JoystickEditor::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    previewTimer_->stop();
    manager_.cancelLearn();
}

void JoystickEditor::toggleLearn(Target target, QPushButton* source) {
    const bool already = manager_.learning() && manager_.learnSlot() == currentSlot() && manager_.learnTarget() == target;
    if (already) manager_.cancelLearn();
    else manager_.beginLearn(currentSlot(), target);
    source->setChecked(manager_.learning() && manager_.learnTarget() == target);
    refreshBindings();
}

void JoystickEditor::rebuildDevices() {
    updating_ = true;
    for (int i = 0; i < joy::kSlotCount; ++i) {
        const QString device = manager_.slotDeviceName(i);
        slot_->setItemText(i, device.isEmpty() ? tr("Port %1 — not assigned").arg(i) : tr("Port %1 — %2").arg(i).arg(device));
    }
    const int slot = currentSlot();
    device_->clear();
    device_->addItem(tr("Not assigned"), QString());
    for (const auto& info : manager_.availableSources()) device_->addItem(info.name, info.id);
    const QString current = manager_.slot(slot).sourceId;
    int index = device_->findData(current);
    if (index < 0 && !current.isEmpty()) {
        device_->addItem(tr("%1 (disconnected)").arg(manager_.slotDeviceName(slot)), current);
        index = device_->count() - 1;
    }
    device_->setCurrentIndex(std::max(0, index));
    if (!manager_.gamepadBackendAvailable()) {
        notice_->setText(tr("Gamepad support is not available in this build; the keyboard always works."));
        notice_->setVisible(true);
    }
    updating_ = false;
    refreshBindings();
}

void JoystickEditor::refreshBindings() {
    updating_ = true;
    const int slot = currentSlot();
    const bool keyboard = manager_.usesKeyboard(slot);
    const bool assigned = !manager_.slot(slot).sourceId.isEmpty();
    const joy::JoystickProfile& profile = manager_.profile(slot);

    for (const auto& ref : binds_) {
        const bool learning = manager_.learning() && manager_.learnSlot() == slot && manager_.learnTarget() == ref.target;
        ref.button->setChecked(learning);
        QString text = learning ? tr("Press input…") : inputFor(profile, ref.target).displayText();
        if (!learning) {
            if (ref.target.kind == Kind::PovUp) text = QStringLiteral("↑ ") + text;
            else if (ref.target.kind == Kind::PovRight) text = QStringLiteral("→ ") + text;
            else if (ref.target.kind == Kind::PovDown) text = QStringLiteral("↓ ") + text;
            else if (ref.target.kind == Kind::PovLeft) text = QStringLiteral("← ") + text;
        }
        ref.button->setText(text);
        ref.button->setEnabled(assigned);
    }
    for (std::size_t a = 0; a < axisFirst_.size(); ++a) {
        axisSecond_[a]->setVisible(keyboard);
        axisFirst_[a]->setToolTip(keyboard ? tr("Key that pushes the axis toward −1") : tr("Gamepad axis"));
        axisSecond_[a]->setToolTip(tr("Key that pushes the axis toward +1"));
        axisInvert_[a]->setChecked(profile.axes[a].invert);
        axisInvert_[a]->setEnabled(assigned);
    }
    povUp_->setVisible(keyboard);
    povRight_->setVisible(keyboard);
    povDown_->setVisible(keyboard);
    povLeft_->setVisible(keyboard);
    povPad_->setVisible(!keyboard);
    updating_ = false;
}

void JoystickEditor::refreshPreview() {
    const int slot = currentSlot();
    const auto& state = manager_.lastStates()[static_cast<std::size_t>(slot)];
    for (std::size_t a = 0; a < axisBar_.size(); ++a) axisBar_[a]->setValue(a < state.axes.size() ? state.axes[a] : 0.f);
    for (int b = 0; b < joy::kButtonCount; ++b) buttonLed_[static_cast<std::size_t>(b)]->setValue((state.buttons >> b) & 1u ? 1.f : 0.f);
    const int pov = state.povs.empty() ? -1 : state.povs[0];
    povValue_->setText(pov < 0 ? QStringLiteral("—") : QStringLiteral("%1°").arg(pov));
}

}  // namespace wpiemu::app::runview
