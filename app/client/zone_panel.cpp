#include "client/zone_panel.h"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <cmath>

#include "runview/joystick_types.h"
#include "ui/qt_text.h"
#include "ui/widget_support.h"
#include "wpiemu/sdk/match/robot_appearance.h"

namespace wpiemu::app::client {
namespace {

constexpr float kRadToDeg = 180.f / 3.14159265358979323846f;
constexpr float kDegToRad = 3.14159265358979323846f / 180.f;

QString zoneLabel(const ClientController& controller, std::size_t zone) {
    QString kind;
    if (const auto view = controller.view(); view && view->own.appearance && zone < view->own.appearance->zones.size()) {
        const std::string& key = view->own.appearance->zones[zone].kind;
        kind = QString::fromStdString(key);
        for (const sdk::ZoneKindInfo& info : controller.scene().zoneKinds)
            if (info.kind == key && !info.displayName.empty()) kind = QString::fromStdString(info.displayName);
    }
    const QString button = zone < static_cast<std::size_t>(runview::joy::kButtonCount)
                               ? runview::joy::buttonName(static_cast<int>(zone))
                               : QString();
    const QString name = kind.isEmpty() ? QCoreApplication::translate("Client", "Zone %1").arg(zone + 1) : kind;
    return button.isEmpty() ? name : QStringLiteral("%1  %2").arg(button, name);
}

}  // namespace

ZonePanel::ZonePanel(ClientController& controller, QWidget* parent) : QFrame(parent), controller_(controller) {
    setObjectName(QStringLiteral("client.zonePanel"));
    setFrameShape(QFrame::StyledPanel);
    setMinimumWidth(250);
    setMaximumWidth(300);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);
    layout->addWidget(ui::headingLabel(tr("Zones"), this));
    auto* listHost = new QWidget(this);
    list_ = new QVBoxLayout(listHost);
    list_->setContentsMargins(0, 0, 0, 0);
    list_->setSpacing(4);
    layout->addWidget(listHost);

    poseBox_ = new QWidget(this);
    poseBox_->setObjectName(QStringLiteral("client.zonePose"));
    auto* poseLayout = new QVBoxLayout(poseBox_);
    poseLayout->setContentsMargins(0, 6, 0, 0);
    poseLayout->setSpacing(4);
    poseTitle_ = ui::headingLabel(tr("Move zone"), poseBox_);
    poseLayout->addWidget(poseTitle_);
    const QString names[4] = {tr("Forward"), tr("Left"), tr("Up"), tr("Turn")};
    for (int i = 0; i < 4; ++i) {
        auto* row = new QHBoxLayout;
        auto* name = new QLabel(names[i], poseBox_);
        name->setMinimumWidth(52);
        sliders_[i] = new QSlider(Qt::Horizontal, poseBox_);
        sliders_[i]->setObjectName(QStringLiteral("client.zoneSlider%1").arg(i));
        sliderValues_[i] = new QLabel(poseBox_);
        sliderValues_[i]->setMinimumWidth(52);
        sliderValues_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        row->addWidget(name);
        row->addWidget(sliders_[i], 1);
        row->addWidget(sliderValues_[i]);
        poseLayout->addLayout(row);
        connect(sliders_[i], &QSlider::valueChanged, this, &ZonePanel::onSlider);
    }
    reset_ = new QPushButton(tr("Reset zone"), poseBox_);
    reset_->setObjectName(QStringLiteral("client.zoneReset"));
    poseLayout->addWidget(reset_);
    layout->addWidget(poseBox_);
    connect(reset_, &QPushButton::clicked, this, [this] {
        controller_.resetZone(controller_.zonePoses().selected());
        syncSliders();
    });

    hint_ = ui::captionLabel(tr("Hold the buttons shown, or click one to keep the zone on. The d-pad moves the selected zone, the "
                                "right stick raises it and the triggers turn it."),
                             this);
    hint_->setObjectName(QStringLiteral("client.zoneHint"));
    layout->addWidget(hint_);
    layout->addStretch(1);
    rebuild();
}

void ZonePanel::rebuild() {
    while (QLayoutItem* item = list_->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    buttons_.clear();
    selectors_.clear();

    const std::size_t count = controller_.zonePoses().zoneCount();
    auto* group = new QButtonGroup(this);
    group->setExclusive(true);
    for (std::size_t i = 0; i < count; ++i) {
        auto* row = new QWidget(this);
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(4);
        auto* toggle = new QPushButton(zoneLabel(controller_, i), row);
        toggle->setObjectName(QStringLiteral("client.zone%1").arg(i));
        toggle->setCheckable(true);
        toggle->setChecked((controller_.zoneButtons() >> i) & 1u);
        toggle->setToolTip(tr("Click to keep this zone on; it is also on while its joystick button is held."));
        auto* move = new QPushButton(QStringLiteral("✥"), row);
        move->setObjectName(QStringLiteral("client.zoneSelect%1").arg(i));
        move->setCheckable(true);
        move->setFixedWidth(30);
        move->setToolTip(tr("Select this zone to move it"));
        group->addButton(move);
        rowLayout->addWidget(toggle, 1);
        rowLayout->addWidget(move);
        list_->addWidget(row);
        buttons_.push_back(toggle);
        selectors_.push_back(move);
        connect(toggle, &QPushButton::toggled, this, [this, i](bool on) { controller_.setZoneButton(i, on); });
        connect(move, &QPushButton::clicked, this, [this, i] { selectZone(i); });
    }
    if (!selectors_.empty()) {
        const std::size_t selected = std::min(controller_.zonePoses().selected(), selectors_.size() - 1);
        selectors_[selected]->setChecked(true);
    }
    setVisible(count > 0);
    poseBox_->setVisible(count > 0);

    if (count > 0) {
        const ZonePoseControls& poses = controller_.zonePoses();
        const sdk::Vec3 lo = poses.minPosition();
        const sdk::Vec3 hi = poses.maxPosition();
        const QSignalBlocker b0(sliders_[0]), b1(sliders_[1]), b2(sliders_[2]), b3(sliders_[3]);
        sliders_[0]->setRange(static_cast<int>(std::floor(lo.x * 1000.f)), static_cast<int>(std::ceil(hi.x * 1000.f)));
        sliders_[1]->setRange(static_cast<int>(std::floor(lo.y * 1000.f)), static_cast<int>(std::ceil(hi.y * 1000.f)));
        sliders_[2]->setRange(static_cast<int>(std::floor(lo.z * 1000.f)), static_cast<int>(std::ceil(hi.z * 1000.f)));
        sliders_[3]->setRange(-180, 180);
        syncSliders();
    }
}

void ZonePanel::selectZone(std::size_t zone) {
    controller_.zonePoses().select(zone);
    syncSliders();
    emit zoneSelected(zone);
}

void ZonePanel::syncSliders() {
    const ZonePoseControls& poses = controller_.zonePoses();
    if (poses.zoneCount() == 0) return;
    const std::size_t zone = poses.selected();
    const sdk::Vec3 p = poses.pose(zone).position;
    const float yaw = poses.yawOffsetRad(zone) * kRadToDeg;
    syncing_ = true;
    sliders_[0]->setValue(static_cast<int>(std::lround(p.x * 1000.f)));
    sliders_[1]->setValue(static_cast<int>(std::lround(p.y * 1000.f)));
    sliders_[2]->setValue(static_cast<int>(std::lround(p.z * 1000.f)));
    sliders_[3]->setValue(static_cast<int>(std::lround(yaw)));
    syncing_ = false;
    sliderValues_[0]->setText(tr("%1 m").arg(static_cast<double>(p.x), 0, 'f', 2));
    sliderValues_[1]->setText(tr("%1 m").arg(static_cast<double>(p.y), 0, 'f', 2));
    sliderValues_[2]->setText(tr("%1 m").arg(static_cast<double>(p.z), 0, 'f', 2));
    sliderValues_[3]->setText(QStringLiteral("%1°").arg(static_cast<int>(std::lround(yaw))));
    poseTitle_->setText(tr("Move zone: %1").arg(zoneLabel(controller_, zone)));
    reset_->setEnabled(poses.moved(zone));
}

void ZonePanel::onSlider() {
    if (syncing_) return;
    const std::size_t zone = controller_.zonePoses().selected();
    const sdk::Vec3 position{static_cast<float>(sliders_[0]->value()) / 1000.f, static_cast<float>(sliders_[1]->value()) / 1000.f,
                             static_cast<float>(sliders_[2]->value()) / 1000.f};
    controller_.moveZone(zone, position, static_cast<float>(sliders_[3]->value()) * kDegToRad);
    syncSliders();
}

void ZonePanel::refresh() {
    const auto view = controller_.view();
    if (!view) return;
    const std::uint32_t live = view->own.zoneFlags;
    for (std::size_t i = 0; i < buttons_.size(); ++i) {
        const bool on = i < 32 && ((live >> i) & 1u) != 0;
        QString text = zoneLabel(controller_, i);
        buttons_[i]->setText(on ? QStringLiteral("● ") + text : text);
        const bool wanted = ((controller_.zoneButtons() >> i) & 1u) != 0;
        if (buttons_[i]->isChecked() != wanted) {
            const QSignalBlocker block(buttons_[i]);
            buttons_[i]->setChecked(wanted);
        }
    }
    if (!poseBox_->underMouse()) syncSliders();   // a key moved the zone; leave a slider being dragged alone
}

}  // namespace wpiemu::app::client
