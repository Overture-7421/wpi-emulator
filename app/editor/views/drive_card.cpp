#include "views/drive_card.h"

#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QToolButton>
#include <QVBoxLayout>

#include "model/config_ops.h"
#include "model/motion_links.h"
#include "model/property_groups.h"
#include "model/slot_status.h"
#include "model/tr.h"
#include "ui/qt_text.h"
#include "ui/widget_support.h"

namespace wpiemu::app::editor {

namespace {

using ui::mutedLabel;
using ui::qs;

QToolButton* segment(const QString& text, const QString& objectName, QWidget* parent) {
    auto* b = new QToolButton(parent);
    b->setText(text);
    b->setObjectName(objectName);
    b->setCheckable(true);
    b->setStyleSheet(QStringLiteral("QToolButton { border: 1px solid palette(mid); padding: 4px 8px; background: transparent; } "
                                    "QToolButton:checked { background-color: #e9a23b; color: #10151c; font-weight: bold; }"));
    b->setToolButtonStyle(Qt::ToolButtonTextOnly);
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return b;
}

}  // namespace

bool DriveCard::applicable(EditorController& controller, const SelectionRef& owner) {
    const MotionIndex motion(controller.document().config(), controller.document().catalog());
    const OwnerMotion om = motion.ownerMotion(owner);
    return !om.axes.empty();
}

DriveCard::DriveCard(EditorController& controller, const IconFactory& icons, const SelectionRef& owner, Registrar registrar,
                     QWidget* parent)
    : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    const sdk::RobotConfig& config = controller.document().config();
    const EditorCatalog& catalog = controller.document().catalog();
    const MotionIndex motion(config, catalog);
    const OwnerMotion om = motion.ownerMotion(owner);
    const std::vector<sdk::MotionAxisInfo> allAxes = motion.axes();
    const bool wheel = owner.kind == SelKind::Wheel;
    const std::string slotOwner = wheel ? std::string(sdk::slot_owner::kDrivetrain) : owner.owner;
    const std::vector<SlotInfo> infos = slotInfos(config, catalog, controller.devices(), slotOwner);
    std::vector<PropertyGroup> driveGroups;
    for (auto& g : propertyGroupsFor(config, catalog, owner))
        if (isDriveGroup(g.id)) driveGroups.push_back(std::move(g));

    const auto makeRow = [&](const SlotInfo& info, QWidget* parentWidget) {
        auto* row = new DeviceRowWidget(icons, parentWidget);
        row->setInfo(info);
        connect(row, &DeviceRowWidget::clearRequested, this, [&controller](const sdk::SlotAddress& s) { controller.clearSlot(s); });
        connect(row, &DeviceRowWidget::reversedToggled, this,
                [&controller](const sdk::SlotAddress& s, bool on) { controller.setMotorReversed(s, on); });
        if (registrar.row) registrar.row(row);
        return row;
    };
    const auto makeForm = [&](const PropertyGroup& group, QWidget* parentWidget) {
        auto* form = new PropertyFormWidget(group, controller.settings().units(), parentWidget);
        connect(form, &PropertyFormWidget::valueEdited, this, [&controller, owner](const QString& groupId, const QString& key, const sdk::Value& v) {
            controller.document().setProperty(owner, groupId.toStdString(), key.toStdString(), v);
        });
        if (registrar.form) registrar.form(form);
        return form;
    };

    for (const sdk::MotionAxisInfo& axis : om.axes) {
        const QString name = qs(axis.address.toString());
        auto* box = new QWidget(this);
        auto* v = new QVBoxLayout(box);
        v->setContentsMargins(0, 0, 0, 0);
        v->setSpacing(6);
        if (om.axes.size() > 1) {
            QString title = qs(axis.address.slot);
            title[0] = title[0].toUpper();
            auto* label = new QLabel(QStringLiteral("<b>%1</b>").arg(title.toHtmlEscaped()), box);
            v->addWidget(label);
        }

        const sdk::MotionLink* link = motion.incoming(axis.address);

        // Own motors | Linked to another part.
        auto* segments = new QWidget(box);
        auto* sh = new QHBoxLayout(segments);
        sh->setContentsMargins(0, 0, 0, 0);
        sh->setSpacing(0);
        auto* ownButton = segment(ETR("Own motors"), QStringLiteral("drive.own:") + name, segments);
        auto* linkedButton = segment(ETR("Linked to another part"), QStringLiteral("drive.linked:") + name, segments);
        auto* group = new QButtonGroup(segments);
        group->setExclusive(true);
        group->addButton(ownButton);
        group->addButton(linkedButton);
        ownButton->setToolTip(ETR("This part has motor controllers of its own."));
        linkedButton->setToolTip(ETR("This part follows another part's motion and sensors, so it takes no motor of its own."));
        sh->addWidget(ownButton);
        sh->addWidget(linkedButton);
        v->addWidget(segments);

        // Own motors: motor model / count / gearing and one row per controller.
        auto* ownBox = new QWidget(box);
        auto* ov = new QVBoxLayout(ownBox);
        ov->setContentsMargins(0, 0, 0, 0);
        ov->setSpacing(6);
        const std::string groupId = wheel ? "wheel.drive" : "mech.drive:" + axis.address.slot;
        for (const PropertyGroup& g : driveGroups)
            if (g.id == groupId && !g.schema.properties.empty()) ov->addWidget(makeForm(g, ownBox));
        // Whatever the part is, its motors read Motor 1..N.
        int motorNumber = 0;
        for (const sdk::SlotAddress& motor : axis.motorSlots) {
            ++motorNumber;
            for (const SlotInfo& info : infos)
                if (info.address == motor) {
                    auto* row = makeRow(info, ownBox);
                    row->setLabel(QStringLiteral("Motor %1").arg(motorNumber));
                    ov->addWidget(row);
                }
        }
        v->addWidget(ownBox);

        // Linked: source, signed ratio, units and the reason a link was refused.
        auto* linkBox = new QWidget(box);
        auto* lv = new QVBoxLayout(linkBox);
        lv->setContentsMargins(0, 0, 0, 0);
        lv->setSpacing(4);
        lv->addWidget(mutedLabel(ETR("Follows its source's motion and sensors, so it takes no motor of its own. Negative ratios reverse the motion."), linkBox));
        auto* source = ui::quietWheel(new QComboBox(linkBox));
        source->setObjectName(QStringLiteral("drivenBy.source:") + name);
        source->addItem(ETR("Choose a source..."), -1);
        int current = 0;
        for (std::size_t i = 0; i < allAxes.size(); ++i) {
            if (allAxes[i].address == axis.address) continue;
            source->addItem(ETR("Driven by %1").arg(qs(axisLabel(config, catalog, allAxes[i].address))), static_cast<int>(i));
            if (link != nullptr && link->source == allAxes[i].address) current = source->count() - 1;
        }
        source->setCurrentIndex(current);
        lv->addWidget(source);

        auto* ratioRow = new QWidget(linkBox);
        auto* h = new QHBoxLayout(ratioRow);
        h->setContentsMargins(0, 0, 0, 0);
        h->addWidget(new QLabel(ETR("Ratio"), ratioRow));
        auto* ratio = ui::quietWheel(new QDoubleSpinBox(ratioRow));
        ratio->setObjectName(QStringLiteral("drivenBy.ratio:") + name);
        ratio->setRange(-1e9, 1e9);
        ratio->setDecimals(6);
        ratio->setValue(1.0);
        h->addWidget(ratio, 1);
        auto* units = new QLabel(ratioRow);
        units->setObjectName(QStringLiteral("drivenBy.units:") + name);
        units->setEnabled(false);
        h->addWidget(units);
        lv->addWidget(ratioRow);

        auto* error = new QLabel(linkBox);
        error->setObjectName(QStringLiteral("drivenBy.error:") + name);
        error->setWordWrap(true);
        error->setTextFormat(Qt::PlainText);
        error->setStyleSheet(QStringLiteral("color: #f85149;"));
        error->setVisible(false);
        lv->addWidget(error);
        v->addWidget(linkBox);

        const auto showMode = [=](bool linked) {
            ownBox->setVisible(!linked);
            linkBox->setVisible(linked);
        };
        (link != nullptr ? linkedButton : ownButton)->setChecked(true);
        showMode(link != nullptr);

        const auto kindOf = [allAxes, axis](int sourceIndex) { return sourceIndex >= 0 ? allAxes[static_cast<std::size_t>(sourceIndex)].kind : axis.kind; };
        const auto sync = [=] {
            const int idx = source->currentData().toInt();
            ratioRow->setVisible(idx >= 0);
            units->setText(qs(ratioUnits(kindOf(idx), axis.kind)));
        };
        if (link != nullptr) {
            const sdk::MotionAxisInfo* from = motion.axis(link->source);
            ratio->setValue(ratioFromSi(link->ratio, from != nullptr ? from->kind : axis.kind, axis.kind));
        }
        sync();
        const auto apply = [=, &controller] {
            sync();
            const int idx = source->currentData().toInt();
            error->setVisible(false);
            if (idx < 0) {
                if (motion.incoming(axis.address) != nullptr) controller.clearDrivenBy(axis.address);
                return;
            }
            const QString problem = controller.setDrivenBy(axis.address, allAxes[static_cast<std::size_t>(idx)].address, ratio->value());
            if (problem.isEmpty()) return;
            error->setText(problem);
            error->setVisible(true);
        };
        connect(source, &QComboBox::activated, this, [apply] { apply(); });
        connect(ratio, &QDoubleSpinBox::editingFinished, this, [=] {
            if (source->currentData().toInt() >= 0) apply();
        });
        connect(linkedButton, &QToolButton::clicked, this, [showMode] { showMode(true); });
        connect(ownButton, &QToolButton::clicked, this, [=, &controller] {
            if (motion.incoming(axis.address) != nullptr) {
                controller.clearDrivenBy(axis.address);   // rebuilds the page
                return;
            }
            showMode(false);
        });
        layout->addWidget(box);
    }

}

}  // namespace wpiemu::app::editor