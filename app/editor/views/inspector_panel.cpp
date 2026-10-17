#include "views/inspector_panel.h"

#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <functional>
#include <map>

#include "model/config_ops.h"
#include "model/property_groups.h"
#include "model/tr.h"
#include "views/motion_links_panel.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::app::editor {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

QLabel* mutedLabel(const QString& text, QWidget* parent, bool wrap = true) {
    auto* l = new QLabel(text, parent);
    l->setWordWrap(wrap);
    l->setEnabled(false);
    return l;
}

QString capabilityText(sdk::CapabilitySet caps) {
    struct Cap { sdk::Capability c; const char* name; };
    static const Cap all[] = {{sdk::Capability::MotorOutput, "motor output"}, {sdk::Capability::RotorFeedback, "rotor feedback"},
                              {sdk::Capability::PositionSensor, "position"},   {sdk::Capability::VelocitySensor, "velocity"},
                              {sdk::Capability::Imu, "IMU"},                   {sdk::Capability::DigitalInput, "digital in"},
                              {sdk::Capability::DigitalOutput, "digital out"}, {sdk::Capability::AnalogInput, "analog in"},
                              {sdk::Capability::PwmOutput, "PWM"},             {sdk::Capability::Solenoid, "solenoid"},
                              {sdk::Capability::LimitInputs, "limit inputs"},  {sdk::Capability::SupplyVoltage, "supply voltage"},
                              {sdk::Capability::Led, "LED"},                   {sdk::Capability::Distance, "distance"}};
    QStringList names;
    for (const auto& c : all)
        if (caps.has(c.c)) names << QString::fromLatin1(c.name);
    return names.join(QStringLiteral(", "));
}

}  // namespace

InspectorPanel::InspectorPanel(EditorController& controller, const IconFactory& icons, QWidget* parent)
    : QWidget(parent), controller_(controller), icons_(icons), scroll_(new QScrollArea(this)) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    scroll_->setWidgetResizable(true);
    scroll_->setFrameShape(QFrame::NoFrame);
    scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    outer->addWidget(scroll_);

    connect(&controller_.selection(), &SelectionState::changed, this, [this] { rebuild(); });
    connect(&controller_.selection(), &SelectionState::fieldFocusRequested, this, [this](const QString& key) { focusField(key); });
    connect(&controller_.document(), &EditorDocument::changed, this, [this](bool authoritative) { refresh(authoritative); });
    connect(&controller_, &EditorController::devicesUpdated, this, [this] { refresh(); });
    connect(&controller_, &EditorController::issuesUpdated, this, [this] { refresh(); });
    connect(&controller_.settings(), &EditorSettings::changed, this, [this] {
        if (controller_.settings().units() != shownUnits_) rebuild();   // snapping etc. do not affect the forms
    });
    rebuild();
}

PropertyFormWidget* InspectorPanel::form(const QString& groupId) const {
    for (auto* f : forms_)
        if (qs(f->groupId()) == groupId) return f;
    return nullptr;
}

void InspectorPanel::clearContent() {
    forms_.clear();
    slotRows_.clear();
    title_ = nullptr;
    if (QWidget* old = scroll_->takeWidget()) old->deleteLater();
    content_ = nullptr;
    layout_ = nullptr;
}

QWidget* InspectorPanel::linkButton(const QString& text, const std::function<void()>& action) {
    auto* b = new QToolButton(content_);
    b->setText(text);
    b->setAutoRaise(true);
    b->setToolButtonStyle(Qt::ToolButtonTextOnly);
    b->setCursor(Qt::PointingHandCursor);
    connect(b, &QToolButton::clicked, this, [action] { action(); });
    return b;
}

QWidget* InspectorPanel::addSection(const QString& title, QWidget* body, bool collapsed) {
    auto* section = new QWidget(content_);
    auto* v = new QVBoxLayout(section);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(6);
    auto* head = new QToolButton(section);
    head->setText(title);
    head->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    head->setArrowType(collapsed ? Qt::RightArrow : Qt::DownArrow);
    head->setCheckable(true);
    head->setChecked(!collapsed);
    head->setAutoRaise(true);
    QFont f = head->font();
    f.setBold(true);
    head->setFont(f);
    head->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    head->setStyleSheet(QStringLiteral("QToolButton { text-align: left; padding: 2px 0; }"));
    v->addWidget(head);
    auto* line = new QFrame(section);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Plain);
    line->setEnabled(false);
    v->addWidget(line);
    body->setParent(section);
    v->addWidget(body);
    body->setVisible(!collapsed);
    connect(head, &QToolButton::toggled, section, [head, body](bool on) {
        head->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
        body->setVisible(on);
    });
    layout_->addWidget(section);
    return section;
}

void InspectorPanel::addHeader(const SelectionRef& target, const std::vector<PropertyGroup>& /*groups*/) {
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    QString title = describeSelection(c, cat, target);
    QString subtitle;
    NodeKind icon = NodeKind::Robot;
    switch (target.kind) {
        case SelKind::None:
        case SelKind::Robot: subtitle = ETR("Robot overview"); icon = NodeKind::Robot; break;
        case SelKind::Chassis:
        case SelKind::ChassisCom: subtitle = ETR("Frame, mass, centre of mass"); icon = NodeKind::Chassis; break;
        case SelKind::Drivetrain: subtitle = ETR("Type and traction model"); icon = NodeKind::Drivetrain; break;
        case SelKind::Wheel:
            if (const auto* w = ops::findWheel(c, target.sub)) subtitle = qs(w->driveMotor.name);
            icon = NodeKind::Wheel;
            break;
        case SelKind::Mechanism:
            if (const auto* m = c.findMechanism(target.owner))
                if (const auto* info = cat.findTemplate(m->type)) subtitle = qs(info->displayName) + QStringLiteral(" \xC2\xB7 ") + qs(info->category);
            icon = NodeKind::Mechanism;
            break;
        case SelKind::Primitive: {
            const QString owner = target.owner == ops::kChassisOwner ? ETR("chassis")
                                  : c.findMechanism(target.owner) != nullptr ? qs(ops::displayNameFor(*c.findMechanism(target.owner), cat))
                                                                              : qs(target.owner);
            subtitle = ETR("Shape of %1").arg(owner);
            icon = NodeKind::Shape;
            break;
        }
        case SelKind::Plugin: subtitle = ETR("Game rules and configuration"); icon = NodeKind::Plugin; break;
        case SelKind::Device: title = ETR("Detected device"); icon = NodeKind::Robot; break;
        default: break;
    }

    auto* head = new QWidget(content_);
    auto* h = new QHBoxLayout(head);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(10);
    auto* iconLabel = new QLabel(head);
    iconLabel->setPixmap(icons_.node(icon).pixmap(28, 28));
    iconLabel->setFixedSize(28, 28);
    iconLabel->setScaledContents(true);
    h->addWidget(iconLabel, 0, Qt::AlignTop);
    auto* texts = new QVBoxLayout;
    texts->setSpacing(0);
    title_ = new QLabel(title, head);
    QFont f = title_->font();
    f.setPointSizeF(f.pointSizeF() * 1.25);
    f.setBold(true);
    title_->setFont(f);
    title_->setWordWrap(true);
    texts->addWidget(title_);
    texts->addWidget(mutedLabel(subtitle, head));
    if (target.kind == SelKind::Primitive && target.owner != ops::kChassisOwner && c.findMechanism(target.owner) != nullptr) {
        const std::string owner = target.owner;
        texts->addWidget(linkButton(ETR("\xE2\x86\x90 Back to mechanism"), [this, owner] { controller_.selection().set(SelectionRef::mechanism(owner)); }));
    } else if (target.kind == SelKind::Primitive) {
        texts->addWidget(linkButton(ETR("\xE2\x86\x90 Back to chassis"), [this] { controller_.selection().set(SelectionRef::chassis()); }));
    }
    h->addLayout(texts, 1);

    auto addAction = [&](const QString& text, const QString& tip, const std::function<void()>& fn) {
        auto* b = new QToolButton(head);
        b->setText(text);
        b->setToolTip(tip);
        b->setAutoRaise(true);
        connect(b, &QToolButton::clicked, this, [fn] { fn(); });
        h->addWidget(b, 0, Qt::AlignTop);
    };
    if (target.kind == SelKind::Mechanism || target.kind == SelKind::Primitive)
        addAction(ETR("Duplicate"), ETR("Duplicate (Ctrl+D)"), [this] { controller_.duplicateSelection(); });
    if (target.kind == SelKind::Mechanism || target.kind == SelKind::Primitive || target.kind == SelKind::Wheel)
        addAction(ETR("Delete"), ETR("Delete (Del)"), [this] { controller_.deleteSelection(); });
    layout_->addWidget(head);
}

void InspectorPanel::addChecklist() {
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    int unfilled = 0;
    for (const auto& owner : ops::slotOwners(c))
        for (const auto& info : slotInfos(c, cat, controller_.devices(), owner))
            if (info.state == SlotState::Unmapped) ++unfilled;
    const bool hasMechanisms = !c.mechanisms.empty();
    const bool hasDevices = !controller_.devices().empty();

    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(4);
    auto step = [&](bool done, const QString& text, const QString& hint, QWidget* action = nullptr) {
        auto* row = new QWidget(body);
        auto* h = new QHBoxLayout(row);
        h->setContentsMargins(0, 0, 0, 0);
        auto* mark = new QLabel(done ? QStringLiteral("\xE2\x9C\x94") : QStringLiteral("\xE2\x97\x8B"), row);
        mark->setStyleSheet(QStringLiteral("color: %1; font-weight: bold;").arg(done ? QStringLiteral("#3fb950") : QStringLiteral("#8b949e")));
        mark->setFixedWidth(18);
        h->addWidget(mark, 0, Qt::AlignTop);
        auto* col = new QVBoxLayout;
        col->setSpacing(0);
        auto* t = new QLabel(text, row);
        t->setWordWrap(true);
        col->addWidget(t);
        if (!hint.isEmpty() && !done) col->addWidget(mutedLabel(hint, row));
        h->addLayout(col, 1);
        if (action != nullptr) h->addWidget(action, 0, Qt::AlignTop);
        v->addWidget(row);
    };
    step(true, ETR("Describe the chassis"), {}, linkButton(ETR("Edit"), [this] { controller_.selection().set(SelectionRef::chassis()); }));
    step(hasMechanisms, ETR("Add your mechanisms"), ETR("Double-click a template in the palette (intake, shooter, elevator...), then place it with the gizmo."));
    step(hasDevices, ETR("Detect devices"),
         ETR("Click \"Detect devices\" and run your robot code: motors, encoders and sensors appear as the code creates them."),
         hasDevices ? nullptr : linkButton(ETR("Detect"), [this] { controller_.toggleDetection(); }));
    step(hasMechanisms && hasDevices && unfilled == 0, ETR("Map devices to slots"),
         unfilled > 0 ? QCoreApplication::translate("Editor", "%n required slot(s) still empty. Drag a detected device onto it.", nullptr, unfilled)
                      : ETR("Drag detected devices onto mechanism slots or wheel drive/steer slots."));
    addSection(ETR("Getting started"), body);
}

void InspectorPanel::addSlots(const SelectionRef& target) {
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    std::string owner = target.owner;
    std::string prefix;
    if (target.kind == SelKind::Wheel) {
        owner = sdk::slot_owner::kDrivetrain;
        prefix = target.sub + ".";
    }
    const auto infos = slotInfos(c, cat, controller_.devices(), owner);
    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(2);
    bool any = false;
    for (const SlotInfo& info : infos) {
        if (!prefix.empty() && info.spec.name.rfind(prefix, 0) != 0) continue;
        any = true;
        auto* row = new SlotRowWidget(icons_, body);
        row->setInfo(info, controller_.devices());
        connect(row, &SlotRowWidget::deviceChosen, this, [this](const sdk::SlotAddress& slot, const sdk::DeviceKey& key, quint32 caps) {
            const QString err = controller_.mapDevice(SelectionRef::slot(slot.owner, slot.slot), key, sdk::CapabilitySet{caps});
            if (!err.isEmpty()) Q_EMIT controller_.statusMessage(err, 1);
        });
        connect(row, &SlotRowWidget::clearRequested, this, [this](const sdk::SlotAddress& slot) { controller_.clearSlot(slot); });
        v->addWidget(row);
        slotRows_.push_back(row);
    }
    if (!any) v->addWidget(mutedLabel(ETR("This item has no device slots."), body));
    else v->addWidget(mutedLabel(ETR("Drag a device from \"Detected devices\" onto a slot, or pick one from the list."), body));
    addSection(ETR("Device slots"), body);
    if (shown_.kind == SelKind::Slot || controller_.selection().current().kind == SelKind::Slot) {
        const SelectionRef cur = controller_.selection().current();
        for (auto* row : slotRows_)
            if (row->address().owner == cur.owner && row->address().slot == cur.sub) QTimer::singleShot(0, row, [row] { row->flash(); });
    }
}

void InspectorPanel::addShapes(const std::string& owner) {
    const sdk::RobotConfig& c = controller_.document().config();
    const auto* list = ops::primitivesOf(c, owner);
    if (list == nullptr) return;
    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(2);
    if (list->empty()) v->addWidget(mutedLabel(ETR("No shapes. Add boxes and cylinders to model the visible and colliding parts."), body));
    for (std::size_t i = 0; i < list->size(); ++i) {
        const auto& p = (*list)[i];
        const int index = static_cast<int>(i);
        auto* b = static_cast<QToolButton*>(linkButton(QStringLiteral("%1  \xC2\xB7  %2").arg(qs(p.name), p.kind == sdk::PrimitiveKind::Box ? ETR("box") : ETR("cylinder")),
                                                       [this, owner, index] { controller_.selection().set(SelectionRef::primitive(owner, index)); }));
        b->setToolButtonStyle(Qt::ToolButtonTextOnly);
        v->addWidget(b);
    }
    auto* row = new QHBoxLayout;
    for (auto kind : {sdk::PrimitiveKind::Box, sdk::PrimitiveKind::Cylinder}) {
        auto* b = new QToolButton(body);
        b->setText(kind == sdk::PrimitiveKind::Box ? ETR("+ Box") : ETR("+ Cylinder"));
        b->setAutoRaise(true);
        connect(b, &QToolButton::clicked, this, [this, owner, kind] {
            const int index = controller_.document().addPrimitive(qs(owner), kind);
            if (index >= 0) controller_.selection().set(SelectionRef::primitive(owner, index));
        });
        row->addWidget(b);
    }
    row->addStretch();
    v->addLayout(row);
    addSection(ETR("Shapes (%1)").arg(list->size()), body, list->empty() ? false : owner != ops::kChassisOwner);
}

void InspectorPanel::addWheelList() {
    const sdk::RobotConfig& c = controller_.document().config();
    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(2);
    for (const auto& w : c.drivetrain.spec.wheels) {
        const std::string name = w.name;
        v->addWidget(linkButton(QStringLiteral("%1  \xC2\xB7  %2").arg(qs(w.name), qs(w.driveMotor.name)),
                                [this, name] { controller_.selection().set(SelectionRef::wheel(name)); }));
    }
    auto* add = new QToolButton(body);
    add->setText(ETR("+ Add wheel"));
    add->setAutoRaise(true);
    connect(add, &QToolButton::clicked, this, [this] {
        const QString name = controller_.document().addWheel();
        if (!name.isEmpty()) controller_.selection().set(SelectionRef::wheel(name.toStdString()));
    });
    v->addWidget(add, 0, Qt::AlignLeft);
    addSection(ETR("Wheels (%1)").arg(c.drivetrain.spec.wheels.size()), body);
}

void InspectorPanel::addChildren(const std::string& mechanismId) {
    const sdk::RobotConfig& c = controller_.document().config();
    const auto children = ops::childrenOf(c, mechanismId);
    if (children.empty()) return;
    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(2);
    for (const auto& id : children) {
        const sdk::MechanismConfig* m = c.findMechanism(id);
        v->addWidget(linkButton(qs(ops::displayNameFor(*m, controller_.document().catalog())) + QStringLiteral("  @ ") + qs(m->resolvedAttachment().parentLink),
                                [this, id] { controller_.selection().set(SelectionRef::mechanism(id)); }));
    }
    addSection(ETR("Attached mechanisms (%1)").arg(children.size()), body);
}

void InspectorPanel::addDeviceCard(const SelectionRef& sel) {
    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    const sdk::DiscoveredDevice* found = nullptr;
    for (const auto& d : controller_.devices().all())
        if (d.descriptor.physicalId().toString() == sel.owner) found = &d;
    if (found == nullptr) {
        v->addWidget(mutedLabel(ETR("This device was not detected."), body));
    } else {
        const auto& d = found->descriptor;
        const DeviceStatus st = deviceStatus(controller_.document().config(), *found);
        auto* label = new QLabel(QStringLiteral("<b>%1</b><br>%2 / %3 \xC2\xB7 id %4 \xC2\xB7 bus %5<br>%6")
                                     .arg(deviceLabel(d).toHtmlEscaped(), qs(d.family).toHtmlEscaped(), qs(d.type).toHtmlEscaped())
                                     .arg(d.id)
                                     .arg(busLabel(found->observedBus.empty() ? d.bus : found->observedBus).toHtmlEscaped(),
                                          capabilityText(d.capabilities).toHtmlEscaped()),
                                 body);
        label->setWordWrap(true);
        v->addWidget(label);
        QString usage = st.mapped() ? ETR("Mapped to:") : ETR("Not mapped: it runs in loopback (outputs are echoed back as inputs).");
        for (const auto& s : st.usedBy) usage += QStringLiteral("\n \xE2\x80\xA2 ") + qs(ops::slotDisplayName(controller_.document().config(), controller_.document().catalog(), s));
        v->addWidget(mutedLabel(usage, body));
    }
    addSection(ETR("Device"), body);
}

QMap<QString, QPair<int, QString>> InspectorPanel::issuesFor(const SelectionRef& target) const {
    QMap<QString, QPair<int, QString>> out;
    const sdk::RobotConfig& c = controller_.document().config();
    for (const auto& issue : controller_.issues()) {
        const IssueLocation loc = locateIssue(c, controller_.document().catalog(), issue);
        if (loc.fieldKey.empty() || formTargetOf(c, loc.selection) != target) continue;
        const QString key = qs(loc.fieldKey);
        const int sev = static_cast<int>(issue.severity);
        if (!out.contains(key) || out[key].first < sev) out[key] = {sev, qs(issue.message)};
    }
    return out;
}

QString InspectorPanel::signature(const std::vector<PropertyGroup>& groups) const {
    const sdk::RobotConfig& c = controller_.document().config();
    const SelectionRef sel = controller_.selection().current();
    const SelectionRef target = formTargetOf(c, sel);
    QStringList parts{qs(sel.key()), QString::number(static_cast<int>(controller_.settings().units()))};
    for (const auto& link : c.motionLinks)
        parts << qs(link.id) << qs(link.source.toString()) << qs(link.target.toString())
              << QString::number(link.ratio, 'g', 17);
    for (const auto& m : c.mechanisms) parts << qs(m.id) << qs(m.type);
    for (const auto& axis : sdk::motionAxisInfos(c, [&](std::string_view type) {
             return controller_.document().catalog().findTemplate(type);
         }))
        parts << qs(axis.address.toString()) << QString::number(static_cast<int>(axis.kind));
    for (const auto& g : groups) {
        parts << qs(g.id) << g.title;
        for (const auto& p : g.schema.properties) {
            parts << qs(p.key) << QString::number(p.options.size());
            for (const auto& o : p.options) parts << qs(o.value);
        }
    }
    switch (target.kind) {
        case SelKind::Wheel:
        case SelKind::Mechanism:
            for (const auto& s : ops::slotSpecsFor(c, controller_.document().catalog(), target.kind == SelKind::Wheel ? std::string(sdk::slot_owner::kDrivetrain) : target.owner))
                parts << qs(s.name);
            break;
        default: break;
    }
    if (target.kind == SelKind::Chassis || target.kind == SelKind::Mechanism)
        if (const auto* list = ops::primitivesOf(c, target.kind == SelKind::Chassis ? std::string(ops::kChassisOwner) : target.owner))
            for (const auto& p : *list) parts << qs(p.name) << (p.kind == sdk::PrimitiveKind::Box ? "b" : "c");
    if (target.kind == SelKind::Mechanism)
        for (const auto& id : ops::childrenOf(c, target.owner)) parts << qs(id);
    if (target.kind == SelKind::Drivetrain)
        for (const auto& w : c.drivetrain.spec.wheels) parts << qs(w.name);
    if (target.kind == SelKind::Robot || target.kind == SelKind::None) {
        parts << QString::number(c.mechanisms.size()) << QString::number(controller_.devices().all().size());
        for (const auto& owner : ops::slotOwners(c))
            for (const auto& info : slotInfos(c, controller_.document().catalog(), controller_.devices(), owner))
                parts << QString::number(static_cast<int>(info.state));
    }
    if (target.kind == SelKind::Device) parts << QString::number(controller_.devices().all().size());
    return parts.join('\x1f');
}

void InspectorPanel::rebuild() {
    clearContent();
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    const SelectionRef sel = controller_.selection().current();
    const SelectionRef target = sel.kind == SelKind::Device ? sel : formTargetOf(c, sel);
    const std::vector<PropertyGroup> groups = sel.kind == SelKind::Device ? std::vector<PropertyGroup>{} : propertyGroupsFor(c, cat, sel);

    content_ = new QWidget;
    layout_ = new QVBoxLayout(content_);
    layout_->setContentsMargins(14, 12, 14, 12);
    layout_->setSpacing(14);
    shown_ = sel;
    shownUnits_ = controller_.settings().units();

    addHeader(target, groups);
    switch (target.kind) {
        case SelKind::None:
        case SelKind::Robot: addChecklist(); break;
        case SelKind::Wheel:
        case SelKind::Mechanism: addSlots(target); break;
        case SelKind::Device: addDeviceCard(target); break;
        case SelKind::Drivetrain: addWheelList(); break;
        default: break;
    }
    const auto issues = issuesFor(target);
    for (const PropertyGroup& g : groups) {
        auto* f = new PropertyFormWidget(g, controller_.settings().units(), content_);
        f->setFieldIssues(issues);
        connect(f, &PropertyFormWidget::valueEdited, this, [this](const QString& groupId, const QString& key, const sdk::Value& v) {
            controller_.document().setProperty(controller_.selection().current(), groupId.toStdString(), key.toStdString(), v);
        });
        forms_.push_back(f);
        if (groups.size() == 1) layout_->addWidget(f);   // one group: the page header already names it
        else addSection(g.title, f, g.collapsed);
    }
    if (target.kind == SelKind::Chassis) addShapes(ops::kChassisOwner);
    if (target.kind == SelKind::Robot || target.kind == SelKind::None ||
        target.kind == SelKind::Drivetrain || target.kind == SelKind::Wheel ||
        target.kind == SelKind::Mechanism)
        addSection(ETR("Motion links"), new MotionLinksPanel(controller_, content_));
    if (target.kind == SelKind::Mechanism) {
        addShapes(target.owner);
        addChildren(target.owner);
    }
    layout_->addStretch();
    scroll_->setWidget(content_);
    signature_ = signature(groups);
    ++rebuilds_;
    if (!pendingFocus_.isEmpty()) {
        const QString key = pendingFocus_;
        pendingFocus_.clear();
        QTimer::singleShot(0, this, [this, key] { focusField(key); });
    }
}

void InspectorPanel::refresh(bool authoritative) {
    const SelectionRef sel = controller_.selection().current();
    if (!(sel == shown_)) return;   // the selection signal rebuilds
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    const SelectionRef target = sel.kind == SelKind::Device ? sel : formTargetOf(c, sel);
    const std::vector<PropertyGroup> groups = sel.kind == SelKind::Device ? std::vector<PropertyGroup>{} : propertyGroupsFor(c, cat, sel);
    if (signature(groups) != signature_) {
        rebuild();
        return;
    }
    const auto issues = issuesFor(target);
    for (const PropertyGroup& g : groups)
        if (auto* f = form(qs(g.id))) {
            f->setValues(g.values, authoritative);
            f->setFieldIssues(issues);
        }
    std::map<std::string, std::vector<SlotInfo>> byOwner;
    for (SlotRowWidget* row : slotRows_) {
        const std::string owner = row->address().owner;
        if (!byOwner.count(owner)) byOwner[owner] = slotInfos(c, cat, controller_.devices(), owner);
        for (const SlotInfo& info : byOwner[owner])
            if (info.address == row->address()) row->setInfo(info, controller_.devices());
    }
    if (title_ != nullptr && target.kind != SelKind::Device) title_->setText(describeSelection(c, cat, target));
}

void InspectorPanel::focusField(const QString& key) {
    if (key == QStringLiteral("motionLinks")) {
        if (auto* field = content_->findChild<QWidget*>(QStringLiteral("motionLinks.source"))) {
            scroll_->ensureWidgetVisible(field);
            field->setFocus();
        }
        return;
    }
    for (auto* f : forms_)
        if (f->focusField(key)) return;
    pendingFocus_.clear();
}

}  // namespace wpiemu::app::editor
