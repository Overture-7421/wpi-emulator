#include "views/inspector_panel.h"

#include <QApplication>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <functional>
#include <map>

#include "model/config_ops.h"
#include "model/motion_links.h"
#include "model/property_groups.h"
#include "model/tr.h"
#include "model/units.h"
#include "views/activation_card.h"
#include "views/drive_card.h"

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
    activationCard_ = nullptr;
    startPoseHint_ = nullptr;
    startPoseReset_ = nullptr;
    title_ = nullptr;
    nameEdit_ = nullptr;
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

QWidget* InspectorPanel::addSection(const QString& title, QWidget* body, bool collapsed, SectionRole role) {
    auto* card = new SectionCard(title, role, body, collapsed, content_);
    layout_->addWidget(card);
    return card;
}

PropertyFormWidget* InspectorPanel::makeForm(const PropertyGroup& group, QWidget* parent) {
    auto* f = new PropertyFormWidget(group, controller_.settings().units(), parent);
    f->setFieldIssues(issues_);
    connect(f, &PropertyFormWidget::valueEdited, this, [this](const QString& groupId, const QString& key, const sdk::Value& v) {
        controller_.document().setProperty(controller_.selection().current(), groupId.toStdString(), key.toStdString(), v);
    });
    forms_.push_back(f);
    return f;
}

DeviceRowWidget* InspectorPanel::makeRow(const SlotInfo& info, QWidget* parent) {
    auto* row = new DeviceRowWidget(icons_, parent);
    row->setInfo(info);
    connect(row, &DeviceRowWidget::clearRequested, this, [this](const sdk::SlotAddress& slot) { controller_.clearSlot(slot); });
    connect(row, &DeviceRowWidget::reversedToggled, this,
            [this](const sdk::SlotAddress& slot, bool reversed) { controller_.setMotorReversed(slot, reversed); });
    slotRows_.push_back(row);
    return row;
}

QString InspectorPanel::editableName(const SelectionRef& target) const {
    const sdk::RobotConfig& c = controller_.document().config();
    switch (target.kind) {
        case SelKind::Wheel: return qs(target.sub);
        case SelKind::Primitive:
            if (const auto* list = ops::primitivesOf(c, target.owner); list != nullptr && target.index >= 0 && target.index < static_cast<int>(list->size()))
                return qs((*list)[static_cast<std::size_t>(target.index)].name);
            return {};
        default: return describeSelection(c, controller_.document().catalog(), target);
    }
}

void InspectorPanel::commitName(const SelectionRef& target, const QString& text) {
    const QString name = text.trimmed();
    if (name.isEmpty() || name == editableName(target)) {
        if (nameEdit_ != nullptr) nameEdit_->setText(editableName(target));
        return;
    }
    const std::string value = name.toStdString();
    switch (target.kind) {
        case SelKind::Mechanism: controller_.document().setProperty(target, "mech.general", "label", sdk::Value(value)); break;
        case SelKind::Primitive: controller_.document().setProperty(target, "primitive", "name", sdk::Value(value)); break;
        case SelKind::Wheel:
            controller_.document().setProperty(target, "wheel", "name", sdk::Value(value));
            if (ops::findWheel(controller_.document().config(), value) != nullptr) controller_.selection().set(SelectionRef::wheel(value));
            break;
        default: break;
    }
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
                // The Type (Rotary / Arm / Linear) is edited in Configure; only a plugin template needs naming here.
                if (const auto* info = cat.findTemplate(m->type); info != nullptr && info->origin != "core") subtitle = qs(info->displayName);
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
        case SelKind::Zone:
            if (const auto* z = c.findZone(target.owner)) {
                const auto* kind = cat.findZoneKind(z->kind);
                title = kind != nullptr ? qs(kind->displayName) : ETR("Zone");
                subtitle = kind != nullptr ? qs(kind->description) : qs(z->kind) + QStringLiteral(" ") + ETR("(unknown zone kind)");
            }
            icon = NodeKind::Zone;
            break;
        case SelKind::Device: title = ETR("Detected device"); icon = NodeKind::Robot; break;
        case SelKind::StartPose: subtitle = ETR("Where the robot starts and where Reset puts it back"); icon = NodeKind::Robot; break;
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
    const bool editable = target.kind == SelKind::Mechanism || target.kind == SelKind::Wheel || target.kind == SelKind::Primitive;
    QFont f = head->font();
    f.setPointSizeF(f.pointSizeF() * 1.25);
    f.setBold(true);
    if (editable) {
        nameEdit_ = new QLineEdit(editableName(target), head);
        nameEdit_->setObjectName(QStringLiteral("inspector.name"));
        nameEdit_->setFont(f);
        nameEdit_->setToolTip(ETR("Name"));
        nameEdit_->setPlaceholderText(ETR("Name"));
        connect(nameEdit_, &QLineEdit::editingFinished, this, [this, target] {
            if (nameEdit_ != nullptr) commitName(target, nameEdit_->text());
        });
        texts->addWidget(nameEdit_);
    } else {
        title_ = new QLabel(title, head);
        title_->setFont(f);
        title_->setWordWrap(true);
        texts->addWidget(title_);
    }
    if (!subtitle.isEmpty()) texts->addWidget(mutedLabel(subtitle, head));
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
    if (target.kind == SelKind::Mechanism || target.kind == SelKind::Primitive || target.kind == SelKind::Zone)
        addAction(ETR("Duplicate"), ETR("Duplicate (Ctrl+D)"), [this] { controller_.duplicateSelection(); });
    if (target.kind == SelKind::Mechanism || target.kind == SelKind::Primitive || target.kind == SelKind::Wheel || target.kind == SelKind::Zone)
        addAction(ETR("Delete"), ETR("Delete (Del)"), [this] { controller_.deleteSelection(); });
    layout_->addWidget(head);
}

void InspectorPanel::addStartPoseCard(const PropertyGroup& group) {
    auto* body = new QWidget(content_);
    auto* v = new QVBoxLayout(body);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(6);
    v->addWidget(makeForm(group, body));
    startPoseHint_ = mutedLabel(QString(), body);
    startPoseHint_->setObjectName(QStringLiteral("startPose.hint"));
    v->addWidget(startPoseHint_);
    startPoseReset_ = static_cast<QToolButton*>(linkButton(ETR("Use default placement"), [this] { controller_.document().clearStartPose(); }));
    startPoseReset_->setObjectName(QStringLiteral("startPose.useDefault"));
    startPoseReset_->setToolTip(ETR("Forget the start pose: the robot starts where the game field puts it."));
    v->addWidget(startPoseReset_, 0, Qt::AlignLeft);
    addSection(ETR("Start pose"), body, false, SectionRole::Placement);
    refreshStartPoseCard();
}

void InspectorPanel::refreshStartPoseCard() {
    if (startPoseHint_ == nullptr || startPoseReset_ == nullptr) return;
    const bool set = controller_.document().config().sim.startPose.has_value();
    const sdk::StartPose def = controller_.document().catalog().defaultStartPose;
    const UnitSystem units = controller_.settings().units();
    const auto length = [units](float m) {
        const UnitDisplay d = unitDisplay(sdk::PropertyUnit::Meters, units);
        return QString::number(toDisplay(static_cast<double>(m), sdk::PropertyUnit::Meters, units), 'f', d.decimals) + unitSuffix(sdk::PropertyUnit::Meters, units);
    };
    startPoseHint_->setText(set ? ETR("The robot starts here and returns here on Reset. Type values or drag the footprint in the viewport.")
                                : ETR("Not set: the robot starts at the default placement (X %1, Y %2). Edit a value or drag the footprint to set it.")
                                      .arg(length(def.xM), length(def.yM)));
    startPoseReset_->setEnabled(set);
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
    addSection(ETR("Shapes (%1)").arg(list->size()), body, list->empty() ? false : owner != ops::kChassisOwner, SectionRole::Shapes);
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
        v->addWidget(linkButton(qs(ops::displayNameFor(*m, controller_.document().catalog())) + QStringLiteral("  @ ") + qs(m->attachment.parentLink),
                                [this, id] { controller_.selection().set(SelectionRef::mechanism(id)); }));
    }
    addSection(ETR("Attached mechanisms (%1)").arg(children.size()), body, false, SectionRole::Shapes);
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
        QString usage = st.mapped() ? ETR("Mapped to:")
                        : !sdk::isMappable(d) ? ETR("Built into the robot controller: there is nothing to map it to.")
                                              : ETR("Not mapped: it runs in loopback (outputs are echoed back as inputs).");
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
            for (const auto& s : slotInfos(c, controller_.document().catalog(), controller_.devices(), target.kind == SelKind::Wheel ? std::string(sdk::slot_owner::kDrivetrain) : target.owner))
                parts << qs(s.spec.name);
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
    if (target.kind == SelKind::Zone)
        if (const auto* z = c.findZone(target.owner)) parts << qs(z->kind);
    return parts.join('\x1f');
}

void InspectorPanel::addPartCards(const SelectionRef& target, const std::vector<PropertyGroup>& groups) {
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    const bool wheel = target.kind == SelKind::Wheel;
    const std::string owner = wheel ? std::string(sdk::slot_owner::kDrivetrain) : target.owner;

    // 1. Drive: how it is actuated.
    if (DriveCard::applicable(controller_, target)) {
        DriveCard::Registrar registrar;
        registrar.form = [this](PropertyFormWidget* f) {
            f->setFieldIssues(issues_);
            forms_.push_back(f);
        };
        registrar.row = [this](DeviceRowWidget* r) { slotRows_.push_back(r); };
        addSection(ETR("Drive"), new DriveCard(controller_, icons_, target, registrar, content_), false, SectionRole::Drive);
    }

    // 2. Configure: what it is and how it is simulated, then its sensors.
    auto* configure = new QWidget(content_);
    auto* cv = new QVBoxLayout(configure);
    cv->setContentsMargins(0, 0, 0, 0);
    cv->setSpacing(8);
    for (const PropertyGroup& g : groups)
        if (g.id == "mech.props" || g.id == "wheel") cv->addWidget(makeForm(g, configure));
    // A swerve module steers: its steer controller sits with the rest of what makes the wheel a swerve module.
    if (wheel)
        if (const auto* w = ops::findWheel(c, target.sub); w != nullptr && w->steered)
            for (const SlotInfo& info : slotInfos(c, cat, controller_.devices(), owner))
                if (info.address.slot == target.sub + ".steer") {
                    auto* row = makeRow(info, configure);
                    row->setLabel(ETR("Steer motor"));
                    cv->addWidget(row);
                }
    std::vector<SlotInfo> sensors;
    for (const SlotInfo& info : slotInfos(c, cat, controller_.devices(), owner))
        if (info.spec.type != sdk::SlotType::Motor && (!wheel || info.spec.name.rfind(target.sub + ".", 0) == 0)) sensors.push_back(info);
    if (!sensors.empty()) {
        auto* heading = new QLabel(ETR("SENSORS"), configure);
        QFont hf = heading->font();
        hf.setBold(true);
        hf.setPointSizeF(hf.pointSizeF() * 0.8);
        heading->setFont(hf);
        heading->setEnabled(false);
        cv->addWidget(heading);
        for (const SlotInfo& info : sensors) cv->addWidget(makeRow(info, configure));
    }
    for (const PropertyGroup& g : groups)
        if (g.id.rfind("panel:", 0) == 0) {
            auto* heading = new QLabel(QStringLiteral("<b>%1</b>").arg(g.title.toHtmlEscaped()), configure);
            cv->addWidget(heading);
            cv->addWidget(makeForm(g, configure));
        }
    addSection(ETR("Configure"), configure, false, SectionRole::Configure);

    // 3. Placement.
    for (const PropertyGroup& g : groups)
        if (g.section == sdk::PropertySection::Placement) addSection(ETR("Placement"), makeForm(g, content_), false, SectionRole::Placement);
}
void InspectorPanel::addZoneCards(const SelectionRef& target, const std::vector<PropertyGroup>& groups) {
    auto* general = new QWidget(content_);
    auto* gv = new QVBoxLayout(general);
    gv->setContentsMargins(0, 0, 0, 0);
    gv->setSpacing(6);
    for (const PropertyGroup& g : groups)
        if (g.id == "zone.general") gv->addWidget(makeForm(g, general));
    auto* id = new QLabel(ETR("ID: %1").arg(QStringLiteral("<b>%1</b>").arg(qs(target.owner).toHtmlEscaped())), general);
    id->setObjectName(QStringLiteral("zone.id"));
    id->setTextFormat(Qt::RichText);
    id->setToolTip(ETR("Used by the game plugin and in robot.json; it cannot be changed."));
    id->setEnabled(false);
    gv->addWidget(id);
    addSection(ETR("General"), general, false, SectionRole::Info);

    for (const PropertyGroup& g : groups)
        if (g.section == sdk::PropertySection::Placement) addSection(ETR("Placement"), makeForm(g, content_), false, SectionRole::Placement);

    activationCard_ = new ActivationCard(controller_, target.owner, content_);
    activationCard_->setIssues(issues_);
    addSection(ETR("Activation"), activationCard_, false, SectionRole::Activation);

    for (const PropertyGroup& g : groups)
        if (g.id == "zone.props") addSection(ETR("Configure"), makeForm(g, content_), false, SectionRole::Configure);
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
        case SelKind::Device: addDeviceCard(target); break;
        default: break;
    }
    issues_ = issuesFor(target);
    if (target.kind == SelKind::Wheel || target.kind == SelKind::Mechanism) {
        addPartCards(target, groups);
    } else if (target.kind == SelKind::Zone) {
        addZoneCards(target, groups);
    } else {
        const auto formGroups = std::count_if(groups.begin(), groups.end(), [](const PropertyGroup& g) { return g.id != "robot.start"; });
        for (const PropertyGroup& g : groups) {
            if (g.id == "robot.start") {
                addStartPoseCard(g);
                continue;
            }
            auto* f = makeForm(g, content_);
            addSection(formGroups == 1 ? ETR("Configure") : g.title, f, g.collapsed, SectionRole::Configure);
        }
    }
    if (target.kind == SelKind::Chassis) addShapes(ops::kChassisOwner);
    if (target.kind == SelKind::Mechanism) {
        addShapes(target.owner);
        addChildren(target.owner);
    }
    layout_->addStretch();
    scroll_->setWidget(content_);
    signature_ = signature(groups);
    ++rebuilds_;
    if (sel.kind == SelKind::Slot)
        for (auto* row : slotRows_)
            if (row->address().owner == sel.owner && row->address().slot == sel.sub) QTimer::singleShot(0, row, [row] { row->flash(); });
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
    if (activationCard_ != nullptr) {
        activationCard_->setIssues(issues);
        activationCard_->refresh(authoritative);
    }
    refreshStartPoseCard();
    std::map<std::string, std::vector<SlotInfo>> byOwner;
    for (DeviceRowWidget* row : slotRows_) {
        const std::string owner = row->address().owner;
        if (!byOwner.count(owner)) byOwner[owner] = slotInfos(c, cat, controller_.devices(), owner);
        for (const SlotInfo& info : byOwner[owner])
            if (info.address == row->address()) row->setInfo(info);
    }
    if (title_ != nullptr && target.kind != SelKind::Device && target.kind != SelKind::Zone) title_->setText(describeSelection(c, cat, target));
    if (nameEdit_ != nullptr && !nameEdit_->hasFocus()) nameEdit_->setText(editableName(target));
}

void InspectorPanel::focusField(const QString& key) {
    if (key.startsWith(QStringLiteral("activation"))) {
        if (activationCard_ != nullptr) {
            const int colon = key.indexOf(QLatin1Char(':'));
            activationCard_->focusCondition(colon < 0 ? 0 : key.mid(colon + 1).toInt());
        }
        return;
    }
    if (key == QStringLiteral("drivenBy") || key == QStringLiteral("motionLinks")) {
        for (auto* combo : content_->findChildren<QComboBox*>())
            if (combo->objectName().startsWith(QStringLiteral("drivenBy.source"))) {
                if (!combo->isVisible())
                    if (auto* linked = content_->findChild<QToolButton*>(QStringLiteral("drive.linked:") + combo->objectName().mid(QStringLiteral("drivenBy.source:").size()))) linked->click();
                scroll_->ensureWidgetVisible(combo);
                combo->setFocus();
                break;
            }
        return;
    }
    for (auto* f : forms_)
        if (f->focusField(key)) return;
    pendingFocus_.clear();
}

}  // namespace wpiemu::app::editor
