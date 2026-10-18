#include "viewmodel/project_tree_model.h"

#include <QMimeData>

#include <algorithm>
#include <functional>
#include <map>
#include <set>

#include "model/config_ops.h"
#include "model/motion_links.h"
#include "model/property_groups.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

bool needsAttention(SlotState s) {
    return s == SlotState::Unmapped || s == SlotState::Incompatible || s == SlotState::AliasConflict;
}

}  // namespace

ProjectTreeModel::ProjectTreeModel(EditorDocument& document, const DeviceDirectory& devices, QObject* parent)
    : QAbstractItemModel(parent), document_(document), devices_(devices) {
    root_ = build();
}

std::unique_ptr<ProjectTreeModel::Node> ProjectTreeModel::build() const {
    const sdk::RobotConfig& c = document_.config();
    const EditorCatalog& cat = document_.catalog();
    auto root = std::make_unique<Node>();
    root->key = QStringLiteral("root");

    auto add = [](Node& parent, NodeKind kind, QString key, SelectionRef ref, QString text, QString detail = {}) -> Node& {
        auto n = std::make_unique<Node>();
        n->kind = kind;
        n->key = std::move(key);
        n->ref = std::move(ref);
        n->text = std::move(text);
        n->detail = std::move(detail);
        n->parent = &parent;
        parent.children.push_back(std::move(n));
        return *parent.children.back();
    };
    auto addSlots = [&](Node& parent, std::string_view owner) {
        bool warn = false;
        for (const SlotInfo& info : slotInfos(c, cat, devices_, owner)) {
            const QString label = qs(info.spec.label.empty() ? info.spec.name : info.spec.label);
            QString detail;
            if (info.mapped) detail = mappedLabel(info);
            else detail = info.spec.required ? ETR("not mapped") : ETR("optional");
            Node& n = add(parent, NodeKind::Slot, QStringLiteral("slot:%1:%2").arg(qs(std::string(owner)), qs(info.spec.name)),
                          SelectionRef::slot(std::string(owner), info.spec.name), label, detail);
            n.state = static_cast<int>(info.state);
            n.tooltip = qs(info.spec.description) + (info.spec.description.empty() ? QString() : QStringLiteral("\n")) + info.detail;
            warn = warn || needsAttention(info.state);
        }
        return warn;
    };

    Node& robot = add(*root, NodeKind::Robot, QStringLiteral("robot"), SelectionRef::robot(), qs(c.name),
                      qs(c.drivetrain.spec.kind) + QStringLiteral(" \xC2\xB7 %1 kg").arg(static_cast<double>(c.chassis.massKg), 0, 'f', 1));

    Node& chassis = add(robot, NodeKind::Chassis, QStringLiteral("chassis"), SelectionRef::chassis(), ETR("Chassis"));
    add(chassis, NodeKind::Com, QStringLiteral("com"), SelectionRef::com(), ETR("Centre of mass"));
    for (std::size_t i = 0; i < c.chassis.primitives.size(); ++i)
        add(chassis, NodeKind::Shape, QStringLiteral("chassis-shape:%1").arg(i), SelectionRef::primitive(ops::kChassisOwner, static_cast<int>(i)),
            qs(c.chassis.primitives[i].name),
            c.chassis.primitives[i].kind == sdk::PrimitiveKind::Box ? ETR("box") : ETR("cylinder"));

    Node& drive = add(chassis, NodeKind::Drivetrain, QStringLiteral("drivetrain"), SelectionRef::drivetrain(), ETR("Drivetrain"),
                      qs(c.drivetrain.spec.kind));
    Node& mechs = add(robot, NodeKind::Group, QStringLiteral("mechanisms"), SelectionRef{}, ETR("Mechanisms"),
                      QString::number(c.mechanisms.size()));
    Node& sensors = add(robot, NodeKind::Group, QStringLiteral("sensors"), SelectionRef{}, ETR("Robot sensors"));
    addSlots(sensors, sdk::slot_owner::kRobot);

    if (cat.hasPlugin()) {
        Node& plugin = add(robot, NodeKind::Plugin, QStringLiteral("plugin"), SelectionRef::plugin(), qs(cat.pluginDisplayName), ETR("game rules"));
        for (const auto& vol : ops::volumesFor(c, cat, ""))
            add(plugin, NodeKind::Volume, QStringLiteral("volume::%1").arg(qs(vol.key)), SelectionRef::overlay({}, vol.key), qs(vol.label), ETR("volume"));
        for (const auto& z : c.zones) {
            const auto att = z.attachment;
            const QString where = att.onChassis() ? QStringLiteral("chassis")
                                                  : qs(att.parentMechanismId + (att.parentLink == sdk::kRootLink ? "" : "." + att.parentLink));
            Node& zn = add(plugin, NodeKind::Zone, QStringLiteral("zone:%1").arg(qs(z.id)), SelectionRef::zone(z.id), qs(ops::zoneDisplayName(z, cat)),
                           QStringLiteral("@ ") + where);
            zn.warning = zonesWithIssues_.count(z.id) != 0;
            if (const auto* kind = cat.findZoneKind(z.kind)) zn.tooltip = qs(kind->displayName) + QStringLiteral(" \xE2\x80\x94 ") + zn.detail;
        }
    }

    // Wheels and mechanisms are created detached, then placed: a follower sits under the owner that drives it,
    // everything else under its structural parent (drivetrain, mechanism attachment, mechanisms group).
    const MotionIndex motion(c, cat);
    struct Owner {
        std::unique_ptr<Node> node;
        QString structuralKey;   // key of the structural parent (an owner or a group)
        QString linkKey;         // key of the owner that drives it; empty when it is not a follower
    };
    std::vector<Owner> owners;
    auto ownerKey = [](const SelectionRef& ref) {
        return ref.kind == SelKind::Wheel ? QStringLiteral("wheel:%1").arg(qs(ref.sub)) : QStringLiteral("mech:%1").arg(qs(ref.owner));
    };
    for (const auto& w : c.drivetrain.spec.wheels) {
        Owner o;
        o.node = makeWheelNode(c, w, motion);
        o.structuralKey = drive.key;
        o.linkKey = linkKeyFor(motion, o.node->ref, ownerKey);
        owners.push_back(std::move(o));
    }
    for (const auto& m : c.mechanisms) {
        Owner o;
        o.node = makeMechanismNode(c, m, motion);
        const auto att = m.attachment;
        o.structuralKey = att.onChassis() || c.findMechanism(att.parentMechanismId) == nullptr ? mechs.key : QStringLiteral("mech:%1").arg(qs(att.parentMechanismId));
        o.linkKey = linkKeyFor(motion, o.node->ref, ownerKey);
        owners.push_back(std::move(o));
    }

    std::map<QString, Node*> byKey;
    std::map<QString, std::size_t> index;
    for (std::size_t i = 0; i < owners.size(); ++i) {
        byKey[owners[i].node->key] = owners[i].node.get();
        index[owners[i].node->key] = i;
    }
    byKey[drive.key] = &drive;
    byKey[mechs.key] = &mechs;
    for (auto& o : owners)
        if (!index.count(o.linkKey)) o.linkKey.clear();   // the source is not a tree node (broken model)

    std::map<QString, QString> parentOf;
    for (const auto& o : owners) parentOf[o.node->key] = o.linkKey.isEmpty() ? o.structuralKey : o.linkKey;
    // A link may point back along the structural chain; the owner that closes such a loop keeps its structural parent.
    for (const auto& o : owners) {
        QString k = o.node->key;
        std::set<QString> seen;
        while (index.count(k) && seen.insert(k).second) k = parentOf[k];
        if (k == o.node->key) parentOf[o.node->key] = o.structuralKey;
    }
    for (auto& o : owners) {
        Node* parent = byKey[parentOf[o.node->key]];
        o.node->parent = parent;
        o.node->nestedByLink = parent->key != o.structuralKey;
        parent->children.push_back(std::move(o.node));
    }
    return root;
}

QString ProjectTreeModel::linkKeyFor(const MotionIndex& motion, const SelectionRef& owner, const std::function<QString(const SelectionRef&)>& key) {
    const OwnerMotion om = motion.ownerMotion(owner);
    return om.primarySource ? key(*om.primarySource) : QString();
}

QString ProjectTreeModel::motionDetail(const sdk::RobotConfig& c, const EditorCatalog& cat, const MotionIndex& motion, const OwnerMotion& om) {
    QStringList parts;
    if (!om.incoming.empty()) {
        const sdk::MotionLink& link = *om.incoming.front();
        const sdk::MotionAxisInfo* from = motion.axis(link.source);
        const sdk::MotionAxisInfo* to = motion.axis(link.target);
        const double ratio = from != nullptr && to != nullptr ? ratioFromSi(link.ratio, from->kind, to->kind) : link.ratio;
        QString text = ETR("follows %1 \xC3\x97%2").arg(qs(axisLabel(c, cat, link.source)), QString::number(ratio, 'g', 4));
        if (om.incoming.size() > 1) text += QStringLiteral(" +%1").arg(om.incoming.size() - 1);
        parts << text;
    }
    if (!om.drives.empty()) parts << ETR("drives %1").arg(om.drives.size());
    return parts.join(QStringLiteral(" \xC2\xB7 "));
}

std::unique_ptr<ProjectTreeModel::Node> ProjectTreeModel::makeWheelNode(const sdk::RobotConfig& c, const sdk::WheelSpec& w, const MotionIndex& motion) const {
    const EditorCatalog& cat = document_.catalog();
    auto node = std::make_unique<Node>();
    Node& n = *node;
    n.kind = NodeKind::Wheel;
    n.key = QStringLiteral("wheel:%1").arg(qs(w.name));
    n.ref = SelectionRef::wheel(w.name);
    n.text = qs(w.name);

    const std::string prefix = w.name + ".";
    std::vector<SlotInfo> slots;
    for (SlotInfo& info : slotInfos(c, cat, devices_, sdk::slot_owner::kDrivetrain))
        if (info.spec.name.rfind(prefix, 0) == 0) slots.push_back(std::move(info));

    const OwnerMotion om = motion.ownerMotion(n.ref);
    n.follower = om.followsAll;
    QStringList parts;
    // A wheel whose only slot is its drive motor shows that motor on the wheel row itself.
    if (slots.size() == 1 && slots.front().spec.name == w.name + ".drive") {
        const SlotInfo& s = slots.front();
        parts << (s.mapped ? mappedLabel(s) : (s.spec.required ? ETR("not mapped") : ETR("optional")));
        n.state = static_cast<int>(s.state);
        n.warning = needsAttention(s.state);
        n.tooltip = qs(s.spec.description) + (s.spec.description.empty() ? QString() : QStringLiteral("\n")) + s.detail;
    } else {
        for (const SlotInfo& s : slots) {
            auto child = std::make_unique<Node>();
            child->kind = NodeKind::Slot;
            child->key = QStringLiteral("slot:%1:%2").arg(qs(std::string(sdk::slot_owner::kDrivetrain)), qs(s.spec.name));
            child->ref = SelectionRef::slot(std::string(sdk::slot_owner::kDrivetrain), s.spec.name);
            child->text = qs(s.spec.name.substr(prefix.size()));
            child->detail = s.mapped ? mappedLabel(s) : (s.spec.required ? ETR("not mapped") : ETR("optional"));
            child->state = static_cast<int>(s.state);
            child->tooltip = qs(s.spec.description) + (s.spec.description.empty() ? QString() : QStringLiteral("\n")) + s.detail;
            child->parent = &n;
            n.warning = n.warning || needsAttention(s.state);
            n.children.push_back(std::move(child));
        }
    }
    const QString motionText = motionDetail(c, cat, motion, om);
    if (!motionText.isEmpty()) parts << motionText;
    n.detail = parts.join(QStringLiteral(" \xC2\xB7 "));
    if (!om.incoming.empty())
        n.tooltip = ETR("Driven by %1 through a motion link; it takes no motor of its own.").arg(qs(axisLabel(c, cat, om.incoming.front()->source))) +
                    (n.tooltip.isEmpty() ? QString() : QStringLiteral("\n") + n.tooltip);
    return node;
}

std::unique_ptr<ProjectTreeModel::Node> ProjectTreeModel::makeMechanismNode(const sdk::RobotConfig& c, const sdk::MechanismConfig& m, const MotionIndex& motion) const {
    const EditorCatalog& cat = document_.catalog();
    auto node = std::make_unique<Node>();
    Node& n = *node;
    n.kind = NodeKind::Mechanism;
    n.key = QStringLiteral("mech:%1").arg(qs(m.id));
    n.ref = SelectionRef::mechanism(m.id);
    n.text = qs(ops::displayNameFor(m, cat));
    const auto* info = cat.findTemplate(m.type);
    QString detail = info != nullptr ? mechanismKindLabel(m, *info) : qs(m.type) + ETR(" (unknown template)");
    const auto att = m.attachment;
    const OwnerMotion om = motion.ownerMotion(n.ref);
    n.follower = om.followsAll;
    // A follower is placed under its source, not where it is attached, so its attachment stays visible as text.
    if (!att.onChassis()) detail += QStringLiteral(" @ %1").arg(qs(att.parentMechanismId + (att.parentLink == sdk::kRootLink ? "" : "." + att.parentLink)));
    else if (om.primarySource) detail += ETR(" @ chassis");
    const QString motionText = motionDetail(c, cat, motion, om);
    n.detail = motionText.isEmpty() ? detail : detail + QStringLiteral(" \xC2\xB7 ") + motionText;
    if (!om.incoming.empty())
        n.tooltip = ETR("Driven by %1 through a motion link; its linked motors need no mapping.").arg(qs(axisLabel(c, cat, om.incoming.front()->source)));

    auto addChild = [&](NodeKind kind, QString key, SelectionRef ref, QString text, QString childDetail, int state = -1) -> Node& {
        auto child = std::make_unique<Node>();
        child->kind = kind;
        child->key = std::move(key);
        child->ref = std::move(ref);
        child->text = std::move(text);
        child->detail = std::move(childDetail);
        child->state = state;
        child->parent = &n;
        n.children.push_back(std::move(child));
        return *n.children.back();
    };
    for (const SlotInfo& s : slotInfos(c, cat, devices_, m.id)) {
        QString slotDetail = s.mapped ? mappedLabel(s) : (s.spec.required ? ETR("not mapped") : ETR("optional"));
        Node& sn = addChild(NodeKind::Slot, QStringLiteral("slot:%1:%2").arg(qs(m.id), qs(s.spec.name)), SelectionRef::slot(m.id, s.spec.name),
                            qs(s.spec.label.empty() ? s.spec.name : s.spec.label), slotDetail, static_cast<int>(s.state));
        sn.tooltip = qs(s.spec.description) + (s.spec.description.empty() ? QString() : QStringLiteral("\n")) + s.detail;
        n.warning = n.warning || needsAttention(s.state);
    }
    for (std::size_t i = 0; i < m.geometry.size(); ++i)
        addChild(NodeKind::Shape, QStringLiteral("shape:%1:%2").arg(qs(m.id)).arg(i), SelectionRef::primitive(m.id, static_cast<int>(i)),
                 qs(m.geometry[i].name), m.geometry[i].kind == sdk::PrimitiveKind::Box ? ETR("box") : ETR("cylinder"));
    for (const auto& vol : ops::volumesFor(c, cat, m.id))
        addChild(NodeKind::Volume, QStringLiteral("volume:%1:%2").arg(qs(m.id), qs(vol.key)), SelectionRef::overlay(m.id, vol.key), qs(vol.label), ETR("volume"));
    return node;
}

void ProjectTreeModel::flatten(const Node& n, std::vector<const Node*>& out) {
    out.push_back(&n);
    for (const auto& c : n.children) flatten(*c, out);
}

bool ProjectTreeModel::sameShape(const Node& a, const Node& b) {
    if (a.key != b.key || a.children.size() != b.children.size()) return false;
    for (std::size_t i = 0; i < a.children.size(); ++i)
        if (!sameShape(*a.children[i], *b.children[i])) return false;
    return true;
}

void ProjectTreeModel::copyContents(Node& dst, const Node& src) {
    dst.kind = src.kind;
    dst.ref = src.ref;
    dst.text = src.text;
    dst.detail = src.detail;
    dst.tooltip = src.tooltip;
    dst.state = src.state;
    dst.warning = src.warning;
    dst.follower = src.follower;
    dst.nestedByLink = src.nestedByLink;
    for (std::size_t i = 0; i < dst.children.size(); ++i) copyContents(*dst.children[i], *src.children[i]);
}

void ProjectTreeModel::setIssues(const std::vector<sdk::Issue>& issues) {
    std::set<std::string> zones;
    for (const sdk::Issue& issue : issues) {
        if (issue.severity == sdk::Severity::Info) continue;
        const IssueLocation loc = locateIssue(document_.config(), document_.catalog(), issue);
        if (loc.selection.kind == SelKind::Zone) zones.insert(loc.selection.owner);
    }
    if (zones == zonesWithIssues_) return;
    zonesWithIssues_ = std::move(zones);
    refresh();
}

void ProjectTreeModel::refresh() {
    std::unique_ptr<Node> fresh = build();
    if (root_ && sameShape(*root_, *fresh)) {
        copyContents(*root_, *fresh);
        // Contents only: let the views repaint every row without losing selection or expansion.
        std::vector<const Node*> all;
        flatten(*root_, all);
        for (const Node* n : all) {
            if (n == root_.get()) continue;
            const QModelIndex idx = indexOfNode(n);
            Q_EMIT dataChanged(idx, idx);
        }
        return;
    }
    beginResetModel();
    root_ = std::move(fresh);
    endResetModel();
}

const ProjectTreeModel::Node* ProjectTreeModel::nodeAt(const QModelIndex& index) const {
    return index.isValid() ? static_cast<const Node*>(index.internalPointer()) : nullptr;
}

QModelIndex ProjectTreeModel::indexOfNode(const Node* node) const {
    if (node == nullptr || node == root_.get() || node->parent == nullptr) return {};
    const auto& siblings = node->parent->children;
    for (std::size_t i = 0; i < siblings.size(); ++i)
        if (siblings[i].get() == node) return createIndex(static_cast<int>(i), 0, const_cast<Node*>(node));
    return {};
}

QModelIndex ProjectTreeModel::index(int row, int column, const QModelIndex& parentIndex) const {
    const Node* parentNode = parentIndex.isValid() ? nodeAt(parentIndex) : root_.get();
    if (parentNode == nullptr || row < 0 || row >= static_cast<int>(parentNode->children.size()) || column != 0) return {};
    return createIndex(row, column, parentNode->children[static_cast<std::size_t>(row)].get());
}

QModelIndex ProjectTreeModel::parent(const QModelIndex& child) const {
    const Node* n = nodeAt(child);
    if (n == nullptr || n->parent == nullptr || n->parent == root_.get()) return {};
    return indexOfNode(n->parent);
}

int ProjectTreeModel::rowCount(const QModelIndex& parentIndex) const {
    if (parentIndex.column() > 0) return 0;
    const Node* n = parentIndex.isValid() ? nodeAt(parentIndex) : root_.get();
    return n != nullptr ? static_cast<int>(n->children.size()) : 0;
}

QVariant ProjectTreeModel::data(const QModelIndex& index, int role) const {
    const Node* n = nodeAt(index);
    if (n == nullptr) return {};
    switch (role) {
        case Qt::DisplayRole: return n->text;
        case Qt::ToolTipRole: return n->tooltip.isEmpty() ? (n->detail.isEmpty() ? n->text : n->text + QStringLiteral(" \xE2\x80\x94 ") + n->detail) : n->tooltip;
        case KindRole: return static_cast<int>(n->kind);
        case StateRole: return n->state;
        case DetailRole: return n->detail;
        case KeyRole: return n->key;
        case WarningRole: return n->warning;
        case FollowerRole: return n->follower;
        case RefKeyRole: return qs(n->ref.key());
        default: return {};
    }
}

SelectionRef ProjectTreeModel::refAt(const QModelIndex& index) const {
    const Node* n = nodeAt(index);
    return n != nullptr ? n->ref : SelectionRef{};
}

QString ProjectTreeModel::keyAt(const QModelIndex& index) const {
    const Node* n = nodeAt(index);
    return n != nullptr ? n->key : QString();
}

const ProjectTreeModel::Node* ProjectTreeModel::findByKey(const Node& node, const QString& key) const {
    if (node.key == key) return &node;
    for (const auto& c : node.children)
        if (const Node* f = findByKey(*c, key)) return f;
    return nullptr;
}

const ProjectTreeModel::Node* ProjectTreeModel::findByRef(const Node& node, const SelectionRef& ref) const {
    if (!node.ref.isNone() && node.ref == ref) return &node;
    for (const auto& c : node.children)
        if (const Node* f = findByRef(*c, ref)) return f;
    return nullptr;
}

QModelIndex ProjectTreeModel::indexForKey(const QString& key) const { return indexOfNode(findByKey(*root_, key)); }
QModelIndex ProjectTreeModel::indexFor(const SelectionRef& ref) const {
    if (ref.isNone()) return {};
    return indexOfNode(findByRef(*root_, ref));
}

sdk::MechanismAttachment ProjectTreeModel::attachmentFor(const QModelIndex& index) const {
    sdk::MechanismAttachment att;
    const Node* n = nodeAt(index);
    if (n != nullptr && n->kind == NodeKind::Mechanism) att.parentMechanismId = n->ref.owner;
    return att;
}

Qt::ItemFlags ProjectTreeModel::flags(const QModelIndex& index) const {
    Qt::ItemFlags f = Qt::ItemIsEnabled | Qt::ItemIsDropEnabled;
    const Node* n = nodeAt(index);
    if (n == nullptr) return f;
    if (!n->ref.isNone()) f |= Qt::ItemIsSelectable;
    // Followers sit under their source, not under their attachment, so they cannot be re-attached by dragging.
    if (n->kind == NodeKind::Mechanism && !n->nestedByLink) f |= Qt::ItemIsDragEnabled;
    if (n->kind == NodeKind::Zone) f |= Qt::ItemIsDragEnabled;
    return f;
}

QStringList ProjectTreeModel::mimeTypes() const {
    return {QString::fromLatin1(kDeviceMime), QString::fromLatin1(kMechanismMime), QString::fromLatin1(kZoneMime)};
}

QMimeData* ProjectTreeModel::mimeData(const QModelIndexList& indexes) const {
    for (const QModelIndex& i : indexes) {
        const Node* n = nodeAt(i);
        if (n != nullptr && n->kind == NodeKind::Zone) {
            auto* mime = new QMimeData;
            mime->setData(QString::fromLatin1(kZoneMime), QByteArray::fromStdString(n->ref.owner));
            return mime;
        }
    }
    for (const QModelIndex& i : indexes) {
        const Node* n = nodeAt(i);
        if (n != nullptr && n->kind == NodeKind::Mechanism) {
            auto* mime = new QMimeData;
            mime->setData(QString::fromLatin1(kMechanismMime), QByteArray::fromStdString(n->ref.owner));
            return mime;
        }
    }
    return nullptr;
}

bool ProjectTreeModel::acceptsDevice(const Node* target, sdk::CapabilitySet caps) const {
    if (target == nullptr) return false;
    const sdk::RobotConfig& c = document_.config();
    if (target->kind == NodeKind::Slot) {
        if (MotionIndex(c, document_.catalog()).isFollowerMotor({target->ref.owner, target->ref.sub})) return false;
        for (const auto& spec : ops::slotSpecsFor(c, document_.catalog(), target->ref.owner))
            if (spec.name == target->ref.sub) return slotAccepts(spec, caps);
        return false;
    }
    if (target->kind == NodeKind::Mechanism)
        return canAcceptDevice(c, document_.catalog(), target->ref.owner, caps);
    if (target->kind == NodeKind::Wheel)
        return canAcceptDevice(c, document_.catalog(), sdk::slot_owner::kDrivetrain, caps, target->ref.sub + ".");
    return false;
}

bool ProjectTreeModel::canDropMimeData(const QMimeData* data, Qt::DropAction, int, int, const QModelIndex& parentIndex) const {
    const Node* target = nodeAt(parentIndex);
    if (const auto payload = devicePayload(data)) return acceptsDevice(target, payload->capabilities);

    const bool attachTarget = target == nullptr || target->kind == NodeKind::Mechanism || target->kind == NodeKind::Chassis ||
                              target->kind == NodeKind::Robot || (target->kind == NodeKind::Group && target->key == QStringLiteral("mechanisms"));
    if (data != nullptr && data->hasFormat(QString::fromLatin1(kZoneMime))) {
        if (!attachTarget) return false;
        const std::string id = data->data(QString::fromLatin1(kZoneMime)).toStdString();
        const sdk::ZoneConfig* zone = document_.config().findZone(id);
        if (zone == nullptr) return false;
        const std::string parent = target != nullptr && target->kind == NodeKind::Mechanism ? target->ref.owner : std::string(sdk::kChassisParent);
        const sdk::MechanismAttachment current = zone->attachment;
        return current.onChassis() ? parent != sdk::kChassisParent : current.parentMechanismId != parent || current.parentLink != sdk::kRootLink;
    }
    if (data != nullptr && data->hasFormat(QString::fromLatin1(kMechanismMime))) {
        if (!attachTarget) return false;
        const std::string id = data->data(QString::fromLatin1(kMechanismMime)).toStdString();
        const std::string parent = target != nullptr && target->kind == NodeKind::Mechanism ? target->ref.owner : std::string(sdk::kChassisParent);
        return ops::canReparent(document_.config(), id, parent) && id != parent;
    }
    return false;
}

bool ProjectTreeModel::dropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parentIndex) {
    if (!canDropMimeData(data, action, row, column, parentIndex)) return false;
    const Node* target = nodeAt(parentIndex);
    if (const auto payload = devicePayload(data)) {
        Q_EMIT deviceDropped(target->ref, payload->key, payload->capabilities.bits);
        return true;
    }
    sdk::MechanismAttachment att;
    if (target != nullptr && target->kind == NodeKind::Mechanism) att.parentMechanismId = target->ref.owner;
    if (data->hasFormat(QString::fromLatin1(kZoneMime))) {
        Q_EMIT zoneDropped(QString::fromUtf8(data->data(QString::fromLatin1(kZoneMime))), att);
        return true;
    }
    Q_EMIT mechanismDropped(QString::fromUtf8(data->data(QString::fromLatin1(kMechanismMime))), att);
    return true;
}

}  // namespace wpiemu::app::editor
