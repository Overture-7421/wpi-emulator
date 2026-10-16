#include "viewmodel/project_tree_model.h"

#include <QMimeData>

#include <algorithm>

#include "model/config_ops.h"
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
    auto addSlots = [&](Node& parent, std::string_view owner, const std::string& prefix) {
        bool warn = false;
        for (const SlotInfo& info : slotInfos(c, cat, devices_, owner)) {
            if (!prefix.empty() && info.spec.name.rfind(prefix, 0) != 0) continue;
            const QString label = prefix.empty() ? qs(info.spec.label.empty() ? info.spec.name : info.spec.label)
                                                 : qs(info.spec.name.substr(prefix.size()));
            QString detail;
            if (info.mapped) detail = deviceLabel(*info.mapped);
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

    Node& drive = add(robot, NodeKind::Drivetrain, QStringLiteral("drivetrain"), SelectionRef::drivetrain(), ETR("Drivetrain"),
                      qs(c.drivetrain.spec.kind));
    for (const auto& w : c.drivetrain.spec.wheels) {
        Node& wn = add(drive, NodeKind::Wheel, QStringLiteral("wheel:%1").arg(qs(w.name)), SelectionRef::wheel(w.name), qs(w.name),
                       qs(w.driveMotor.name));
        wn.warning = addSlots(wn, sdk::slot_owner::kDrivetrain, w.name + ".");
    }

    Node& mechs = add(robot, NodeKind::Group, QStringLiteral("mechanisms"), SelectionRef{}, ETR("Mechanisms"),
                      QString::number(c.mechanisms.size()));
    for (const auto& m : c.mechanisms)
        if (m.resolvedAttachment().onChassis()) addMechanismNode(mechs, c, m);
    // Mechanisms whose parent is missing (broken config) still show up so they can be fixed.
    for (const auto& m : c.mechanisms) {
        const auto att = m.resolvedAttachment();
        if (!att.onChassis() && c.findMechanism(att.parentMechanismId) == nullptr) addMechanismNode(mechs, c, m);
    }

    Node& sensors = add(robot, NodeKind::Group, QStringLiteral("sensors"), SelectionRef{}, ETR("Robot sensors"));
    addSlots(sensors, sdk::slot_owner::kRobot, {});

    if (cat.hasPlugin()) {
        Node& plugin = add(robot, NodeKind::Plugin, QStringLiteral("plugin"), SelectionRef::plugin(), qs(cat.pluginDisplayName), ETR("game rules"));
        for (const auto& vol : ops::volumesFor(c, cat, ""))
            add(plugin, NodeKind::Volume, QStringLiteral("volume::%1").arg(qs(vol.key)), SelectionRef::overlay({}, vol.key), qs(vol.label), ETR("volume"));
    }
    return root;
}

void ProjectTreeModel::addMechanismNode(Node& parentNode, const sdk::RobotConfig& c, const sdk::MechanismConfig& m) const {
    const EditorCatalog& cat = document_.catalog();
    auto node = std::make_unique<Node>();
    Node& n = *node;
    n.kind = NodeKind::Mechanism;
    n.key = QStringLiteral("mech:%1").arg(qs(m.id));
    n.ref = SelectionRef::mechanism(m.id);
    n.text = qs(ops::displayNameFor(m, cat));
    const auto* info = cat.findTemplate(m.type);
    n.detail = info != nullptr ? qs(info->displayName) : qs(m.type) + ETR(" (unknown template)");
    const auto att = m.resolvedAttachment();
    if (!att.onChassis()) n.detail += QStringLiteral(" @ %1").arg(qs(att.parentMechanismId + (att.parentLink == sdk::kRootLink ? "" : "." + att.parentLink)));
    n.parent = &parentNode;
    parentNode.children.push_back(std::move(node));
    Node& self = *parentNode.children.back();

    auto addChild = [&](NodeKind kind, QString key, SelectionRef ref, QString text, QString detail, int state = -1) -> Node& {
        auto child = std::make_unique<Node>();
        child->kind = kind;
        child->key = std::move(key);
        child->ref = std::move(ref);
        child->text = std::move(text);
        child->detail = std::move(detail);
        child->state = state;
        child->parent = &self;
        self.children.push_back(std::move(child));
        return *self.children.back();
    };
    for (const SlotInfo& s : slotInfos(c, cat, devices_, m.id)) {
        QString detail = s.mapped ? deviceLabel(*s.mapped) : (s.spec.required ? ETR("not mapped") : ETR("optional"));
        Node& sn = addChild(NodeKind::Slot, QStringLiteral("slot:%1:%2").arg(qs(m.id), qs(s.spec.name)), SelectionRef::slot(m.id, s.spec.name),
                            qs(s.spec.label.empty() ? s.spec.name : s.spec.label), detail, static_cast<int>(s.state));
        sn.tooltip = qs(s.spec.description) + (s.spec.description.empty() ? QString() : QStringLiteral("\n")) + s.detail;
        self.warning = self.warning || needsAttention(s.state);
    }
    for (std::size_t i = 0; i < m.geometry.size(); ++i)
        addChild(NodeKind::Shape, QStringLiteral("shape:%1:%2").arg(qs(m.id)).arg(i), SelectionRef::primitive(m.id, static_cast<int>(i)),
                 qs(m.geometry[i].name), m.geometry[i].kind == sdk::PrimitiveKind::Box ? ETR("box") : ETR("cylinder"));
    for (const auto& vol : ops::volumesFor(c, cat, m.id))
        addChild(NodeKind::Volume, QStringLiteral("volume:%1:%2").arg(qs(m.id), qs(vol.key)), SelectionRef::overlay(m.id, vol.key), qs(vol.label), ETR("volume"));
    for (const auto& child : c.mechanisms) {
        const auto ca = child.resolvedAttachment();
        if (!ca.onChassis() && ca.parentMechanismId == m.id && child.id != m.id) addMechanismNode(self, c, child);
    }
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
    for (std::size_t i = 0; i < dst.children.size(); ++i) copyContents(*dst.children[i], *src.children[i]);
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
    if (n->kind == NodeKind::Mechanism) f |= Qt::ItemIsDragEnabled;
    return f;
}

QStringList ProjectTreeModel::mimeTypes() const {
    return {QString::fromLatin1(kDeviceMime), QString::fromLatin1(kTemplateMime), QString::fromLatin1(kMechanismMime)};
}

QMimeData* ProjectTreeModel::mimeData(const QModelIndexList& indexes) const {
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
        for (const auto& spec : ops::slotSpecsFor(c, document_.catalog(), target->ref.owner))
            if (spec.name == target->ref.sub) return slotAccepts(spec, caps);
        return false;
    }
    if (target->kind == NodeKind::Mechanism)
        return bestSlotFor(c, document_.catalog(), target->ref.owner, caps).has_value();
    if (target->kind == NodeKind::Wheel)
        return bestSlotFor(c, document_.catalog(), sdk::slot_owner::kDrivetrain, caps, target->ref.sub + ".").has_value();
    return false;
}

bool ProjectTreeModel::canDropMimeData(const QMimeData* data, Qt::DropAction, int, int, const QModelIndex& parentIndex) const {
    const Node* target = nodeAt(parentIndex);
    if (const auto payload = devicePayload(data)) return acceptsDevice(target, payload->capabilities);

    const bool attachTarget = target == nullptr || target->kind == NodeKind::Mechanism || target->kind == NodeKind::Chassis ||
                              target->kind == NodeKind::Robot || (target->kind == NodeKind::Group && target->key == QStringLiteral("mechanisms"));
    if (data != nullptr && data->hasFormat(QString::fromLatin1(kTemplateMime))) return attachTarget;
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
    if (data->hasFormat(QString::fromLatin1(kTemplateMime))) {
        Q_EMIT templateDropped(QString::fromUtf8(data->data(QString::fromLatin1(kTemplateMime))), att);
        return true;
    }
    Q_EMIT mechanismDropped(QString::fromUtf8(data->data(QString::fromLatin1(kMechanismMime))), att);
    return true;
}

}  // namespace wpiemu::app::editor
