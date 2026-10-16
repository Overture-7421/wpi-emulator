#include "scene_model.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace wpiemu::app::runview {
namespace {

constexpr float kUnit = 100.f;  // size of Qt Quick 3D's built-in #Cube / #Cylinder / #Sphere
const QQuaternion kCylinderToZ = QQuaternion::fromAxisAndAngle(1.f, 0.f, 0.f, 90.f);

QVector3D toQt(const sdk::Vec3& v) { return {v.x, v.y, v.z}; }
QQuaternion toQt(const sdk::Quat& q) { return {q.w, q.x, q.y, q.z}; }

}  // namespace

QColor parseColor(const std::string& css, const QColor& fallback) {
    const QColor c(QString::fromStdString(css));
    return c.isValid() ? c : fallback;
}

// ---- EntityInstancing --------------------------------------------------------------------------
EntityInstancing::EntityInstancing(QObject* parent) : QQuick3DInstancing(nullptr) { setParent(parent); }

void EntityInstancing::setPositions(const std::vector<sdk::Vec3>& positions) {
    const auto entrySize = static_cast<qsizetype>(sizeof(InstanceTableEntry));
    buffer_.resize(entrySize * static_cast<qsizetype>(positions.size()));
    auto* out = reinterpret_cast<InstanceTableEntry*>(buffer_.data());
    for (std::size_t i = 0; i < positions.size(); ++i)
        out[i] = calculateTableEntry(toQt(positions[i]), QVector3D(1, 1, 1), QVector3D(0, 0, 0), Qt::white);
    count_ = static_cast<int>(positions.size());
    markDirty();
}

QByteArray EntityInstancing::getInstanceBuffer(int* instanceCount) {
    *instanceCount = count_;
    return buffer_;
}

// ---- PartListModel -----------------------------------------------------------------------------
PartListModel::PartListModel(QObject* parent) : QAbstractListModel(parent) {}

PartListModel::Item PartListModel::makeItem(const ScenePart& part) {
    Item it;
    it.name = QString::fromStdString(part.name);
    it.shape = part.shape == PartShape::Cylinder ? 1 : 0;
    it.position = toQt(part.pose.position);
    if (part.shape == PartShape::Cylinder) {
        it.rotation = toQt(part.pose.rotation) * kCylinderToZ;
        it.scale = {part.size.x / kUnit, part.size.z / kUnit, part.size.y / kUnit};
    } else {
        it.rotation = toQt(part.pose.rotation);
        it.scale = {part.size.x / kUnit, part.size.y / kUnit, part.size.z / kUnit};
    }
    it.color = parseColor(part.color);
    if (part.translucent) it.color.setAlphaF(0.28f);
    return it;
}

int PartListModel::rowCount(const QModelIndex& parent) const { return parent.isValid() ? 0 : static_cast<int>(items_.size()); }

QVariant PartListModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const Item& it = items_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case NameRole: return it.name;
        case ShapeRole: return it.shape;
        case PositionRole: return it.position;
        case RotationRole: return it.rotation;
        case ScaleRole: return it.scale;
        case ColorRole: return it.color;
        case ShownRole: return it.label.isEmpty() || !hidden_.contains(it.label);
        case LabelRole: return it.label;
        default: return {};
    }
}

QHash<int, QByteArray> PartListModel::roleNames() const {
    return {{NameRole, "partName"}, {ShapeRole, "shape"},   {PositionRole, "partPosition"}, {RotationRole, "partRotation"},
            {ScaleRole, "partScale"}, {ColorRole, "partColor"}, {ShownRole, "shown"},        {LabelRole, "label"}};
}

void PartListModel::setParts(const std::vector<ScenePart>& parts, const std::vector<std::string>& labels) {
    beginResetModel();
    items_.clear();
    items_.reserve(parts.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
        Item it = makeItem(parts[i]);
        if (i < labels.size()) it.label = QString::fromStdString(labels[i]);
        items_.push_back(std::move(it));
    }
    endResetModel();
}

void PartListModel::updatePoses(const std::vector<ScenePart>& parts, const std::vector<char>& changed) {
    const std::size_t n = std::min({parts.size(), items_.size(), changed.size()});
    for (std::size_t i = 0; i < n; ++i) {
        if (!changed[i]) continue;
        const QString label = items_[i].label;
        items_[i] = makeItem(parts[i]);
        items_[i].label = label;
        const QModelIndex idx = index(static_cast<int>(i));
        emit dataChanged(idx, idx, {PositionRole, RotationRole, ScaleRole});
    }
}

void PartListModel::setLabelShown(const QString& label, bool shown) {
    if (shown == !hidden_.contains(label)) return;
    if (shown) hidden_.remove(label);
    else hidden_.insert(label);
    for (std::size_t i = 0; i < items_.size(); ++i)
        if (items_[i].label == label) {
            const QModelIndex idx = index(static_cast<int>(i));
            emit dataChanged(idx, idx, {ShownRole});
        }
}

// ---- EntityGroupModel --------------------------------------------------------------------------
EntityGroupModel::EntityGroupModel(QObject* parent) : QAbstractListModel(parent) {}

int EntityGroupModel::rowCount(const QModelIndex& parent) const { return parent.isValid() ? 0 : static_cast<int>(groups_.size()); }

QVariant EntityGroupModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const Group& g = groups_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case NameRole: return g.name;
        case SphereRole: return g.sphere;
        case ScaleRole: return g.scale;
        case ColorRole: return g.color;
        case InstancingRole: return QVariant::fromValue<QObject*>(g.instancing);
        default: return {};
    }
}

QHash<int, QByteArray> EntityGroupModel::roleNames() const {
    return {{NameRole, "groupName"}, {SphereRole, "isSphere"}, {ScaleRole, "groupScale"},
            {ColorRole, "groupColor"}, {InstancingRole, "entityInstancing"}};
}

int EntityGroupModel::totalInstances() const {
    int n = 0;
    for (const auto& g : groups_) n += g.instancing != nullptr ? g.instancing->count() : 0;
    return n;
}

void EntityGroupModel::update(std::span<const sdk::EntityGroupSample> groups) {
    auto scaleOf = [](const sdk::EntityGroupSample& s) {
        return s.shape == "box" ? QVector3D(s.halfExtentsM.x * 2.f, s.halfExtentsM.y * 2.f, s.halfExtentsM.z * 2.f) / kUnit
                                : QVector3D(1, 1, 1) * (s.radiusM * 2.f / kUnit);
    };
    bool structural = groups.size() != groups_.size();
    for (std::size_t i = 0; !structural && i < groups.size(); ++i)
        structural = groups_[i].name.toStdString() != groups[i].group || groups_[i].sphere != (groups[i].shape != "box");

    if (structural) {
        beginResetModel();
        for (auto& g : groups_) delete g.instancing;
        groups_.clear();
        for (const auto& s : groups) {
            Group g;
            g.name = QString::fromStdString(s.group);
            g.sphere = s.shape != "box";
            g.scale = scaleOf(s);
            g.color = parseColor(s.color, QColor(QStringLiteral("#ffd000")));
            g.instancing = new EntityInstancing(this);
            groups_.push_back(std::move(g));
        }
        endResetModel();
    }
    for (std::size_t i = 0; i < groups.size(); ++i) {
        Group& g = groups_[i];
        const QVector3D scale = scaleOf(groups[i]);
        const QColor color = parseColor(groups[i].color, QColor(QStringLiteral("#ffd000")));
        if (scale != g.scale || color != g.color) {
            g.scale = scale;
            g.color = color;
            const QModelIndex idx = index(static_cast<int>(i));
            emit dataChanged(idx, idx, {ScaleRole, ColorRole});
        }
        g.instancing->setPositions(groups[i].positions);
    }
}

// ---- ViewportTheme -----------------------------------------------------------------------------
ViewportTheme::ViewportTheme(QObject* parent) : QObject(parent) { setPalette(QPalette()); }

void ViewportTheme::setPalette(const QPalette& p) {
    const QColor window = p.color(QPalette::Window);
    dark_ = window.lightnessF() < 0.5;
    background_ = dark_ ? QColor(QStringLiteral("#12151a")) : QColor(QStringLiteral("#dfe5ee"));
    ground_ = dark_ ? QColor(QStringLiteral("#1a1e25")) : QColor(QStringLiteral("#c9d1de"));
    panel_ = dark_ ? QColor(24, 28, 34, 215) : QColor(255, 255, 255, 215);
    panelBorder_ = dark_ ? QColor(255, 255, 255, 40) : QColor(0, 0, 0, 40);
    text_ = dark_ ? QColor(QStringLiteral("#e8ecf2")) : QColor(QStringLiteral("#1b2230"));
    muted_ = dark_ ? QColor(QStringLiteral("#9aa5b6")) : QColor(QStringLiteral("#586377"));
    accent_ = p.color(QPalette::Highlight);
    emit changed();
}

// ---- SceneModel --------------------------------------------------------------------------------
SceneModel::SceneModel(QObject* parent) : QObject(parent), field_(this), robot_(this), zones_(this), entities_(this) {
    refreshCamera();
}

void SceneModel::setSceneInfo(const RunSceneInfo& info) {
    info_ = info;
    rig_.setConfig(info_.config);
    robot_.setParts(rig_.parts());

    const FieldParts fp = buildFieldParts(info_.field.get());
    field_.setParts(fp.parts);
    fieldLength_ = fp.lengthM;
    fieldWidth_ = fp.widthM;
    fieldCenter_ = toQt(fp.center);
    camera_.setField(fp.center, fp.lengthM, fp.widthM);
    emit fieldChanged();

    zoneState_.clear();
    zones_.setParts({});
    zoneLabels_.clear();
    updateZones(nullptr);
    refreshCamera();
}

void SceneModel::setZonesVisible(bool v) {
    if (v == zonesVisible_) return;
    zonesVisible_ = v;
    emit zonesVisibleChanged();
}

void SceneModel::setCameraMode(int mode) {
    camera_.setMode(static_cast<CameraMode>(std::clamp(mode, 0, 2)));
    refreshCamera();
}

void SceneModel::refreshCamera() {
    const CameraPose p = camera_.pose();
    const QVector3D pos = toQt(p.position);
    const QQuaternion rot = toQt(p.rotation);
    if (pos == cameraPosition_ && rot == cameraRotation_ && p.fieldOfViewDeg == cameraFov_) return;
    cameraPosition_ = pos;
    cameraRotation_ = rot;
    cameraFov_ = p.fieldOfViewDeg;
    emit cameraChanged();
}

void SceneModel::updateZones(const sdk::ISimSnapshot* snapshot) {
    std::vector<ZoneOverlay> next;
    if (info_.config) next = resolveZoneOverlays(*info_.config, info_.panels, snapshot);

    auto sameShape = [&] {
        if (next.size() != zoneState_.size()) return false;
        for (std::size_t i = 0; i < next.size(); ++i)
            if (next[i].label != zoneState_[i].label || next[i].color != zoneState_[i].color) return false;
        return true;
    };
    auto toParts = [&] {
        std::vector<ScenePart> parts;
        for (const auto& z : next) {
            ScenePart p;
            p.name = z.label;
            p.pose = z.pose;
            p.size = z.size;
            p.color = z.color;
            p.translucent = true;
            parts.push_back(std::move(p));
        }
        return parts;
    };

    if (!sameShape()) {
        std::vector<std::string> labels;
        QStringList uniq;
        for (const auto& z : next) {
            labels.push_back(z.label);
            const QString l = QString::fromStdString(z.label);
            if (!uniq.contains(l)) uniq << l;
        }
        zones_.setParts(toParts(), labels);
        if (uniq != zoneLabels_) {
            zoneLabels_ = uniq;
            emit zoneLabelsChanged();
        }
    } else {
        std::vector<char> changed(next.size(), 0);
        bool any = false;
        for (std::size_t i = 0; i < next.size(); ++i)
            if (!(next[i].pose == zoneState_[i].pose) || !(next[i].size == zoneState_[i].size)) { changed[i] = 1; any = true; }
        if (any) zones_.updatePoses(toParts(), changed);
    }
    zoneState_ = std::move(next);
}

void SceneModel::update(const sdk::ISimSnapshot* snapshot, double dtSeconds) {
    if (snapshot != nullptr) {
        const sdk::Pose3& pose = snapshot->chassis().pose;
        const QVector3D pos = toQt(pose.position);
        const QQuaternion rot = toQt(pose.rotation);
        if (pos != chassisPosition_ || rot != chassisRotation_ || !robotVisible_) {
            chassisPosition_ = pos;
            chassisRotation_ = rot;
            robotVisible_ = true;
            emit poseChanged();
        }
        camera_.setRobot(pose.position, pose.rotation.yaw());
        if (rig_.update(snapshot)) robot_.updatePoses(rig_.parts(), rig_.changed());
        entities_.update(snapshot->entityGroups());
        updateZones(snapshot);
    } else if (robotVisible_) {
        robotVisible_ = false;
        emit poseChanged();
        entities_.update({});
    }
    camera_.advance(dtSeconds);
    refreshCamera();
}

}  // namespace wpiemu::app::runview
