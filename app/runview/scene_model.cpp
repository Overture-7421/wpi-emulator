#include "scene_model.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "camera_hud.h"
#include "common/camera_frustum.h"
#include "status_chip.h"

namespace wpiemu::app::runview {
namespace {

constexpr float kUnit = 100.f;  // size of Qt Quick 3D's built-in #Cube / #Cylinder / #Sphere
const QQuaternion kCylinderToZ = QQuaternion::fromAxisAndAngle(1.f, 0.f, 0.f, 90.f);

QVector3D toQt(const sdk::Vec3& v) { return {v.x, v.y, v.z}; }
QQuaternion toQt(const sdk::Quat& q) { return {q.w, q.x, q.y, q.z}; }

// The camera body marker and the opacities of the camera colours: a camera that sees no tag is dimmer, a disabled one barely there.
constexpr QVector3D kCameraBodyM{0.07f, 0.11f, 0.05f};
constexpr float kBodyOpacitySeeing = 1.0f;
constexpr float kBodyOpacityIdle = 0.55f;
constexpr float kFrustumOpacitySeeing = 0.22f;
constexpr float kFrustumOpacityIdle = 0.07f;
constexpr float kFrustumOpacityDisabled = 0.04f;

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
    if (part.highlighted) it.color = it.color.lighter(140);
    if (part.translucent) it.color.setAlphaF(part.opacity);
    it.visible = part.visible;
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
        case ShownRole: return it.visible && (it.label.isEmpty() || !hidden_.contains(it.label));
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
        const QColor previousColor = items_[i].color;
        const bool previouslyVisible = items_[i].visible;
        items_[i] = makeItem(parts[i]);
        items_[i].label = label;
        const QModelIndex idx = index(static_cast<int>(i));
        QList<int> roles{PositionRole, RotationRole, ScaleRole};
        if (items_[i].color != previousColor) roles.append(ColorRole);
        if (items_[i].visible != previouslyVisible) roles.append(ShownRole);
        emit dataChanged(idx, idx, roles);
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

// ---- FrustumGeometry ---------------------------------------------------------------------------
FrustumGeometry::FrustumGeometry(QObject* parent) : QQuick3DGeometry(nullptr) { setParent(parent); }

void FrustumGeometry::setShape(double hfovRad, double vfovRad, double depthM) {
    constexpr double kMinFov = 0.05, kMaxFov = 3.0;   // a camera with unset optics still gets a sane pyramid
    const camera::Frustum f = camera::frustum(sdk::Pose3{}, std::clamp(hfovRad, kMinFov, kMaxFov), std::clamp(vfovRad, kMinFov, kMaxFov), depthM);
    const QVector3D apex = toQt(f.apex);
    const QVector3D topLeft = toQt(f.farCorners[0]), topRight = toQt(f.farCorners[1]);
    const QVector3D bottomRight = toQt(f.farCorners[2]), bottomLeft = toQt(f.farCorners[3]);

    // Four sides and the far face, 18 vertices of position + normal.
    QByteArray vertices;
    vertices.reserve(18 * 6 * static_cast<qsizetype>(sizeof(float)));
    const auto triangle = [&vertices](const QVector3D& a, const QVector3D& b, const QVector3D& c) {
        const QVector3D n = QVector3D::normal(a, b, c);
        for (const QVector3D& v : {a, b, c}) {
            const float vertex[6] = {v.x(), v.y(), v.z(), n.x(), n.y(), n.z()};
            vertices.append(reinterpret_cast<const char*>(vertex), static_cast<qsizetype>(sizeof(vertex)));
        }
    };
    triangle(apex, topLeft, topRight);
    triangle(apex, topRight, bottomRight);
    triangle(apex, bottomRight, bottomLeft);
    triangle(apex, bottomLeft, topLeft);
    triangle(topLeft, bottomLeft, bottomRight);
    triangle(topLeft, bottomRight, topRight);

    clear();
    setStride(6 * static_cast<int>(sizeof(float)));
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);
    addAttribute(QQuick3DGeometry::Attribute::NormalSemantic, 3 * static_cast<int>(sizeof(float)), QQuick3DGeometry::Attribute::F32Type);
    setVertexData(vertices);
    setBounds(QVector3D(0.f, std::min(topLeft.y(), topRight.y()), std::min(bottomLeft.z(), topLeft.z())),
              QVector3D(topLeft.x(), std::max(topLeft.y(), topRight.y()), std::max(bottomLeft.z(), topLeft.z())));
    update();
}

// ---- CameraListModel -----------------------------------------------------------------------------
CameraListModel::CameraListModel(QObject* parent) : QAbstractListModel(parent) {}

int CameraListModel::rowCount(const QModelIndex& parent) const { return parent.isValid() ? 0 : static_cast<int>(items_.size()); }

QVariant CameraListModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const Item& it = items_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case IdRole: return it.id;
        case PositionRole: return it.position;
        case RotationRole: return it.rotation;
        case BodyScaleRole: return kCameraBodyM / kUnit;
        case BodyColorRole: return it.bodyColor;
        case FrustumColorRole: return it.frustumColor;
        case FrustumRole: return QVariant::fromValue<QObject*>(it.frustum);
        default: return {};
    }
}

QHash<int, QByteArray> CameraListModel::roleNames() const {
    return {{IdRole, "cameraId"},          {PositionRole, "camPosition"},         {RotationRole, "camRotation"},
            {BodyScaleRole, "bodyScale"}, {BodyColorRole, "bodyColor"}, {FrustumColorRole, "frustumColor"},
            {FrustumRole, "frustumGeometry"}};
}

void CameraListModel::fill(Item& item, const CameraOverlay& camera) {
    item.position = toQt(camera.pose.position);
    item.rotation = toQt(camera.pose.rotation);
    const QColor tone = toneColor(toneOf(camera.state));
    item.bodyColor = tone;
    item.bodyColor.setAlphaF(camera.seesTags ? kBodyOpacitySeeing : kBodyOpacityIdle);
    item.frustumColor = tone;
    item.frustumColor.setAlphaF(camera.state == CameraState::Disabled ? kFrustumOpacityDisabled
                                : camera.seesTags                     ? kFrustumOpacitySeeing
                                                                      : kFrustumOpacityIdle);
}

void CameraListModel::setCameras(const std::vector<CameraOverlay>& cameras) {
    std::vector<FrustumGeometry*> stale;
    beginResetModel();
    for (const Item& item : items_) stale.push_back(item.frustum);
    items_.clear();
    items_.reserve(cameras.size());
    for (const CameraOverlay& camera : cameras) {
        Item item;
        item.id = QString::fromStdString(camera.id);
        fill(item, camera);
        item.frustum = new FrustumGeometry(this);
        item.frustum->setShape(camera.hfovRad, camera.vfovRad, kFrustumDepthM);
        items_.push_back(std::move(item));
    }
    endResetModel();
    for (FrustumGeometry* geometry : stale) geometry->deleteLater();   // the delegates that used them are gone
}

void CameraListModel::updateCameras(const std::vector<CameraOverlay>& cameras, const std::vector<char>& changed) {
    const std::size_t n = std::min({cameras.size(), items_.size(), changed.size()});
    for (std::size_t i = 0; i < n; ++i) {
        if (!changed[i]) continue;
        fill(items_[i], cameras[i]);
        const QModelIndex idx = index(static_cast<int>(i));
        emit dataChanged(idx, idx, {PositionRole, RotationRole, BodyColorRole, FrustumColorRole});
    }
}

// ---- LabelListModel ------------------------------------------------------------------------------
LabelListModel::LabelListModel(QObject* parent) : QAbstractListModel(parent) {}

int LabelListModel::rowCount(const QModelIndex& parent) const { return parent.isValid() ? 0 : static_cast<int>(items_.size()); }

QVariant LabelListModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const Item& it = items_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case TextRole: return QString::fromStdString(it.text);
        case XRole: return it.x;
        case YRole: return it.y;
        case ShownRole: return it.shown;
        case EmphasisRole: return it.emphasized;
        case KindRole: return static_cast<int>(it.kind);
        default: return {};
    }
}

QHash<int, QByteArray> LabelListModel::roleNames() const {
    return {{TextRole, "labelText"}, {XRole, "labelX"},       {YRole, "labelY"},
            {ShownRole, "labelShown"}, {EmphasisRole, "labelEmphasis"}, {KindRole, "labelKind"}};
}

void LabelListModel::place(const std::vector<OverlayLabel>& labels, const OrbitCamera& eye, float widthPx, float heightPx, bool tagsOn,
                           bool camerasOn) {
    bool structural = labels.size() != items_.size();
    for (std::size_t i = 0; !structural && i < labels.size(); ++i) structural = labels[i].kind != items_[i].kind;
    if (structural) {
        beginResetModel();
        items_.assign(labels.size(), Item{});
        for (std::size_t i = 0; i < labels.size(); ++i) items_[i].kind = labels[i].kind;
        endResetModel();
    }
    for (std::size_t i = 0; i < labels.size(); ++i) {
        Item& item = items_[i];
        const OverlayLabel& label = labels[i];
        const LabelPlacement placement = placeLabel(label, eye, widthPx, heightPx);
        const bool layerOn = label.kind == LabelKind::Tag ? tagsOn : camerasOn;
        const bool shown = layerOn && placement.visible;
        const float x = shown ? placement.xPx : item.x;   // a hidden label keeps its last place: nothing to repaint
        const float y = shown ? placement.yPx : item.y;
        if (item.shown == shown && item.x == x && item.y == y && item.emphasized == label.emphasized && item.text == label.text) continue;
        item.shown = shown;
        item.x = x;
        item.y = y;
        item.emphasized = label.emphasized;
        item.text = label.text;
        const QModelIndex idx = index(static_cast<int>(i));
        emit dataChanged(idx, idx);
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
SceneModel::SceneModel(QObject* parent)
    : QObject(parent), field_(this), robot_(this), zones_(this), cameras_(this), tags_(this), rays_(this), labels_(this), entities_(this) {
    resetCamera();
}

void SceneModel::resetCamera() {
    // Side-on home view framing the whole field, so both alliance walls are visible.
    const float span = std::max(fieldLength_, fieldWidth_);
    camera_ = OrbitCamera{};
    camera_.target = {fieldCenter_.x(), fieldCenter_.y(), fieldCenter_.z()};
    camera_.yawRad = -OrbitCamera::degToRad(90.f);
    camera_.pitchRad = 0.4f;
    camera_.distance = span * 1.05f;
    camera_.maxDistance = std::max(camera_.maxDistance, span * 6.f);
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
    resetCamera();
    emit fieldChanged();

    zoneRig_.setScene(info_.config, info_.panels, info_.zoneKinds);
    zones_.setParts(zoneRig_.parts(), zoneRig_.labels());
    QStringList labels;
    for (const auto& l : zoneRig_.labels()) {
        const QString q = QString::fromStdString(l);
        if (!labels.contains(q)) labels << q;
    }
    if (labels != zoneLabels_) {
        zoneLabels_ = labels;
        emit zoneLabelsChanged();
    }
    updateVision(nullptr);   // cameras and tags come with the next snapshot
}

void SceneModel::setZonesVisible(bool v) {
    if (v == zonesVisible_) return;
    zonesVisible_ = v;
    emit zonesVisibleChanged();
}

void SceneModel::setCamerasVisible(bool v) {
    if (v == camerasVisible_) return;
    camerasVisible_ = v;
    refreshLabels();
    emit camerasVisibleChanged();
}

void SceneModel::setTagsVisible(bool v) {
    if (v == tagsVisible_) return;
    tagsVisible_ = v;
    refreshLabels();
    emit tagsVisibleChanged();
}

void SceneModel::setViewHeight(float heightPx) {
    viewHeightPx_ = heightPx > 1.f ? heightPx : 1.f;
    refreshLabels();
}

void SceneModel::setViewSize(float widthPx, float heightPx) {
    viewWidthPx_ = widthPx > 1.f ? widthPx : 1.f;
    setViewHeight(heightPx);
}

void SceneModel::refreshCamera() {
    const QVector3D pos = toQt(camera_.position());
    const QQuaternion rot = toQt(camera_.rotation());
    if (pos == cameraPosition_ && rot == cameraRotation_ && camera_.fovDeg == cameraFov_) return;
    cameraPosition_ = pos;
    cameraRotation_ = rot;
    cameraFov_ = camera_.fovDeg;
    emit cameraChanged();
    refreshLabels();
}

void SceneModel::refreshLabels() { labels_.place(cameraRig_.labels(), camera_, viewWidthPx_, viewHeightPx_, tagsVisible_, camerasVisible_); }

void SceneModel::updateZones(const sdk::ISimSnapshot* snapshot) {
    if (zoneRig_.update(snapshot)) zones_.updatePoses(zoneRig_.parts(), zoneRig_.changed());
}

void SceneModel::updateVision(const sdk::ISimSnapshot* snapshot) {
    if (!cameraRig_.update(snapshot)) return;
    const bool rebuilt = cameraRig_.camerasRebuilt() || cameraRig_.tagsRebuilt();
    if (cameraRig_.camerasRebuilt()) cameras_.setCameras(cameraRig_.cameras());
    else cameras_.updateCameras(cameraRig_.cameras(), cameraRig_.camerasChanged());
    if (cameraRig_.tagsRebuilt()) tags_.setParts(cameraRig_.tagParts());
    else tags_.updatePoses(cameraRig_.tagParts(), cameraRig_.tagsChanged());
    if (rebuilt) rays_.setParts(cameraRig_.rayParts());
    else rays_.updatePoses(cameraRig_.rayParts(), cameraRig_.raysChanged());
    refreshLabels();
}

void SceneModel::update(const sdk::ISimSnapshot* snapshot) {
    if (snapshot != nullptr) {
        const sdk::Pose3& pose = snapshot->chassis().pose;
        chassisPose_ = pose;
        const QVector3D pos = toQt(pose.position);
        const QQuaternion rot = toQt(pose.rotation);
        if (pos != chassisPosition_ || rot != chassisRotation_ || !robotVisible_) {
            chassisPosition_ = pos;
            chassisRotation_ = rot;
            robotVisible_ = true;
            emit poseChanged();
        }
        if (rig_.update(snapshot)) robot_.updatePoses(rig_.parts(), rig_.changed());
        entities_.update(snapshot->entityGroups());
        updateZones(snapshot);
        updateVision(snapshot);
    } else if (robotVisible_) {
        robotVisible_ = false;
        endRobotDrag();
        emit poseChanged();
        entities_.update({});
        updateVision(nullptr);
    }
}

// ---- placing the robot with the mouse ------------------------------------------------------------
namespace {
constexpr float kPickMarginM = 0.08f;                                       // forgiving: the bumpers and a little more
constexpr float kWheelNotchRad = 5.f * static_cast<float>(sdk::kPi) / 180.f;   // turning while dragging
constexpr float kSnapMoveM = 0.1f;
constexpr float kSnapYawRad = 15.f * static_cast<float>(sdk::kPi) / 180.f;

placement::Snap snapFor(int modifiers) {
    return (modifiers & Qt::ControlModifier) != 0 ? placement::Snap{kSnapMoveM, kSnapYawRad} : placement::Snap{};
}
}  // namespace

placement::LocalBox SceneModel::robotBounds() const {
    placement::LocalBox box = placement::LocalBox::empty();
    for (const ScenePart& part : rig_.parts()) {
        const sdk::Vec3 half = part.size * 0.5f;   // boxes and cylinders (axis = local Z) alike
        for (const float sx : {-1.f, 1.f})
            for (const float sy : {-1.f, 1.f})
                for (const float sz : {-1.f, 1.f}) box.include(part.pose.transformPoint({sx * half.x, sy * half.y, sz * half.z}));
    }
    return box.valid() ? box.grown(kPickMarginM) : box;
}

void SceneModel::setPlacementEnabled(bool enabled) {
    if (enabled == placementEnabled_) return;
    placementEnabled_ = enabled;
    if (!enabled) endRobotDrag();
}

bool SceneModel::robotAt(float x, float y) const {
    if (!placementEnabled_ || !robotVisible_) return false;
    const Ray ray = camera_.rayAt(x, y, viewWidthPx_, viewHeightPx_);
    return placement::rayHitDistance(ray, chassisPose_, robotBounds()).has_value();
}

bool SceneModel::beginRobotDrag(float x, float y) {
    if (!placementEnabled_ || !robotVisible_ || dragging_) return false;
    const Ray ray = camera_.rayAt(x, y, viewWidthPx_, viewHeightPx_);
    const auto hit = placement::rayHitDistance(ray, chassisPose_, robotBounds());
    if (!hit) return false;
    drag_ = placement::FloorDrag::move(sdk::toStartPose(chassisPose_), ray.origin + ray.direction * *hit);
    dragHeightM_ = chassisPose_.position.z;
    lastRequested_.reset();
    dragging_ = true;
    emit robotDragChanged();
    return true;
}

void SceneModel::dragRobot(float x, float y, int modifiers) {
    if (!dragging_) return;
    const auto pose = drag_.update(camera_.rayAt(x, y, viewWidthPx_, viewHeightPx_), snapFor(modifiers));
    if (!pose || pose == lastRequested_) return;
    lastRequested_ = pose;
    emit robotPoseRequested(pose->xM, pose->yM, dragHeightM_, pose->yawRad);
}

bool SceneModel::turnRobotDrag(float wheelSteps, int modifiers) {
    if (!dragging_) return false;
    const sdk::StartPose pose = drag_.rotateBy(wheelSteps * kWheelNotchRad, snapFor(modifiers));
    lastRequested_ = pose;
    emit robotPoseRequested(pose.xM, pose.yM, dragHeightM_, pose.yawRad);
    return true;
}

void SceneModel::endRobotDrag() {
    if (!dragging_) return;
    dragging_ = false;
    lastRequested_.reset();
    emit robotDragChanged();
}

}  // namespace wpiemu::app::runview
