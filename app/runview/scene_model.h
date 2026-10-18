#pragma once

// QObject / model layer between the Qt-free scene logic (scene_parts.h, common/orbit_camera.h) and the
// Qt Quick 3D scene (qml/RunScene.qml). Everything is driven by ISimSnapshot; no business logic here.

#include <QAbstractListModel>
#include <QColor>
#include <QPalette>
#include <QQuaternion>
#include <QStringList>
#include <QVector3D>
#include <QSet>
#include <QtQuick3D/qquick3dgeometry.h>
#include <QtQuick3D/qquick3dinstancing.h>

#include <span>
#include <optional>
#include <string>
#include <vector>

#include "camera_rig.h"
#include "common/orbit_camera.h"
#include "common/robot_placement.h"
#include "run_backend.h"
#include "scene_parts.h"

namespace wpiemu::app::runview {

QColor parseColor(const std::string& css, const QColor& fallback = QColor(QStringLiteral("#808080")));

// A translucent viewing pyramid in the camera frame (apex at the origin, +X = the optical axis, SDK axes): the far corners come from
// the frustum geometry the editor shares (common/camera_frustum.h). Built once per camera; the node it hangs on carries the pose.
class FrustumGeometry : public QQuick3DGeometry {
    Q_OBJECT
public:
    explicit FrustumGeometry(QObject* parent = nullptr);
    void setShape(double hfovRad, double vfovRad, double depthM);
};

// Draws N identical entities (fuel) from one position array using GPU instancing.
class EntityInstancing : public QQuick3DInstancing {
    Q_OBJECT
public:
    explicit EntityInstancing(QObject* parent = nullptr);
    void setPositions(const std::vector<sdk::Vec3>& positions);
    int count() const { return count_; }

protected:
    QByteArray getInstanceBuffer(int* instanceCount) override;

private:
    QByteArray buffer_;
    int count_ = 0;
};

// Primitives (box / cylinder) with Qt Quick 3D built-in-mesh transforms (#Cube / #Cylinder are 100 units).
class PartListModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { NameRole = Qt::UserRole + 1, ShapeRole, PositionRole, RotationRole, ScaleRole, ColorRole, ShownRole, LabelRole };

    explicit PartListModel(QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    // `labels` (optional, same size) group parts for visibility toggles.
    void setParts(const std::vector<ScenePart>& parts, const std::vector<std::string>& labels = {});
    void updatePoses(const std::vector<ScenePart>& parts, const std::vector<char>& changed);
    void setLabelShown(const QString& label, bool shown);
    bool labelShown(const QString& label) const { return !hidden_.contains(label); }

private:
    struct Item {
        QString name;
        int shape = 0;
        QVector3D position;
        QQuaternion rotation;
        QVector3D scale{1, 1, 1};
        QColor color;
        QString label;
        bool visible = true;
    };
    static Item makeItem(const ScenePart& part);
    std::vector<Item> items_;
    QSet<QString> hidden_;
};

// One row per camera: its pose, the colour of its body and of its viewing frustum (by state, dimmer while it sees no tag) and the
// frustum geometry. Poses are SDK-frame; the QML node above converts the frame.
class CameraListModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { IdRole = Qt::UserRole + 1, PositionRole, RotationRole, BodyScaleRole, BodyColorRole, FrustumColorRole, FrustumRole };

    explicit CameraListModel(QObject* parent = nullptr);
    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    // Rebuilds the rows and a frustum geometry per camera from its field of view.
    void setCameras(const std::vector<CameraOverlay>& cameras);
    // Refreshes the rows flagged in `changed` (pose and colours only).
    void updateCameras(const std::vector<CameraOverlay>& cameras, const std::vector<char>& changed);

private:
    struct Item {
        QString id;
        QVector3D position;
        QQuaternion rotation;
        QColor bodyColor;
        QColor frustumColor;
        FrustumGeometry* frustum = nullptr;
    };
    static void fill(Item& item, const CameraOverlay& camera);
    std::vector<Item> items_;
};

// The 2D labels of the vision overlay (Qt Quick 3D has no 3D text): the anchor of each label is projected with the same orbit camera
// that drives the 3D view, so a label sits exactly on its tag or camera and is hidden behind the camera or off screen (placeLabel).
class LabelListModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { TextRole = Qt::UserRole + 1, XRole, YRole, ShownRole, EmphasisRole, KindRole };

    explicit LabelListModel(QObject* parent = nullptr);
    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    // Places every label in a `widthPx` x `heightPx` view seen by `eye`; a kind whose layer is off is hidden. Emits only for rows that
    // changed, and resets the model when the labels changed shape.
    void place(const std::vector<OverlayLabel>& labels, const OrbitCamera& eye, float widthPx, float heightPx, bool tagsOn, bool camerasOn);

private:
    struct Item {
        std::string text;
        LabelKind kind = LabelKind::Tag;
        float x = 0.f;
        float y = 0.f;
        bool shown = false;
        bool emphasized = false;
    };
    std::vector<Item> items_;
};

class EntityGroupModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { NameRole = Qt::UserRole + 1, SphereRole, ScaleRole, ColorRole, InstancingRole };

    explicit EntityGroupModel(QObject* parent = nullptr);
    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    void update(std::span<const sdk::EntityGroupSample> groups);
    int totalInstances() const;

private:
    struct Group {
        QString name;
        bool sphere = true;
        QVector3D scale{1, 1, 1};
        QColor color;
        EntityInstancing* instancing = nullptr;
    };
    std::vector<Group> groups_;
};

// Colours and fonts the QML layer needs, derived from the widget palette (light / dark aware).
class ViewportTheme : public QObject {
    Q_OBJECT
    Q_PROPERTY(QColor background READ background NOTIFY changed)
    Q_PROPERTY(QColor ground READ ground NOTIFY changed)
    Q_PROPERTY(QColor panel READ panel NOTIFY changed)
    Q_PROPERTY(QColor panelBorder READ panelBorder NOTIFY changed)
    Q_PROPERTY(QColor text READ text NOTIFY changed)
    Q_PROPERTY(QColor mutedText READ mutedText NOTIFY changed)
    Q_PROPERTY(QColor accent READ accent NOTIFY changed)
    Q_PROPERTY(QColor success READ success NOTIFY changed)
    Q_PROPERTY(QColor warning READ warning NOTIFY changed)
    Q_PROPERTY(QColor danger READ danger NOTIFY changed)
    Q_PROPERTY(bool dark READ dark NOTIFY changed)
public:
    explicit ViewportTheme(QObject* parent = nullptr);
    void setPalette(const QPalette& palette);

    QColor background() const { return background_; }
    QColor ground() const { return ground_; }
    QColor panel() const { return panel_; }
    QColor panelBorder() const { return panelBorder_; }
    QColor text() const { return text_; }
    QColor mutedText() const { return muted_; }
    QColor accent() const { return accent_; }
    QColor success() const { return QColor(QStringLiteral("#2fb36d")); }
    QColor warning() const { return QColor(QStringLiteral("#e0a526")); }
    QColor danger() const { return QColor(QStringLiteral("#e5484d")); }
    bool dark() const { return dark_; }

signals:
    void changed();

private:
    QColor background_, ground_, panel_, panelBorder_, text_, muted_, accent_;
    bool dark_ = true;
};

// Which overlay layers the viewport shows (toolbar toggles).
class ViewportOptions : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool showHud READ showHud WRITE setShowHud NOTIFY changed)
    Q_PROPERTY(bool showMechanisms READ showMechanisms WRITE setShowMechanisms NOTIFY changed)
public:
    using QObject::QObject;
    bool showHud() const { return showHud_; }
    bool showMechanisms() const { return showMechanisms_; }
    void setShowHud(bool v) { if (v != showHud_) { showHud_ = v; emit changed(); } }
    void setShowMechanisms(bool v) { if (v != showMechanisms_) { showMechanisms_ = v; emit changed(); } }

signals:
    void changed();

private:
    bool showHud_ = true;
    bool showMechanisms_ = true;
};

class SceneModel : public QObject {
    Q_OBJECT
    Q_PROPERTY(QObject* fieldParts READ fieldParts CONSTANT)
    Q_PROPERTY(QObject* robotParts READ robotParts CONSTANT)
    Q_PROPERTY(QObject* zones READ zones CONSTANT)
    Q_PROPERTY(QObject* cameras READ cameras CONSTANT)
    Q_PROPERTY(QObject* tags READ tags CONSTANT)
    Q_PROPERTY(QObject* tagRays READ tagRays CONSTANT)
    Q_PROPERTY(QObject* labels READ labels CONSTANT)
    Q_PROPERTY(QObject* entityGroups READ entityGroups CONSTANT)
    Q_PROPERTY(QVector3D chassisPosition READ chassisPosition NOTIFY poseChanged)
    Q_PROPERTY(QQuaternion chassisRotation READ chassisRotation NOTIFY poseChanged)
    Q_PROPERTY(bool robotVisible READ robotVisible NOTIFY poseChanged)
    Q_PROPERTY(QVector3D cameraPosition READ cameraPosition NOTIFY cameraChanged)
    Q_PROPERTY(QQuaternion cameraRotation READ cameraRotation NOTIFY cameraChanged)
    Q_PROPERTY(float cameraFov READ cameraFov NOTIFY cameraChanged)
    Q_PROPERTY(float fieldLength READ fieldLength NOTIFY fieldChanged)
    Q_PROPERTY(float fieldWidth READ fieldWidth NOTIFY fieldChanged)
    Q_PROPERTY(QVector3D fieldCenter READ fieldCenter NOTIFY fieldChanged)
    Q_PROPERTY(bool zonesVisible READ zonesVisible WRITE setZonesVisible NOTIFY zonesVisibleChanged)
    Q_PROPERTY(bool camerasVisible READ camerasVisible WRITE setCamerasVisible NOTIFY camerasVisibleChanged)
    Q_PROPERTY(bool tagsVisible READ tagsVisible WRITE setTagsVisible NOTIFY tagsVisibleChanged)
    Q_PROPERTY(bool robotDragging READ robotDragging NOTIFY robotDragChanged)
public:
    explicit SceneModel(QObject* parent = nullptr);

    void setSceneInfo(const RunSceneInfo& info);
    void update(const sdk::ISimSnapshot* snapshot);

    QObject* fieldParts() { return &field_; }
    QObject* robotParts() { return &robot_; }
    QObject* zones() { return &zones_; }
    QObject* cameras() { return &cameras_; }
    QObject* tags() { return &tags_; }
    QObject* tagRays() { return &rays_; }
    QObject* labels() { return &labels_; }
    QObject* entityGroups() { return &entities_; }
    PartListModel& robotModel() { return robot_; }
    EntityGroupModel& entityModel() { return entities_; }
    CameraListModel& cameraModel() { return cameras_; }
    PartListModel& tagModel() { return tags_; }
    PartListModel& rayModel() { return rays_; }
    LabelListModel& labelModel() { return labels_; }
    RobotRig& rig() { return rig_; }
    ZoneRig& zoneRig() { return zoneRig_; }
    CameraRig& cameraRig() { return cameraRig_; }
    OrbitCamera& camera() { return camera_; }

    QVector3D chassisPosition() const { return chassisPosition_; }
    QQuaternion chassisRotation() const { return chassisRotation_; }
    bool robotVisible() const { return robotVisible_; }
    QVector3D cameraPosition() const { return cameraPosition_; }
    QQuaternion cameraRotation() const { return cameraRotation_; }
    float cameraFov() const { return cameraFov_; }
    float fieldLength() const { return fieldLength_; }
    float fieldWidth() const { return fieldWidth_; }
    QVector3D fieldCenter() const { return fieldCenter_; }

    bool zonesVisible() const { return zonesVisible_; }
    void setZonesVisible(bool v);
    QStringList zoneLabels() const { return zoneLabels_; }
    void setZoneLabelShown(const QString& label, bool shown) { zones_.setLabelShown(label, shown); }
    bool zoneLabelShown(const QString& label) const { return zones_.labelShown(label); }

    // The vision layers of the View menu: cameras (bodies, frustums, labels, camera cards) and AprilTags (plates, ids). The rays between
    // them need both.
    bool camerasVisible() const { return camerasVisible_; }
    void setCamerasVisible(bool v);
    bool tagsVisible() const { return tagsVisible_; }
    void setTagsVisible(bool v);

    // Same orbit camera and input scale as the editor viewport. The home view frames the whole field.
    Q_INVOKABLE void orbit(float dx, float dy) { camera_.orbit(dx, dy); refreshCamera(); }
    Q_INVOKABLE void pan(float dx, float dy) { camera_.pan(dx, dy, viewHeightPx_); refreshCamera(); }
    Q_INVOKABLE void zoom(float steps) { camera_.zoom(steps); refreshCamera(); }
    Q_INVOKABLE void resetCamera();
    Q_INVOKABLE void setViewHeight(float heightPx);
    Q_INVOKABLE void setViewSize(float widthPx, float heightPx);

    // ---- placing the robot with the mouse (a live teleport; the drag math is common/robot_placement.h) ----
    // Offered only while a simulation runs: the page enables it. The pose handed out is upright at the height the chassis had
    // when it was picked up; wheel while dragging turns it. A Reset or a new run starts from the start pose again.
    void setPlacementEnabled(bool enabled);
    bool placementEnabled() const { return placementEnabled_; }
    bool robotDragging() const { return dragging_; }
    // Is the robot under the pixel (x, y) of the view? (hover cursor)
    Q_INVOKABLE bool robotAt(float x, float y) const;
    // Left press at (x, y): picks the robot up when the ray through that pixel hits it and placement is enabled. True = it did,
    // and the view must not pan.
    Q_INVOKABLE bool beginRobotDrag(float x, float y);
    // Moves the grabbed robot to follow the pointer. `modifiers` are Qt::KeyboardModifiers: Ctrl snaps to 10 cm and 15 degrees.
    Q_INVOKABLE void dragRobot(float x, float y, int modifiers);
    Q_INVOKABLE void endRobotDrag();
    // Wheel while dragging: turns the robot about the grabbed spot (5 degrees a notch, counter-clockwise for a positive wheel).
    // True when it consumed the wheel.
    Q_INVOKABLE bool turnRobotDrag(float wheelSteps, int modifiers);

signals:
    void poseChanged();
    void cameraChanged();
    void fieldChanged();
    void zonesVisibleChanged();
    void zoneLabelsChanged();
    void camerasVisibleChanged();
    void tagsVisibleChanged();
    // The pose the dragged robot should take: x, y, height (m) and yaw (rad). The page forwards it to the backend.
    void robotPoseRequested(double xM, double yM, double zM, double yawRad);
    void robotDragChanged();

private:
    void refreshCamera();
    void refreshLabels();
    void updateZones(const sdk::ISimSnapshot* snapshot);
    void updateVision(const sdk::ISimSnapshot* snapshot);
    placement::LocalBox robotBounds() const;   // the rig's parts in the chassis frame, a little grown for forgiving picking

    PartListModel field_;
    PartListModel robot_;
    PartListModel zones_;
    CameraListModel cameras_;
    PartListModel tags_;
    PartListModel rays_;
    LabelListModel labels_;
    EntityGroupModel entities_;
    RobotRig rig_;
    OrbitCamera camera_;
    float viewWidthPx_ = 1280.f;
    float viewHeightPx_ = 720.f;
    RunSceneInfo info_;
    ZoneRig zoneRig_;
    CameraRig cameraRig_;
    QStringList zoneLabels_;

    QVector3D chassisPosition_;
    QQuaternion chassisRotation_;
    sdk::Pose3 chassisPose_;
    bool robotVisible_ = false;
    bool placementEnabled_ = false;
    bool dragging_ = false;
    placement::FloorDrag drag_;
    float dragHeightM_ = 0.f;   // height the chassis had when picked up: kept while dragging
    std::optional<sdk::StartPose> lastRequested_;
    QVector3D cameraPosition_;
    QQuaternion cameraRotation_;
    float cameraFov_ = 45.f;
    float fieldLength_ = 16.46f;
    float fieldWidth_ = 8.23f;
    QVector3D fieldCenter_{8.2f, 4.1f, 0.f};
    bool zonesVisible_ = true;
    bool camerasVisible_ = true;
    bool tagsVisible_ = true;
};

}  // namespace wpiemu::app::runview
