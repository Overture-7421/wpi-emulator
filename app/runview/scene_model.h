#pragma once

// QObject / model layer between the Qt-free scene logic (scene_parts.h, common/orbit_camera.h) and the
// Qt Quick 3D scene (qml/RunScene.qml). Everything is driven by ISimSnapshot; no business logic here.

#include <QAbstractListModel>
#include <QColor>
#include <QPalette>
#include <QQuaternion>
#include <QStringList>
#include <QUrl>
#include <QVector3D>
#include <QSet>
#include <QtQuick3D/qquick3dinstancing.h>

#include <span>
#include <string>
#include <vector>

#include "common/orbit_camera.h"
#include "contract/runtime_host.h"
#include "scene_parts.h"

namespace wpiemu::app::runview {

QColor parseColor(const std::string& css, const QColor& fallback = QColor(QStringLiteral("#808080")));

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
    };
    static Item makeItem(const ScenePart& part);
    std::vector<Item> items_;
    QSet<QString> hidden_;
};

// One textured quad per AprilTag: the official 36h11 image on the tag's face plane. Static after setDecals(); ids
// without a shipped image are skipped (the tag then shows its bare panel).
class TagMarkerModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { PositionRole = Qt::UserRole + 1, RotationRole, SizeRole, ImageRole };

    explicit TagMarkerModel(QObject* parent = nullptr);
    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    void setDecals(const std::vector<TagDecal>& decals);

private:
    struct Item {
        QVector3D position;
        QQuaternion rotation;
        float size = 0.f;
        QUrl image;
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
    Q_PROPERTY(int topCenterInset READ topCenterInset WRITE setTopCenterInset NOTIFY changed)
public:
    using QObject::QObject;
    bool showHud() const { return showHud_; }
    void setShowHud(bool v) { if (v != showHud_) { showHud_ = v; emit changed(); } }
    // Pixels the top-centre HUD stack leaves free for a widget drawn over the view (the Driver Station clock).
    int topCenterInset() const { return topCenterInset_; }
    void setTopCenterInset(int v) { if (v != topCenterInset_) { topCenterInset_ = v; emit changed(); } }

signals:
    void changed();

private:
    bool showHud_ = true;
    int topCenterInset_ = 0;
};

class SceneModel : public QObject {
    Q_OBJECT
    Q_PROPERTY(QObject* fieldParts READ fieldParts CONSTANT)
    Q_PROPERTY(QObject* robotParts READ robotParts CONSTANT)
    Q_PROPERTY(QObject* zones READ zones CONSTANT)
    Q_PROPERTY(QObject* tags READ tags CONSTANT)
    Q_PROPERTY(QObject* tagMarkers READ tagMarkers CONSTANT)
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
    Q_PROPERTY(bool tagsVisible READ tagsVisible WRITE setTagsVisible NOTIFY tagsVisibleChanged)
public:
    explicit SceneModel(QObject* parent = nullptr);

    void setSceneInfo(const HostScene& info);
    void update(const sdk::ISimSnapshot* snapshot);

    QObject* fieldParts() { return &field_; }
    QObject* robotParts() { return &robot_; }
    QObject* zones() { return &zones_; }
    QObject* tags() { return &tags_; }
    QObject* tagMarkers() { return &tagMarkers_; }
    QObject* entityGroups() { return &entities_; }
    PartListModel& robotModel() { return robot_; }
    EntityGroupModel& entityModel() { return entities_; }
    RobotRig& rig() { return rig_; }
    ZoneRig& zoneRig() { return zoneRig_; }
    TagRig& tagRig() { return tagRig_; }
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
    // Field AprilTags; the View menu's "AprilTags".
    bool tagsVisible() const { return tagsVisible_; }
    void setTagsVisible(bool v);
    QStringList zoneLabels() const { return zoneLabels_; }
    void setZoneLabelShown(const QString& label, bool shown) { zones_.setLabelShown(label, shown); }
    bool zoneLabelShown(const QString& label) const { return zones_.labelShown(label); }

    // Same orbit camera and input scale as the editor viewport. The home view frames the whole field.
    Q_INVOKABLE void orbit(float dx, float dy) { camera_.orbit(dx, dy); refreshCamera(); }
    Q_INVOKABLE void pan(float dx, float dy) { camera_.pan(dx, dy, viewHeightPx_); refreshCamera(); }
    Q_INVOKABLE void zoom(float steps) { camera_.zoom(steps); refreshCamera(); }
    Q_INVOKABLE void resetCamera();
    Q_INVOKABLE void setViewHeight(float heightPx) { viewHeightPx_ = heightPx > 1.f ? heightPx : 1.f; }

signals:
    void poseChanged();
    void cameraChanged();
    void fieldChanged();
    void zonesVisibleChanged();
    void tagsVisibleChanged();
    void zoneLabelsChanged();

private:
    void refreshCamera();
    void updateZones(const sdk::ISimSnapshot* snapshot);
    void updateVision(const sdk::ISimSnapshot* snapshot);

    PartListModel field_;
    PartListModel robot_;
    PartListModel zones_;
    PartListModel tags_;
    TagMarkerModel tagMarkers_;
    EntityGroupModel entities_;
    RobotRig rig_;
    OrbitCamera camera_;
    float viewHeightPx_ = 720.f;
    HostScene info_;
    ZoneRig zoneRig_;
    TagRig tagRig_;
    QStringList zoneLabels_;

    QVector3D chassisPosition_;
    QQuaternion chassisRotation_;
    bool robotVisible_ = false;
    QVector3D cameraPosition_;
    QQuaternion cameraRotation_;
    float cameraFov_ = 45.f;
    float fieldLength_ = 16.46f;
    float fieldWidth_ = 8.23f;
    QVector3D fieldCenter_{8.2f, 4.1f, 0.f};
    bool zonesVisible_ = true;
    bool tagsVisible_ = true;
};

}  // namespace wpiemu::app::runview
