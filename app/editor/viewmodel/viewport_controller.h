#pragma once

// View-model of the 3D viewport. Owns the orbit camera, builds the scene / gizmo / selection
// outlines for QML, and turns mouse input into model edits (select, move / rotate / scale with
// snapping, numeric entry). QML is a thin renderer: it forwards pointer events and draws lists.

#include <QColor>
#include <QObject>
#include <QPointF>
#include <QQuaternion>
#include <QStringList>
#include <QVariantList>
#include <QVector3D>

#include <optional>

#include "common/robot_placement.h"
#include "model/scene_theme.h"
#include "model/transform_access.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/orbit_camera.h"
#include "viewmodel/scene_model.h"

namespace wpiemu::app::editor {

enum class GizmoMode { Move, Rotate, Scale };

struct TransformFields {
    bool valid = false;
    GizmoMode mode = GizmoMode::Move;
    sdk::Vec3 value;          // Move/Scale: metres, Rotate: degrees (roll, pitch, yaw)
    bool editable = false;
    QString targetName;
};

class ViewportController : public QObject {
    Q_OBJECT
    Q_PROPERTY(QObject* sceneModel READ sceneModelObject CONSTANT)
    Q_PROPERTY(QVector3D cameraPosition READ cameraPosition NOTIFY cameraChanged)
    Q_PROPERTY(QQuaternion cameraRotation READ cameraRotation NOTIFY cameraChanged)
    Q_PROPERTY(qreal fieldOfView READ fieldOfView CONSTANT)
    Q_PROPERTY(QVariantList gizmoHandles READ gizmoHandles NOTIFY gizmoChanged)
    Q_PROPERTY(QVariantList selectionEdges READ selectionEdges NOTIFY gizmoChanged)
    Q_PROPERTY(QVariantList decor READ decor NOTIFY themeChanged)
    Q_PROPERTY(QVariantList tagLabels READ tagLabels NOTIFY tagLabelsChanged)
    Q_PROPERTY(QVariantList axisIndicator READ axisIndicator NOTIFY cameraChanged)
    Q_PROPERTY(QColor backgroundColor READ backgroundColor NOTIFY themeChanged)
    Q_PROPERTY(QColor textColor READ textColor NOTIFY themeChanged)
    Q_PROPERTY(QColor accentColor READ accentColor NOTIFY themeChanged)
    Q_PROPERTY(QString hintText READ hintText NOTIFY hintChanged)
    Q_PROPERTY(QString infoText READ infoText NOTIFY infoChanged)

public:
    ViewportController(EditorController& editor, QObject* parent = nullptr);

    // ---- QML-facing ----
    QObject* sceneModelObject() { return &scene_; }
    QVector3D cameraPosition() const { return SceneModel::toQt(camera_.position()); }
    QQuaternion cameraRotation() const { return SceneModel::toQt(camera_.rotation()); }
    qreal fieldOfView() const { return static_cast<qreal>(camera_.fovDeg); }
    QVariantList gizmoHandles() const { return gizmo_; }
    QVariantList selectionEdges() const { return edges_; }
    QVariantList decor() const { return decor_; }
    // The AprilTag ids to write on screen: {x, y, text, color} of every tag plate in the view, nothing while the tags are hidden.
    QVariantList tagLabels() const { return tagLabels_; }
    QVariantList axisIndicator() const;
    QColor backgroundColor() const { return theme_.background; }
    QColor textColor() const { return theme_.text; }
    QColor accentColor() const { return theme_.accent; }
    QString hintText() const;
    QString infoText() const;

    Q_INVOKABLE void setViewSize(qreal width, qreal height);
    // `hitIds`: pickId of every object under the cursor, nearest first ("h:<handle>" gizmo, "i:<item>" scene).
    Q_INVOKABLE void press(qreal x, qreal y, int button, int modifiers, const QStringList& hitIds);
    Q_INVOKABLE void move(qreal x, qreal y, int buttons, int modifiers);
    Q_INVOKABLE void release(qreal x, qreal y, int button);
    Q_INVOKABLE void wheel(qreal angleDeltaY);

    // ---- C++ API (toolbar, shortcuts, tests) ----
    GizmoMode gizmoMode() const { return mode_; }
    void setGizmoMode(GizmoMode mode);
    std::optional<GizmoMode> effectiveMode() const;
    void setView(const QString& preset);   // "front", "side", "top", "iso"
    void frameSelection();
    void resetCamera();
    void setTheme(const SceneTheme& theme);
    const SceneTheme& theme() const { return theme_; }
    OrbitCamera& camera() { return camera_; }
    SceneModel& scene() { return scene_; }
    // What the scene shows besides the robot: the tags follow the toolbar toggle, and a selected camera or the vision settings.
    SceneOptions sceneOptions() const;

    TransformFields transformFields() const;
    bool setTransformComponent(int axis, double value);

    // Exposed for tests: the same entry points the mouse uses.
    bool beginGizmoDrag(const QString& handle, qreal x, qreal y);
    void updateGizmoDrag(qreal x, qreal y, bool snapToggle);
    void endGizmoDrag();
    // Start pose marker: `itemId` is the scene item under the cursor (kStartPoseFootprintId moves the pose, the nose turns it).
    bool beginStartPoseDrag(const QString& itemId, qreal x, qreal y);
    void updateStartPoseDrag(qreal x, qreal y, bool snapToggle);
    void endStartPoseDrag();
    SelectionRef chooseItem(const QStringList& hitIds) const;

Q_SIGNALS:
    void cameraChanged();
    void gizmoChanged();
    void themeChanged();
    void hintChanged();
    void infoChanged();
    void modeChanged();
    void transformFieldsChanged();
    void tagLabelsChanged();

private:
    enum class DragKind { None, Orbit, Pan, Gizmo, StartPose };
    struct Drag {
        DragKind kind = DragKind::None;
        QPointF last;
        QPointF start;
        bool moved = false;
        bool clickToDeselect = false;
        // gizmo drag
        GizmoMode mode = GizmoMode::Move;
        int axis = 0;
        SelectionRef target;
        TransformTarget initial;
        sdk::Vec3 axisWorld;
        sdk::Vec3 axisLocal;
        sdk::Vec3 origin;
        float t0 = 0.f;
        float angle0 = 0.f;
        sdk::Vec3 vec0;
        QString session;
        QString handle;
        // start pose drag (moving the footprint or turning it by the nose)
        placement::FloorDrag floor;
    };

    void rebuildScene();
    void rebuildDecor();
    void rebuildGizmo();
    void rebuildTagLabels();
    void refreshGizmoScale();   // camera moved: rebuild only when the on-screen size would visibly change
    void onSelectionChanged();
    SelectionRef transformSelection() const;
    const SceneItem* pickItem(const QStringList& hitIds) const;   // nearest solid item under the cursor, else the nearest volume
    float gizmoLength(const sdk::Vec3& origin) const;
    bool snapActive(bool toggle) const;
    QVariantMap entry(const QString& source, const sdk::Vec3& center, const QQuaternion& rotation, const QVector3D& scale,
                      const QColor& color, double opacity = 1.0, const QString& pickId = {}) const;
    QVariantMap boxEntry(const sdk::Vec3& center, const sdk::Quat& rotation, const sdk::Vec3& size, const QColor& color,
                         double opacity = 1.0, const QString& pickId = {}) const;
    QVariantMap lineEntry(const sdk::Vec3& a, const sdk::Vec3& b, float thickness, const QColor& color, double opacity = 1.0,
                          const QString& pickId = {}) const;
    QVariantMap cylinderEntry(const sdk::Vec3& a, const sdk::Vec3& b, float diameter, const QColor& color, const QString& pickId) const;
    QVariantMap coneEntry(const sdk::Vec3& base, const sdk::Vec3& tip, float diameter, const QColor& color, const QString& pickId) const;
    QColor axisColor(int axis, bool active) const;

    EditorController& editor_;
    SceneModel scene_;
    SceneTheme theme_;
    OrbitCamera camera_;
    GizmoMode mode_ = GizmoMode::Move;
    QVariantList gizmo_;
    QVariantList edges_;
    QVariantList decor_;
    QVariantList tagLabels_;
    SceneOptions sceneOptions_;   // what the scene was last built with
    qreal viewW_ = 800;
    qreal viewH_ = 600;
    Drag drag_;
    int dragCounter_ = 0;
    float gizmoLength_ = 0.f;
    sdk::Vec3 gizmoOrigin_;
    bool gizmoActive_ = false;
};

}  // namespace wpiemu::app::editor
