#include "viewmodel/viewport_controller.h"

#include <QVariantMap>

#include <algorithm>
#include <cmath>

#include "model/config_ops.h"
#include "model/property_groups.h"
#include "model/quat_utils.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

using sdk::Pose3;
using sdk::Quat;
using sdk::Vec3;

namespace {

constexpr float kPi = static_cast<float>(sdk::kPi);

QVector3D q3(const Vec3& v) { return SceneModel::toQt(v); }

float component(const Vec3& v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

void setComponent(Vec3& v, int axis, float value) {
    if (axis == 0) v.x = value;
    else if (axis == 1) v.y = value;
    else v.z = value;
}

Vec3 unitAxis(int axis) { return axis == 0 ? Vec3{1.f, 0.f, 0.f} : axis == 1 ? Vec3{0.f, 1.f, 0.f} : Vec3{0.f, 0.f, 1.f}; }

}  // namespace

ViewportController::ViewportController(EditorController& editor, QObject* parent)
    : QObject(parent), editor_(editor), scene_(this) {
    connect(&editor_.document(), &EditorDocument::changed, this, [this] {
        rebuildScene();
        rebuildGizmo();
        Q_EMIT hintChanged();
        Q_EMIT infoChanged();
        Q_EMIT transformFieldsChanged();
    });
    connect(&editor_.selection(), &SelectionState::changed, this, [this] { onSelectionChanged(); });
    connect(&editor_.settings(), &EditorSettings::changed, this, [this] {
        Q_EMIT infoChanged();
        Q_EMIT transformFieldsChanged();
    });
    rebuildDecor();
    rebuildScene();
    rebuildGizmo();
}

// ---- scene ---------------------------------------------------------------------------------------
void ViewportController::rebuildScene() {
    const SceneContent content = buildScene(editor_.document().config(), editor_.document().catalog(), theme_);
    scene_.setContent(content, theme_.accent);
    const SelectionRef sel = editor_.selection().current();
    const SelectionRef highlight = (sel.kind == SelKind::Primitive || sel.kind == SelKind::Overlay) ? sel : formTargetOf(editor_.document().config(), sel);
    scene_.setHighlight(highlight, theme_.accent);
}

void ViewportController::setTheme(const SceneTheme& theme) {
    theme_ = theme;
    rebuildDecor();
    rebuildScene();
    rebuildGizmo();
    Q_EMIT themeChanged();
}

void ViewportController::onSelectionChanged() {
    const SelectionRef sel = editor_.selection().current();
    const SelectionRef highlight = (sel.kind == SelKind::Primitive || sel.kind == SelKind::Overlay) ? sel : formTargetOf(editor_.document().config(), sel);
    scene_.setHighlight(highlight, theme_.accent);
    rebuildGizmo();
    Q_EMIT infoChanged();
    Q_EMIT transformFieldsChanged();
}

QVariantMap ViewportController::entry(const QString& source, const Vec3& center, const QQuaternion& rotation, const QVector3D& scale,
                                      const QColor& color, double opacity, const QString& pickId) const {
    QVariantMap m;
    m.insert(QStringLiteral("source3d"), source);
    m.insert(QStringLiteral("pos"), q3(center));
    m.insert(QStringLiteral("rot"), rotation);
    m.insert(QStringLiteral("scl"), scale);
    m.insert(QStringLiteral("tint"), color);
    m.insert(QStringLiteral("alpha"), opacity);
    m.insert(QStringLiteral("pickId"), pickId);
    return m;
}

QVariantMap ViewportController::boxEntry(const Vec3& center, const Quat& rotation, const Vec3& size, const QColor& color, double opacity,
                                         const QString& pickId) const {
    return entry(QStringLiteral("#Cube"), center, SceneModel::toQt(rotation), QVector3D(size.x / 100.f, size.y / 100.f, size.z / 100.f), color,
                 opacity, pickId);
}

QVariantMap ViewportController::lineEntry(const Vec3& a, const Vec3& b, float thickness, const QColor& color, double opacity,
                                          const QString& pickId) const {
    return boxEntry((a + b) * 0.5f, rotationFromX(b - a), Vec3{sdk::length(b - a), thickness, thickness}, color, opacity, pickId);
}

QVariantMap ViewportController::cylinderEntry(const Vec3& a, const Vec3& b, float diameter, const QColor& color, const QString& pickId) const {
    const float len = sdk::length(b - a);
    const QQuaternion rot = QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), q3(sdk::normalized(b - a)));
    return entry(QStringLiteral("#Cylinder"), (a + b) * 0.5f, rot, QVector3D(diameter / 100.f, len / 100.f, diameter / 100.f), color, 1.0, pickId);
}

QVariantMap ViewportController::coneEntry(const Vec3& base, const Vec3& tip, float diameter, const QColor& color, const QString& pickId) const {
    const float len = sdk::length(tip - base);
    const QQuaternion rot = QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), q3(sdk::normalized(tip - base)));
    return entry(QStringLiteral("#Cone"), (base + tip) * 0.5f, rot, QVector3D(diameter / 100.f, len / 100.f, diameter / 100.f), color, 1.0, pickId);
}

void ViewportController::rebuildDecor() {
    decor_.clear();
    decor_.push_back(boxEntry({0.f, 0.f, -0.011f}, Quat{}, {14.f, 14.f, 0.02f}, theme_.floor));
    for (int i = -10; i <= 10; ++i) {
        const float p = static_cast<float>(i) * 0.5f;
        const bool major = i % 2 == 0;
        const QColor c = major ? theme_.gridMajor : theme_.gridMinor;
        const float w = major ? 0.008f : 0.004f;
        decor_.push_back(boxEntry({0.f, p, 0.001f}, Quat{}, {10.f, w, 0.002f}, c));
        decor_.push_back(boxEntry({p, 0.f, 0.001f}, Quat{}, {w, 10.f, 0.002f}, c));
    }
    decor_.push_back(lineEntry({0.f, 0.f, 0.004f}, {0.9f, 0.f, 0.004f}, 0.016f, theme_.axisX));
    decor_.push_back(lineEntry({0.f, 0.f, 0.004f}, {0.f, 0.9f, 0.004f}, 0.016f, theme_.axisY));
    decor_.push_back(lineEntry({0.f, 0.f, 0.004f}, {0.f, 0.f, 0.9f}, 0.016f, theme_.axisZ));
}

// ---- gizmo ---------------------------------------------------------------------------------------
SelectionRef ViewportController::transformSelection() const {
    const SelectionRef sel = editor_.selection().current();
    const sdk::RobotConfig& c = editor_.document().config();
    switch (sel.kind) {
        case SelKind::Slot: return c.findMechanism(sel.owner) != nullptr ? SelectionRef::mechanism(sel.owner) : SelectionRef{};
        case SelKind::Chassis:
        case SelKind::ChassisCom:
        case SelKind::Wheel:
        case SelKind::Primitive:
        case SelKind::Mechanism:
        case SelKind::Zone:
        case SelKind::Camera:
        case SelKind::Overlay: return sel;
        default: return {};
    }
}

std::optional<GizmoMode> ViewportController::effectiveMode() const {
    const auto target = readTransform(editor_.document().config(), editor_.document().catalog(), transformSelection());
    if (!target) return std::nullopt;
    const EditableTransform& t = target->transform;
    auto allowed = [&](GizmoMode m) { return m == GizmoMode::Move ? t.canMove : m == GizmoMode::Rotate ? t.canRotate : t.canScale; };
    if (allowed(mode_)) return mode_;
    for (GizmoMode m : {GizmoMode::Move, GizmoMode::Scale, GizmoMode::Rotate})
        if (allowed(m)) return m;
    return std::nullopt;
}

void ViewportController::setGizmoMode(GizmoMode mode) {
    mode_ = mode;
    rebuildGizmo();
    Q_EMIT modeChanged();
    Q_EMIT transformFieldsChanged();
}

float ViewportController::gizmoLength(const Vec3& origin) const {
    return std::max(0.08f, sdk::length(origin - camera_.position()) * 0.16f);
}

QColor ViewportController::axisColor(int axis, bool active) const {
    const QColor base = axis == 0 ? theme_.axisX : axis == 1 ? theme_.axisY : theme_.axisZ;
    return active ? QColor(255, 220, 80) : base;
}

void ViewportController::rebuildGizmo() {
    gizmo_.clear();
    edges_.clear();

    // Outline of everything that belongs to the selection.
    for (const SceneItem& item : scene_.items()) {
        if (!scene_.isHighlighted(item) || item.role == ItemRole::Link) continue;
        const Vec3 h = item.size * 0.5f;
        auto corner = [&](int sx, int sy, int sz) {
            return item.world.transformPoint({h.x * static_cast<float>(sx), h.y * static_cast<float>(sy), h.z * static_cast<float>(sz)});
        };
        for (int s1 = -1; s1 <= 1; s1 += 2)
            for (int s2 = -1; s2 <= 1; s2 += 2) {
                edges_.push_back(lineEntry(corner(-1, s1, s2), corner(1, s1, s2), 0.006f, theme_.accent));
                edges_.push_back(lineEntry(corner(s1, -1, s2), corner(s1, 1, s2), 0.006f, theme_.accent));
                edges_.push_back(lineEntry(corner(s1, s2, -1), corner(s1, s2, 1), 0.006f, theme_.accent));
            }
    }

    gizmoActive_ = false;
    const auto mode = effectiveMode();
    if (mode) {
        const auto target = readTransform(editor_.document().config(), editor_.document().catalog(), transformSelection());
        const Vec3 origin = target->world().position;
        const float L = gizmoLength(origin);
        gizmoActive_ = true;
        gizmoOrigin_ = origin;
        gizmoLength_ = L;
        const Quat frame = *mode == GizmoMode::Scale ? target->world().rotation : target->parentWorld.rotation;
        for (int i = 0; i < 3; ++i) {
            const Vec3 axis = sdk::normalized(frame.rotate(unitAxis(i)));
            const QString id = QStringLiteral("h:%1:%2").arg(*mode == GizmoMode::Move ? 'm' : *mode == GizmoMode::Rotate ? 'r' : 's').arg(i);
            const bool active = drag_.kind == DragKind::Gizmo && drag_.handle == id;
            const QColor color = axisColor(i, active);
            if (*mode == GizmoMode::Move) {
                gizmo_.push_back(cylinderEntry(origin, origin + axis * (L * 0.8f), L * 0.05f, color, id));
                gizmo_.push_back(coneEntry(origin + axis * (L * 0.8f), origin + axis * L, L * 0.17f, color, id));
            } else if (*mode == GizmoMode::Scale) {
                const float reach = target->transform.size.x > 0.f ? std::max(component(target->transform.size, i) * 0.5f, 0.f) : 0.f;
                const Vec3 handle = origin + axis * (reach + L * 0.35f);
                gizmo_.push_back(cylinderEntry(origin + axis * reach, handle, L * 0.04f, color, id));
                gizmo_.push_back(boxEntry(handle, frame, Vec3{L * 0.13f, L * 0.13f, L * 0.13f}, color, 1.0, id));
            } else {
                // Ring in the plane perpendicular to the axis: a polygon of short boxes.
                Vec3 u = std::fabs(axis.z) < 0.9f ? sdk::normalized(sdk::cross(axis, Vec3{0.f, 0.f, 1.f})) : Vec3{1.f, 0.f, 0.f};
                const Vec3 v = sdk::cross(axis, u);
                const int segments = 40;
                const float radius = L * 0.85f;
                for (int s = 0; s < segments; ++s) {
                    const float a0 = 2.f * kPi * static_cast<float>(s) / segments;
                    const float a1 = 2.f * kPi * static_cast<float>(s + 1) / segments;
                    const Vec3 p0 = origin + (u * std::cos(a0) + v * std::sin(a0)) * radius;
                    const Vec3 p1 = origin + (u * std::cos(a1) + v * std::sin(a1)) * radius;
                    gizmo_.push_back(lineEntry(p0, p1, L * 0.035f, color, 1.0, id));
                }
            }
        }
    }
    Q_EMIT gizmoChanged();
}

// ---- camera --------------------------------------------------------------------------------------
QVariantList ViewportController::axisIndicator() const {
    const CameraBasis b = camera_.basis();
    QVariantList out;
    const char* labels[] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i) {
        const Vec3 e = unitAxis(i);
        QVariantMap m;
        m.insert(QStringLiteral("label"), QString::fromLatin1(labels[i]));
        m.insert(QStringLiteral("x"), static_cast<double>(sdk::dot(e, b.right)));
        m.insert(QStringLiteral("y"), static_cast<double>(sdk::dot(e, b.up)));
        m.insert(QStringLiteral("depth"), static_cast<double>(sdk::dot(e, b.forward)));
        m.insert(QStringLiteral("color"), i == 0 ? theme_.axisX : i == 1 ? theme_.axisY : theme_.axisZ);
        out.push_back(m);
    }
    return out;
}

void ViewportController::refreshGizmoScale() {
    if (!gizmoActive_) return;
    const float L = gizmoLength(gizmoOrigin_);
    if (std::fabs(L - gizmoLength_) > 0.04f * gizmoLength_) rebuildGizmo();
}

void ViewportController::setView(const QString& preset) {
    camera_.preset(preset.toUtf8().constData());
    Q_EMIT cameraChanged();
    refreshGizmoScale();
}

void ViewportController::resetCamera() {
    camera_ = OrbitCamera{};
    Q_EMIT cameraChanged();
    refreshGizmoScale();
}

void ViewportController::frameSelection() {
    const sdk::RobotConfig& c = editor_.document().config();
    Vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
    bool any = false;
    auto grow = [&](const Vec3& p) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        any = true;
    };
    for (const SceneItem& item : scene_.items()) {
        const bool selected = scene_.isHighlighted(item);
        if (!selected && !(editor_.selection().current().kind == SelKind::Robot || editor_.selection().current().isNone())) continue;
        if (item.role == ItemRole::Link || item.role == ItemRole::Overlay) continue;
        const Vec3 h = item.size * 0.5f;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                    grow(item.world.transformPoint({h.x * static_cast<float>(sx), h.y * static_cast<float>(sy), h.z * static_cast<float>(sz)}));
    }
    (void)c;
    if (!any) return;
    const Vec3 center = (lo + hi) * 0.5f;
    camera_.frame(center, std::max(0.25f, sdk::length(hi - lo) * 0.5f));
    Q_EMIT cameraChanged();
    refreshGizmoScale();
}

void ViewportController::setViewSize(qreal width, qreal height) {
    viewW_ = std::max<qreal>(width, 1);
    viewH_ = std::max<qreal>(height, 1);
}

void ViewportController::wheel(qreal angleDeltaY) {
    camera_.zoom(static_cast<float>(angleDeltaY / 120.0));
    Q_EMIT cameraChanged();
    refreshGizmoScale();
}

// ---- input ---------------------------------------------------------------------------------------
SelectionRef ViewportController::chooseItem(const QStringList& hitIds) const {
    const SceneItem* best = nullptr;
    for (const QString& id : hitIds) {
        if (!id.startsWith(QLatin1String("i:"))) continue;
        const SceneItem* item = scene_.itemById(id.mid(2).toStdString());
        if (item == nullptr || item->ref.isNone()) continue;
        if (best == nullptr) best = item;
        if (item->role != ItemRole::Overlay) {
            best = item;
            break;
        }
    }
    if (best == nullptr) return {};
    // Drill-down: clicking a mechanism's / the chassis' shape selects the owner first and the shape once
    // the owner is already selected.
    if (best->ref.kind == SelKind::Primitive && !best->group.isNone()) {
        const SelectionRef cur = editor_.selection().current();
        const bool inGroup = cur == best->group || (cur.kind == SelKind::Primitive && cur.owner == best->ref.owner) ||
                             formTargetOf(editor_.document().config(), cur) == best->group;
        return inGroup ? best->ref : best->group;
    }
    return best->ref;
}

void ViewportController::press(qreal x, qreal y, int button, int modifiers, const QStringList& hitIds) {
    drag_ = Drag{};
    drag_.last = drag_.start = QPointF(x, y);
    const bool alt = (modifiers & Qt::AltModifier) != 0;
    const bool shift = (modifiers & Qt::ShiftModifier) != 0;

    if (button == Qt::LeftButton && !alt) {
        for (const QString& id : hitIds)
            if (id.startsWith(QLatin1String("h:")) && beginGizmoDrag(id, x, y)) return;
        const SelectionRef picked = chooseItem(hitIds);
        if (!picked.isNone()) {
            editor_.selection().set(picked);
            return;
        }
        drag_.kind = DragKind::Pan;
        drag_.clickToDeselect = true;
    } else if (button == Qt::RightButton || (button == Qt::LeftButton && alt)) {
        drag_.kind = shift ? DragKind::Pan : DragKind::Orbit;
    } else if (button == Qt::MiddleButton) {
        drag_.kind = DragKind::Pan;
    }
}

void ViewportController::move(qreal x, qreal y, int, int modifiers) {
    const QPointF pos(x, y);
    const QPointF delta = pos - drag_.last;
    if (!drag_.moved && (pos - drag_.start).manhattanLength() > 3) drag_.moved = true;
    switch (drag_.kind) {
        case DragKind::Orbit:
            camera_.orbit(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
            Q_EMIT cameraChanged();
            break;
        case DragKind::Pan:
            camera_.pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()), static_cast<float>(viewH_));
            Q_EMIT cameraChanged();
            break;
        case DragKind::Gizmo:
            if (drag_.moved) updateGizmoDrag(x, y, (modifiers & Qt::ControlModifier) != 0);
            break;
        case DragKind::None: break;
    }
    drag_.last = pos;
    if (drag_.kind == DragKind::Orbit || drag_.kind == DragKind::Pan) refreshGizmoScale();
}

void ViewportController::release(qreal, qreal, int) {
    if (drag_.kind == DragKind::Gizmo) endGizmoDrag();
    else if (drag_.clickToDeselect && !drag_.moved) editor_.selection().set(SelectionRef::robot());
    drag_ = Drag{};
}

bool ViewportController::snapActive(bool toggle) const { return editor_.settings().snapEnabled() != toggle; }

bool ViewportController::beginGizmoDrag(const QString& handle, qreal x, qreal y) {
    const QStringList parts = handle.split(QLatin1Char(':'));
    if (parts.size() != 3 || parts[0] != QLatin1String("h")) return false;
    const SelectionRef sel = transformSelection();
    const auto target = readTransform(editor_.document().config(), editor_.document().catalog(), sel);
    if (!target) return false;

    Drag d;
    d.kind = DragKind::Gizmo;
    d.last = d.start = QPointF(x, y);
    d.handle = handle;
    d.axis = std::clamp(parts[2].toInt(), 0, 2);
    d.mode = parts[1] == QLatin1String("m") ? GizmoMode::Move : parts[1] == QLatin1String("r") ? GizmoMode::Rotate : GizmoMode::Scale;
    d.target = sel;
    d.initial = *target;
    d.axisLocal = unitAxis(d.axis);
    const Quat frame = d.mode == GizmoMode::Scale ? target->world().rotation : target->parentWorld.rotation;
    d.axisWorld = sdk::normalized(frame.rotate(d.axisLocal));
    d.origin = target->world().position;
    d.session = QStringLiteral("g%1").arg(++dragCounter_);

    const Ray ray = camera_.rayAt(static_cast<float>(x), static_cast<float>(y), static_cast<float>(viewW_), static_cast<float>(viewH_));
    if (d.mode == GizmoMode::Rotate) {
        const auto hit = intersectPlane(ray, d.origin, d.axisWorld);
        if (!hit) return false;
        d.vec0 = *hit - d.origin;
    } else {
        const auto t = closestParamOnAxis(ray, d.origin, d.axisWorld);
        if (!t) return false;
        d.t0 = *t;
    }
    drag_ = d;
    rebuildGizmo();
    return true;
}

void ViewportController::updateGizmoDrag(qreal x, qreal y, bool snapToggle) {
    if (drag_.kind != DragKind::Gizmo) return;
    const Ray ray = camera_.rayAt(static_cast<float>(x), static_cast<float>(y), static_cast<float>(viewW_), static_cast<float>(viewH_));
    EditableTransform t = drag_.initial.transform;
    const bool snap = snapActive(snapToggle);
    const float step = static_cast<float>(editor_.settings().moveStepM());

    if (drag_.mode == GizmoMode::Rotate) {
        const auto hit = intersectPlane(ray, drag_.origin, drag_.axisWorld);
        if (!hit) return;
        const Vec3 v = *hit - drag_.origin;
        float angle = std::atan2(sdk::dot(drag_.axisWorld, sdk::cross(drag_.vec0, v)), sdk::dot(drag_.vec0, v));
        if (snap) angle = degToRadF(snapValue(radToDegF(angle), static_cast<float>(editor_.settings().rotateStepDeg())));
        t.local.rotation = rotatedAbout(drag_.initial.transform.local.rotation, drag_.axisLocal, angle);
    } else {
        const auto param = closestParamOnAxis(ray, drag_.origin, drag_.axisWorld);
        if (!param) return;
        const float delta = *param - drag_.t0;
        if (drag_.mode == GizmoMode::Move) {
            float value = component(drag_.initial.transform.local.position, drag_.axis) + delta;
            if (snap) value = snapValue(value, step);
            setComponent(t.local.position, drag_.axis, value);
        } else {
            float value = std::max(0.005f, component(drag_.initial.transform.size, drag_.axis) + 2.f * delta);
            if (snap) value = std::max(step, snapValue(value, step));
            setComponent(t.size, drag_.axis, value);
        }
    }
    editor_.document().setTransform(drag_.target, t, drag_.session);
}

void ViewportController::endGizmoDrag() {
    if (drag_.kind != DragKind::Gizmo) return;
    drag_.kind = DragKind::None;
    drag_.handle.clear();
    rebuildGizmo();
}

// ---- numeric entry -------------------------------------------------------------------------------
TransformFields ViewportController::transformFields() const {
    TransformFields f;
    const SelectionRef sel = transformSelection();
    const auto target = readTransform(editor_.document().config(), editor_.document().catalog(), sel);
    const auto mode = effectiveMode();
    if (!target || !mode) return f;
    f.valid = true;
    f.mode = *mode;
    f.editable = true;
    f.targetName = describeSelection(editor_.document().config(), editor_.document().catalog(), sel);
    if (*mode == GizmoMode::Move) f.value = target->transform.local.position;
    else if (*mode == GizmoMode::Scale) f.value = target->transform.size;
    else {
        const Euler e = toEuler(target->transform.local.rotation);
        f.value = Vec3{radToDegF(e.rollRad), radToDegF(e.pitchRad), radToDegF(e.yawRad)};
    }
    return f;
}

bool ViewportController::setTransformComponent(int axis, double value) {
    const SelectionRef sel = transformSelection();
    auto target = readTransform(editor_.document().config(), editor_.document().catalog(), sel);
    const auto mode = effectiveMode();
    if (!target || !mode || axis < 0 || axis > 2) return false;
    EditableTransform t = target->transform;
    const float v = static_cast<float>(value);
    if (*mode == GizmoMode::Move) setComponent(t.local.position, axis, v);
    else if (*mode == GizmoMode::Scale) setComponent(t.size, axis, std::max(v, 0.005f));
    else {
        Euler e = toEuler(t.local.rotation);
        if (axis == 0) e.rollRad = degToRadF(v);
        else if (axis == 1) e.pitchRad = degToRadF(v);
        else e.yawRad = degToRadF(v);
        t.local.rotation = fromEuler(e);
    }
    return editor_.document().setTransform(sel, t);
}

QString ViewportController::hintText() const {
    if (!editor_.document().config().mechanisms.empty()) return {};
    return ETR("Add your first mechanism: double-click a template in the palette, then drop detected devices onto its slots.");
}

QString ViewportController::infoText() const {
    const TransformFields f = transformFields();
    if (!f.valid) return ETR("Click a part to select it \xC2\xB7 left/middle-drag pans \xC2\xB7 right-drag orbits \xC2\xB7 wheel zooms");
    return f.targetName;
}

}  // namespace wpiemu::app::editor
