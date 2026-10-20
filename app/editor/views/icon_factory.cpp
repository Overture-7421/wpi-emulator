#include "views/icon_factory.h"

#include <QFont>
#include <QPainter>
#include <QPixmap>

namespace wpiemu::app::editor {

namespace {

constexpr int kSize = 16;
constexpr int kScale = 2;

QPixmap canvas() {
    QPixmap pm(kSize * kScale, kSize * kScale);
    pm.setDevicePixelRatio(kScale);
    pm.fill(Qt::transparent);
    return pm;
}

}  // namespace

QColor IconFactory::stateColor(SlotState state) {
    switch (state) {
        case SlotState::Mapped: return QColor("#3fb950");
        case SlotState::Unmapped: return QColor("#d29922");
        case SlotState::NotDetected: return QColor("#d29922");
        case SlotState::Incompatible:
        case SlotState::AliasConflict: return QColor("#f85149");
        case SlotState::OptionalEmpty: return QColor("#8b949e");
    }
    return QColor("#8b949e");
}

QColor IconFactory::colorForCategory(const QString& category) {
    static const QHash<QString, QColor> known{{"Intake", QColor("#e9a23b")}, {"Shooter", QColor("#d1495b")}, {"Climber", QColor("#6aa6d8")},
                                              {"Arm", QColor("#8e7dd1")},    {"Generic", QColor("#7fb685")}};
    return known.value(category, QColor("#8b949e"));
}

QIcon IconFactory::dot(const QColor& color, bool hollow) const {
    const QString key = QStringLiteral("dot:%1:%2").arg(color.name()).arg(hollow);
    if (cache_.contains(key)) return cache_.value(key);
    QPixmap pm = canvas();
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(color, 1.6));
    p.setBrush(hollow ? Qt::NoBrush : QBrush(color));
    p.drawEllipse(QRectF(4, 4, 8, 8));
    p.end();
    const QIcon icon(pm);
    cache_.insert(key, icon);
    return icon;
}

QIcon IconFactory::status(SlotState state) const { return dot(stateColor(state), state == SlotState::OptionalEmpty || state == SlotState::NotDetected); }

QIcon IconFactory::deviceStatus(bool live, bool mapped, int aliasState) const {
    if (aliasState == 2) return dot(QColor("#f85149"), false);
    if (!live) return dot(QColor("#8b949e"), true);
    return mapped ? dot(QColor("#3fb950"), false) : dot(QColor("#d29922"), false);
}

QIcon IconFactory::severity(int s) const {
    return dot(s == 2 ? QColor("#f85149") : s == 1 ? QColor("#d29922") : QColor("#58a6ff"), false);
}

QIcon IconFactory::glyph(const QString& text, const QColor& color) const {
    const QString key = QStringLiteral("glyph:%1:%2").arg(text, color.name());
    if (cache_.contains(key)) return cache_.value(key);
    QPixmap pm = canvas();
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(color);
    p.drawRoundedRect(QRectF(1, 1, 14, 14), 3.5, 3.5);
    QFont f = p.font();
    f.setBold(true);
    f.setPixelSize(9);
    p.setFont(f);
    p.setPen(color.lightness() > 150 ? QColor("#1b1f24") : QColor("#ffffff"));
    p.drawText(QRectF(1, 1, 14, 14), Qt::AlignCenter, text);
    p.end();
    const QIcon icon(pm);
    cache_.insert(key, icon);
    return icon;
}

QIcon IconFactory::node(NodeKind kind) const {
    switch (kind) {
        case NodeKind::Robot: return glyph(QStringLiteral("R"), QColor("#58a6ff"));
        case NodeKind::Chassis: return glyph(QStringLiteral("C"), QColor("#6f86a3"));
        case NodeKind::Com: return glyph(QStringLiteral("+"), QColor("#ff9f1c"));
        case NodeKind::Drivetrain: return glyph(QStringLiteral("D"), QColor("#8e7dd1"));
        case NodeKind::Wheel: return glyph(QStringLiteral("W"), QColor("#4b525c"));
        case NodeKind::Mechanism: return glyph(QStringLiteral("M"), QColor("#d9a441"));
        case NodeKind::Shape: return glyph(QStringLiteral("\xE2\x96\xAB"), QColor("#8f98a5"));
        case NodeKind::Volume: return glyph(QStringLiteral("V"), QColor("#44cc88"));
        case NodeKind::Plugin: return glyph(QStringLiteral("G"), QColor("#cc55bb"));
        case NodeKind::Zone: return glyph(QStringLiteral("Z"), QColor("#33b5c9"));
        case NodeKind::Camera: return glyph(QStringLiteral("\xE2\x97\x89"), QColor("#9be15d"));
        case NodeKind::Sensor: return glyph(QStringLiteral("\xE2\x97\x8E"), QColor("#f2a65a"));
        case NodeKind::StartPosition: return glyph(QStringLiteral("\xE2\x9A\x91"), QColor("#7aa2f7"));
        case NodeKind::Group: return {};
        case NodeKind::Slot: break;
    }
    return {};
}

}  // namespace wpiemu::app::editor
