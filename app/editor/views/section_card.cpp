#include "views/section_card.h"

#include <QFont>
#include <QPainter>
#include <QPixmap>
#include <QVBoxLayout>

namespace wpiemu::app::editor {

QColor SectionCard::colorFor(SectionRole role) {
    switch (role) {
        case SectionRole::Drive: return QColor("#e9a23b");
        case SectionRole::Configure: return QColor("#5b8cff");
        case SectionRole::Placement: return QColor("#3ecf8e");
        case SectionRole::Shapes: return QColor("#b58be0");
        case SectionRole::Info: return QColor("#8b949e");
        case SectionRole::Activation: return QColor("#e0607e");
    }
    return QColor("#8b949e");
}

QString SectionCard::glyphFor(SectionRole role) {
    switch (role) {
        case SectionRole::Drive: return QStringLiteral("D");
        case SectionRole::Configure: return QStringLiteral("C");
        case SectionRole::Placement: return QStringLiteral("P");
        case SectionRole::Shapes: return QStringLiteral("S");
        case SectionRole::Info: return QStringLiteral("i");
        case SectionRole::Activation: return QStringLiteral("A");
    }
    return {};
}

const char* SectionCard::nameFor(SectionRole role) {
    switch (role) {
        case SectionRole::Drive: return "drive";
        case SectionRole::Configure: return "configure";
        case SectionRole::Placement: return "placement";
        case SectionRole::Shapes: return "shapes";
        case SectionRole::Info: return "info";
        case SectionRole::Activation: return "activation";
    }
    return "info";
}

namespace {

QIcon glyphIcon(const QString& text, const QColor& color) {
    constexpr int size = 18, scale = 2;
    QPixmap pm(size * scale, size * scale);
    pm.setDevicePixelRatio(scale);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(color);
    p.drawRoundedRect(QRectF(1, 1, size - 2, size - 2), 4, 4);
    QFont f = p.font();
    f.setBold(true);
    f.setPixelSize(11);
    p.setFont(f);
    p.setPen(QColor("#10151c"));
    p.drawText(QRectF(1, 1, size - 2, size - 2), Qt::AlignCenter, text);
    p.end();
    return QIcon(pm);
}

}  // namespace

SectionCard::SectionCard(const QString& title, SectionRole role, QWidget* body, bool collapsed, QWidget* parent)
    : QFrame(parent), role_(role), body_(body), header_(new QToolButton(this)) {
    const QColor c = colorFor(role);
    setObjectName(QStringLiteral("card.") + QLatin1String(nameFor(role)));
    setFrameShape(QFrame::StyledPanel);
    setProperty("sectionCard", true);
    setStyleSheet(QStringLiteral(
                      "QFrame[sectionCard=\"true\"] { border: 1px solid %1; border-radius: 6px; }"
                      "QFrame[sectionCard=\"true\"] QToolButton[sectionHeader=\"true\"] { border: none; border-bottom: 1px solid %1; "
                      "border-left: 5px solid %1; border-top-left-radius: 5px; border-top-right-radius: 5px; "
                      "background: rgba(%2, %3, %4, 100); text-align: left; padding: 6px 8px; font-weight: bold; }"
                      "")
                      .arg(c.name()).arg(c.red()).arg(c.green()).arg(c.blue()));

    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    header_->setProperty("sectionHeader", true);
    header_->setText(title);
    header_->setIcon(glyphIcon(glyphFor(role), c));
    header_->setIconSize(QSize(18, 18));
    header_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    header_->setCheckable(true);
    header_->setChecked(!collapsed);
    header_->setAutoRaise(false);
    header_->setCursor(Qt::PointingHandCursor);
    header_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    v->addWidget(header_);

    body_->setParent(this);
    auto* inner = new QWidget(this);
    auto* il = new QVBoxLayout(inner);
    il->setContentsMargins(10, 8, 10, 10);
    il->addWidget(body_);
    v->addWidget(inner);
    inner->setVisible(!collapsed);
    connect(header_, &QToolButton::toggled, this, [inner](bool open) { inner->setVisible(open); });
}

}  // namespace wpiemu::app::editor
