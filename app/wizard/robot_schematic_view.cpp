#include "wizard/robot_schematic_view.h"

#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace wpiemu::app {

RobotSchematicView::RobotSchematicView(const UiResources& ui, QWidget* parent) : QWidget(parent), ui_(ui) {
    setAccessibleName(tr("Robot schematic"));
    connect(&ui_, &UiResources::themeChanged, this, qOverload<>(&QWidget::update));
    setBasics(basics_, units_);
}

void RobotSchematicView::setBasics(const RobotBasics& basics, const UnitFormatter& units) {
    basics_ = basics;
    units_ = units;
    model_ = buildRobotConfig(basics_, RobotBuildContext{});
    update();
}

void RobotSchematicView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const ThemeTokens& tk = ui_.tokens();

    const QRectF area = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QPen(tk.border, 1));
    p.setBrush(tk.altBase);
    p.drawRoundedRect(area, 10, 10);

    const double length = basics_.frameLengthM;
    const double width = basics_.frameWidthM;
    const double bump = basics_.bumperThicknessM;
    const double outerL = length + 2 * bump;
    const double outerW = width + 2 * bump;
    const double margin = 46.0;
    const double scale = std::max(1.0, std::min((area.width() - 2 * margin) / outerW, (area.height() - 2 * margin) / outerL));
    const QPointF centre = area.center();
    const auto map = [&](double x, double y) { return QPointF(centre.x() - y * scale, centre.y() - x * scale); };
    const auto rectFor = [&](double x0, double y0, double x1, double y1) {
        return QRectF(map(x1, y0), map(x0, y1)).normalized();
    };

    // faint 0.5 m floor grid
    QColor grid = tk.border;
    grid.setAlphaF(0.45f);
    p.setPen(QPen(grid, 1));
    for (double g = 0.5; g * scale < std::max(area.width(), area.height()); g += 0.5) {
        for (const double s : {-1.0, 1.0}) {
            p.drawLine(QPointF(centre.x() + s * g * scale, area.top() + 1), QPointF(centre.x() + s * g * scale, area.bottom() - 1));
            p.drawLine(QPointF(area.left() + 1, centre.y() + s * g * scale), QPointF(area.right() - 1, centre.y() + s * g * scale));
        }
    }

    // bumpers (ring) then frame
    const QColor bumperColor = basics_.alliance == Alliance::Red ? QColor("#c0392b") : QColor("#2b5fd0");
    if (bump > 0.0) {
        QPainterPath ring;
        ring.addRoundedRect(rectFor(-outerL / 2, -outerW / 2, outerL / 2, outerW / 2), 7, 7);
        QPainterPath hole;
        hole.addRect(rectFor(-length / 2, -width / 2, length / 2, width / 2));
        p.setPen(Qt::NoPen);
        p.setBrush(bumperColor);
        p.drawPath(ring.subtracted(hole));
    }
    p.setPen(QPen(tk.text, 1.5));
    p.setBrush(tk.card);
    p.drawRoundedRect(rectFor(-length / 2, -width / 2, length / 2, width / 2), 3, 3);

    // wheels
    for (const sdk::WheelSpec& w : model_.drivetrain.spec.wheels) {
        const double wl = w.radiusM * 2.0, ww = std::max<double>(w.widthM, 0.035);
        const QRectF r = rectFor(w.positionM.x - wl / 2, w.positionM.y - ww / 2, w.positionM.x + wl / 2, w.positionM.y + ww / 2);
        p.setPen(QPen(tk.text, 1));
        p.setBrush(w.kind == sdk::WheelKind::Mecanum ? tk.accent : tk.mutedText);
        p.drawRoundedRect(r, 2, 2);
        if (w.kind == sdk::WheelKind::Mecanum) {
            p.setPen(QPen(tk.accentText, 1.2));
            const double sign = w.tire.tractionAngleRad >= 0 ? 1.0 : -1.0;
            p.drawLine(r.center() + QPointF(-r.width() / 2.5, sign * r.height() / 2.5), r.center() + QPointF(r.width() / 2.5, -sign * r.height() / 2.5));
        } else if (w.kind == sdk::WheelKind::Swerve) {
            p.setPen(QPen(tk.accent, 2));
            p.drawLine(r.center() + QPointF(0, r.height() * 0.28), r.center() - QPointF(0, r.height() * 0.28));
            p.setBrush(tk.accent);
            p.drawEllipse(r.center(), 2.5, 2.5);
        }
    }

    // centre of mass marker
    p.setPen(QPen(tk.mutedText, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(centre, 5, 5);
    p.drawLine(centre + QPointF(-8, 0), centre + QPointF(8, 0));
    p.drawLine(centre + QPointF(0, -8), centre + QPointF(0, 8));

    // driving direction
    const QPointF tip = map(outerL / 2 + 0.16, 0);
    QPainterPath arrow;
    arrow.moveTo(tip);
    arrow.lineTo(tip + QPointF(-7, 12));
    arrow.lineTo(tip + QPointF(7, 12));
    arrow.closeSubpath();
    p.setPen(Qt::NoPen);
    p.setBrush(tk.accent);
    p.drawPath(arrow);
    QFont small = font();
    small.setPointSizeF(std::max(7.5, font().pointSizeF() - 1.5));
    p.setFont(small);
    p.setPen(tk.accent);
    p.drawText(QRectF(tip.x() - 40, tip.y() - 18, 80, 16), Qt::AlignCenter, tr("FRONT"));

    // dimension labels
    p.setPen(tk.mutedText);
    const QPointF below = map(-outerL / 2 - 0.04, 0);
    p.drawText(QRectF(below.x() - 70, below.y() + 2, 140, 18), Qt::AlignCenter, tr("%1 wide").arg(units_.formatLength(width)));
    p.save();
    const QPointF side = map(0, -outerW / 2 - 0.04);
    p.translate(side.x() + 4, side.y());
    p.rotate(90);
    p.drawText(QRectF(-70, 0, 140, 18), Qt::AlignCenter, tr("%1 long").arg(units_.formatLength(length)));
    p.restore();

    const QString legend = tr("%1 · %2 wheels · %3")
                               .arg(basics_.isSwerve() ? tr("Swerve") : basics_.isMecanum() ? tr("Mecanum") : tr("Tank"))
                               .arg(basics_.wheelCount())
                               .arg(units_.formatMass(basics_.massKg, 0));
    p.drawText(QRectF(area.left() + 12, area.bottom() - 26, area.width() - 24, 18), Qt::AlignLeft | Qt::AlignVCenter, legend);
}

}  // namespace wpiemu::app
