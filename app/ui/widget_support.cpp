#include "ui/widget_support.h"

#include <QFont>
#include <QLabel>
#include <QTimer>

namespace wpiemu::app::ui {

QLabel* mutedLabel(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text, parent);
    l->setWordWrap(true);
    l->setEnabled(false);
    return l;
}

QLabel* captionLabel(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text, parent);
    l->setEnabled(false);
    QFont f = l->font();
    f.setPointSizeF(f.pointSizeF() * 0.85);
    l->setFont(f);
    return l;
}

QLabel* headingLabel(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text.toUpper(), parent);
    QFont f = l->font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() * 0.8);
    f.setLetterSpacing(QFont::PercentageSpacing, 108);
    l->setFont(f);
    l->setEnabled(false);
    return l;
}

void flashWidget(QWidget* widget, const QString& styleSheet) {
    widget->setStyleSheet(styleSheet);
    QTimer::singleShot(kFlashMs, widget, [widget] { widget->setStyleSheet(QString()); });
}

}  // namespace wpiemu::app::ui
