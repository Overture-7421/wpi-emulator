#include "ui/widget_support.h"

#include <QComboBox>
#include <QEvent>
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

namespace {

// Qt focuses a WheelFocus widget on the first wheel notch and a combo box accepts the event, which stops the enclosing
// scroll area from ever seeing it. Ignoring the event here lets it propagate to the parent.
class WheelGuard : public QObject {
public:
    explicit WheelGuard(QWidget* input) : QObject(input), input_(input) {}

    bool eventFilter(QObject* watched, QEvent* event) override {
        if (watched != input_ || event->type() != QEvent::Wheel) return false;
        if (qobject_cast<QComboBox*>(input_) == nullptr && input_->hasFocus()) return false;
        event->ignore();
        return true;
    }

private:
    QWidget* input_;
};

}  // namespace

void setQuietWheel(QWidget* input) {
    input->setFocusPolicy(Qt::StrongFocus);
    input->installEventFilter(new WheelGuard(input));
}

void flashWidget(QWidget* widget, const QString& styleSheet) {
    widget->setStyleSheet(styleSheet);
    QTimer::singleShot(kFlashMs, widget, [widget] { widget->setStyleSheet(QString()); });
}

}  // namespace wpiemu::app::ui
