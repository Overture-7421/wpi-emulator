#include "views/toast_overlay.h"

#include <QEvent>
#include <QGraphicsOpacityEffect>
#include <QLabel>
#include <QMouseEvent>
#include <QPropertyAnimation>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

namespace wpiemu::app::editor {

namespace {

class ToastFrame : public QFrame {
public:
    using QFrame::QFrame;

protected:
    void mousePressEvent(QMouseEvent*) override {
        if (dismissRequested) dismissRequested();
    }

public:
    std::function<void()> dismissRequested;
};

}  // namespace

ToastOverlay::ToastOverlay(QWidget* host) : QWidget(host), host_(host) {
    setAttribute(Qt::WA_TransparentForMouseEvents, false);
    setAttribute(Qt::WA_NoSystemBackground);
    host->installEventFilter(this);
    raise();
    show();
}

int ToastOverlay::activeCount() const { return static_cast<int>(toasts_.size()); }

void ToastOverlay::showToast(const QString& title, const QString& detail, int severity, int durationMs) {
    const QString accent = severity == 2 ? QStringLiteral("#f85149") : severity == 1 ? QStringLiteral("#d29922") : QStringLiteral("#58a6ff");
    auto* frame = new ToastFrame(this);
    frame->setObjectName(QStringLiteral("toast"));
    frame->setStyleSheet(QStringLiteral("QFrame#toast { background: palette(window); border: 1px solid palette(mid); border-left: 4px solid %1; border-radius: 6px; }").arg(accent));
    auto* v = new QVBoxLayout(frame);
    v->setContentsMargins(12, 8, 12, 8);
    v->setSpacing(1);
    auto* t = new QLabel(title, frame);
    QFont f = t->font();
    f.setBold(true);
    t->setFont(f);
    v->addWidget(t);
    if (!detail.isEmpty()) {
        auto* d = new QLabel(detail, frame);
        d->setWordWrap(true);
        d->setEnabled(false);
        v->addWidget(d);
    }
    frame->setFixedWidth(300);
    frame->adjustSize();
    toasts_.push_back(frame);
    while (toasts_.size() > 4) {
        QFrame* old = toasts_.front();
        toasts_.erase(toasts_.begin());
        old->deleteLater();
    }

    auto dismiss = [this, frame] {
        const auto it = std::find(toasts_.begin(), toasts_.end(), frame);
        if (it == toasts_.end()) return;
        toasts_.erase(it);
        frame->deleteLater();
        relayout();
    };
    frame->dismissRequested = dismiss;
    auto* fade = new QGraphicsOpacityEffect(frame);
    frame->setGraphicsEffect(fade);
    QTimer::singleShot(durationMs, frame, [frame, fade, dismiss] {
        auto* anim = new QPropertyAnimation(fade, "opacity", frame);
        anim->setDuration(350);
        anim->setStartValue(1.0);
        anim->setEndValue(0.0);
        QObject::connect(anim, &QPropertyAnimation::finished, frame, [dismiss] { dismiss(); });
        anim->start();
    });
    frame->show();
    relayout();
}

bool ToastOverlay::eventFilter(QObject* watched, QEvent* event) {
    if (watched == host_ && (event->type() == QEvent::Resize || event->type() == QEvent::Show)) relayout();
    return QWidget::eventFilter(watched, event);
}

void ToastOverlay::relayout() {
    setGeometry(host_->rect());
    int y = height() - 14;
    for (auto it = toasts_.rbegin(); it != toasts_.rend(); ++it) {
        QFrame* f = *it;
        f->adjustSize();
        y -= f->height();
        f->move(width() - f->width() - 14, y);
        f->raise();
        y -= 8;
    }
    // Only the toasts themselves should take mouse input.
    raise();
    setMask(QRegion());
    QRegion region;
    for (QFrame* f : toasts_) region += f->geometry();
    if (toasts_.empty()) region = QRegion(0, 0, 1, 1);
    setMask(region);
}

}  // namespace wpiemu::app::editor
