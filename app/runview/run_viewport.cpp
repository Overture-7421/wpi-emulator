#include "run_viewport.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QProgressBar>
#include <QPushButton>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickWidget>
#include <QVBoxLayout>

#include "status_chip.h"

namespace wpiemu::app::runview {

StateOverlay::StateOverlay(QWidget* parent) : QWidget(parent) {
    setObjectName(QStringLiteral("runview.stateOverlay"));
    setAttribute(Qt::WA_NoMousePropagation);
    icon_ = new QLabel(this);
    icon_->setAlignment(Qt::AlignCenter);
    title_ = new QLabel(this);
    title_->setAlignment(Qt::AlignCenter);
    QFont f = title_->font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() * 1.5);
    title_->setFont(f);
    detail_ = new QLabel(this);
    detail_->setAlignment(Qt::AlignCenter);
    detail_->setWordWrap(true);
    detail_->setFixedWidth(440);
    busy_ = new QProgressBar(this);
    busy_->setRange(0, 0);
    busy_->setTextVisible(false);
    busy_->setMaximumWidth(260);
    busy_->setMaximumHeight(6);
    start_ = new QPushButton(tr("Start robot"), this);
    start_->setDefault(true);
    start_->setMinimumWidth(130);
    restart_ = new QPushButton(tr("Restart robot"), this);
    restart_->setMinimumWidth(130);
    console_ = new QPushButton(tr("Show console"), this);

    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    buttons->addWidget(start_);
    buttons->addWidget(restart_);
    buttons->addWidget(console_);
    buttons->addStretch(1);

    auto* layout = new QVBoxLayout(this);
    layout->addStretch(1);
    layout->addWidget(icon_);
    layout->addWidget(title_);
    layout->addWidget(detail_, 0, Qt::AlignHCenter);
    layout->addWidget(busy_, 0, Qt::AlignHCenter);
    layout->addSpacing(8);
    layout->addLayout(buttons);
    layout->addStretch(1);
    layout->setSpacing(8);

    connect(start_, &QPushButton::clicked, this, &StateOverlay::startRequested);
    connect(restart_, &QPushButton::clicked, this, &StateOverlay::restartRequested);
    connect(console_, &QPushButton::clicked, this, &StateOverlay::showConsoleRequested);
    setVisible(false);
}

void StateOverlay::setPresentation(const StatusPresentation& p) {
    setVisible(p.showOverlay);
    if (!p.showOverlay) return;
    const QColor c = toneColor(p.tone);
    const QString glyph = p.tone == Tone::Error ? QStringLiteral("⚠") : (p.tone == Tone::Info ? QStringLiteral("⏳") : QStringLiteral("▶"));
    icon_->setText(QStringLiteral("<span style=\"font-size:36px;color:%1\">%2</span>").arg(c.name(), glyph));
    title_->setText(p.title);
    detail_->setText(p.detail);
    detail_->setFixedHeight(detail_->heightForWidth(detail_->width()) + 4);
    detail_->setVisible(!p.detail.isEmpty());
    busy_->setVisible(p.tone == Tone::Info);
    start_->setVisible(p.offerStart);
    restart_->setVisible(p.offerRestart);
    console_->setVisible(p.offerShowConsole);
    if (p.offerStart) start_->setFocus();
    else if (p.offerRestart) restart_->setFocus();
}

void StateOverlay::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    QColor bg = palette().color(QPalette::Window);
    bg.setAlpha(225);
    painter.fillRect(rect(), bg);
}

RunViewport::RunViewport(SceneModel& scene, HudModel& hud, MechanismOverlayModel& mechanisms, ViewportTheme& theme,
                         ViewportOptions& options, QWidget* parent)
    : QWidget(parent) {
    setObjectName(QStringLiteral("runview.viewport"));
    quick_ = new QQuickWidget(this);
    quick_->setResizeMode(QQuickWidget::SizeRootObjectToView);
    QQmlContext* ctx = quick_->rootContext();
    ctx->setContextProperty(QStringLiteral("scene"), &scene);
    ctx->setContextProperty(QStringLiteral("hud"), &hud);
    ctx->setContextProperty(QStringLiteral("mechanisms"), &mechanisms);
    ctx->setContextProperty(QStringLiteral("theme"), &theme);
    ctx->setContextProperty(QStringLiteral("options"), &options);
    quick_->setSource(QUrl(QStringLiteral("qrc:/wpiemu/runview/qml/RunScene.qml")));

    errorLabel_ = new QLabel(this);
    errorLabel_->setWordWrap(true);
    errorLabel_->setAlignment(Qt::AlignCenter);
    errorLabel_->setVisible(false);
    if (quick_->status() == QQuickWidget::Error) {
        for (const auto& e : quick_->errors()) error_ += e.toString() + QLatin1Char('\n');
        errorLabel_->setText(tr("The 3D view could not be loaded:\n%1").arg(error_));
        errorLabel_->setVisible(true);
        quick_->setVisible(false);
    }

    overlay_ = new StateOverlay(this);
    quick_->setGeometry(rect());
}

bool RunViewport::sceneLoaded() const { return quick_->status() == QQuickWidget::Ready; }

QImage RunViewport::grabScene() { return quick_->grabFramebuffer(); }

void RunViewport::resizeEvent(QResizeEvent*) {
    quick_->setGeometry(rect());
    errorLabel_->setGeometry(rect());
    overlay_->setGeometry(rect());
    overlay_->raise();
}

}  // namespace wpiemu::app::runview
