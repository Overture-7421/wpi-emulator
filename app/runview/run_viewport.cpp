#include "run_viewport.h"

#include <QComboBox>
#include <QEvent>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMenu>
#include <functional>
#include <QLabel>
#include <QIcon>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include "status_chip.h"

namespace wpiemu::app::runview {

namespace {

QIcon paintedIcon(const std::function<void(QPainter&)>& draw) {
    const qreal dpr = 2.0;
    QPixmap pm(QSize(20, 20) * dpr);
    pm.setDevicePixelRatio(dpr);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(Qt::white, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    draw(p);
    return QIcon(pm);
}

// Shared look of the floating controls (restart, speed, home) over the 3D scene.
constexpr const char* kOverlayButtonStyle =
    "QToolButton, QComboBox, QLineEdit { color: white; background: rgba(20,24,32,170);"
    " border: 1px solid rgba(255,255,255,60); border-radius: 6px; }"
    "QToolButton:hover, QComboBox:hover { background: rgba(60,70,90,220); }"
    "QToolButton:disabled, QComboBox:disabled, QLineEdit:disabled { color: rgba(255,255,255,90); }"
    "QToolButton::menu-indicator { image: none; }"
    "QToolButton[seg=\"left\"] { border-top-right-radius: 0; border-bottom-right-radius: 0; }"
    "QToolButton[seg=\"right\"] { border-top-left-radius: 0; border-bottom-left-radius: 0; border-left: none; }"
    "QToolButton { padding: 0; font-weight: 600; }"
    "QMenu { background: #1c2028; color: white; border: 1px solid rgba(255,255,255,60); }"
    "QMenu::item:selected { background: #3b6fb6; }";
}  // namespace

QIcon reloadIcon() {
    return paintedIcon([](QPainter& p) {
        QPainterPath arc;
        arc.arcMoveTo(QRectF(3.5, 3.5, 13, 13), 40);
        arc.arcTo(QRectF(3.5, 3.5, 13, 13), 40, 280);
        p.drawPath(arc);
        QPainterPath head;   // arrowhead at the arc start
        head.moveTo(13.6, 1.8);
        head.lineTo(13.6, 6.2);
        head.lineTo(9.4, 6.2);
        p.drawPath(head);
    });
}

QIcon chevronIcon() {
    return paintedIcon([](QPainter& p) {
        QPainterPath path;
        path.moveTo(4.5, 7.5);
        path.lineTo(10, 13);
        path.lineTo(15.5, 7.5);
        p.drawPath(path);
    });
}

namespace {
QIcon houseIcon() {
    return paintedIcon([](QPainter& p) {
        QPainterPath path;
        path.moveTo(2.5, 9.5);
        path.lineTo(10, 3);
        path.lineTo(17.5, 9.5);
        path.moveTo(4.5, 8.5);
        path.lineTo(4.5, 17);
        path.lineTo(8.3, 17);
        path.lineTo(8.3, 12);
        path.lineTo(11.7, 12);
        path.lineTo(11.7, 17);
        path.lineTo(15.5, 17);
        path.lineTo(15.5, 8.5);
        p.drawPath(path);
    });
}
}  // namespace

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
    restart_ = new QPushButton(tr("Restart robot"), this);
    restart_->setDefault(true);
    restart_->setMinimumWidth(130);
    console_ = new QPushButton(tr("Show console"), this);

    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
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
    restart_->setVisible(p.offerRestart);
    console_->setVisible(p.offerShowConsole);
    if (p.offerRestart) restart_->setFocus();
}

void StateOverlay::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    QColor bg = palette().color(QPalette::Window);
    bg.setAlpha(225);
    painter.fillRect(rect(), bg);
}

RunViewport::RunViewport(SceneModel& scene, HudModel& hud, MechanismOverlayModel& mechanisms, CameraHudModel& cameras, ViewportTheme& theme,
                         ViewportOptions& options, QWidget* parent)
    : QWidget(parent) {
    setObjectName(QStringLiteral("runview.viewport"));
    quick_ = new QQuickWidget(this);
    quick_->setResizeMode(QQuickWidget::SizeRootObjectToView);
    QQmlContext* ctx = quick_->rootContext();
    ctx->setContextProperty(QStringLiteral("scene"), &scene);
    ctx->setContextProperty(QStringLiteral("hud"), &hud);
    ctx->setContextProperty(QStringLiteral("mechanisms"), &mechanisms);
    ctx->setContextProperty(QStringLiteral("cameraHud"), &cameras);
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

    matchClock_ = new QLabel(this);
    matchClock_->setObjectName(QStringLiteral("runview.matchTimer"));
    matchClock_->setAlignment(Qt::AlignCenter);
    matchClock_->setStyleSheet(QStringLiteral(
        "QLabel { color: white; background: rgba(20,24,32,170); border: 1px solid rgba(255,255,255,60); border-radius: 8px;"
        " padding: 4px 16px; font-size: 20px; font-weight: 700; }"));
    matchClock_->hide();

    controls_ = new QWidget(this);
    controls_->setObjectName(QStringLiteral("runview.viewControls"));
    controls_->setStyleSheet(QString::fromLatin1(kOverlayButtonStyle));
    controlsLayout_ = new QHBoxLayout(controls_);
    controlsLayout_->setContentsMargins(0, 0, 0, 0);
    controlsLayout_->setSpacing(6);

    home_ = new QToolButton(controls_);
    home_->setObjectName(QStringLiteral("runview.homeButton"));
    home_->setIcon(houseIcon());
    home_->setIconSize(QSize(20, 20));
    home_->setToolTip(tr("Home view (Ctrl+0, or double-click the view)"));
    home_->setAccessibleName(tr("Home view"));
    home_->setCursor(Qt::PointingHandCursor);
    home_->setFixedSize(34, 34);
    controlsLayout_->addWidget(home_);
    connect(home_, &QToolButton::clicked, this, &RunViewport::homeRequested);
    quick_->setGeometry(rect());
}

void RunViewport::addControl(QWidget* w) {
    w->setParent(controls_);
    controlsLayout_->insertWidget(controlsLayout_->count() - 1, w);
    w->show();
    placeControls();
}

void RunViewport::addRestartControl(QAction* restart, QMenu* menu) {
    auto* main = new QToolButton;
    main->setObjectName(QStringLiteral("runview.restartButton"));
    main->setDefaultAction(restart);
    main->setToolButtonStyle(Qt::ToolButtonIconOnly);
    main->setIconSize(QSize(20, 20));
    main->setFixedSize(34, 34);
    main->setProperty("seg", QStringLiteral("left"));
    main->setCursor(Qt::PointingHandCursor);

    auto* more = new QToolButton;
    more->setObjectName(QStringLiteral("runview.restartMenuButton"));
    more->setIcon(chevronIcon());
    more->setIconSize(QSize(18, 18));
    more->setFixedSize(20, 34);
    more->setMenu(menu);
    more->setPopupMode(QToolButton::InstantPopup);
    more->setToolTip(tr("More restart options"));
    more->setProperty("seg", QStringLiteral("right"));
    more->setCursor(Qt::PointingHandCursor);

    auto* group = new QWidget;
    auto* gl = new QHBoxLayout(group);
    gl->setContentsMargins(0, 0, 0, 0);
    gl->setSpacing(0);
    gl->addWidget(main);
    gl->addWidget(more);
    addControl(group);
}

void RunViewport::addSpeedControl(QComboBox* combo) {
    // The combo stays the model (and is kept hidden); a menu button with centred text is what the user sees.
    combo->setParent(this);
    combo->hide();
    auto* button = new QToolButton;
    button->setObjectName(QStringLiteral("runview.speedButton"));
    button->setToolTip(combo->toolTip());
    button->setFixedSize(58, 34);
    button->setCursor(Qt::PointingHandCursor);
    button->setPopupMode(QToolButton::InstantPopup);
    auto* menu = new QMenu(button);
    for (int i = 0; i < combo->count(); ++i) {
        QAction* a = menu->addAction(combo->itemText(i));
        connect(a, &QAction::triggered, combo, [combo, i] {
            combo->setCurrentIndex(i);
            emit combo->activated(i);
        });
    }
    button->setMenu(menu);
    const auto sync = [button, combo] { button->setText(combo->currentText()); button->setEnabled(combo->isEnabled()); };
    connect(combo, &QComboBox::currentIndexChanged, button, sync);
    speedCombo_ = combo;
    speedSync_ = sync;
    combo->installEventFilter(this);
    sync();
    addControl(button);
}

bool RunViewport::eventFilter(QObject* obj, QEvent* e) {
    if (obj == speedCombo_ && e->type() == QEvent::EnabledChange && speedSync_) speedSync_();
    return QWidget::eventFilter(obj, e);
}

void RunViewport::setMatchClock(const QString& text) {
    matchClock_->setText(text);
    matchClock_->setVisible(!text.isEmpty());
    placeControls();
}

void RunViewport::placeControls() {
    matchClock_->adjustSize();
    matchClock_->move((width() - matchClock_->width()) / 2, 12);
    matchClock_->raise();
    controls_->adjustSize();
    controls_->move(width() - controls_->width() - 12, height() - controls_->height() - 12);
    controls_->raise();
}

bool RunViewport::sceneLoaded() const { return quick_->status() == QQuickWidget::Ready; }

QImage RunViewport::grabScene() { return quick_->grabFramebuffer(); }

void RunViewport::resizeEvent(QResizeEvent*) {
    quick_->setGeometry(rect());
    errorLabel_->setGeometry(rect());
    overlay_->setGeometry(rect());
    overlay_->raise();
    placeControls();
}

}  // namespace wpiemu::app::runview
