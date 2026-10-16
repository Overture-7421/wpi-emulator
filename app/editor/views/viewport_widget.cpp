#include "views/viewport_widget.h"

#include <QButtonGroup>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickItem>
#include <QSignalBlocker>
#include <QStackedLayout>
#include <QVBoxLayout>

#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {

QToolButton* makeButton(const QString& text, const QString& tip, QWidget* parent, bool checkable = false) {
    auto* b = new QToolButton(parent);
    b->setText(text);
    b->setToolTip(tip);
    b->setCheckable(checkable);
    b->setAutoRaise(true);
    b->setToolButtonStyle(Qt::ToolButtonTextOnly);
    b->setFocusPolicy(Qt::NoFocus);
    return b;
}

QFrame* separator(QWidget* parent) {
    auto* f = new QFrame(parent);
    f->setFrameShape(QFrame::VLine);
    f->setFrameShadow(QFrame::Sunken);
    return f;
}

}  // namespace

ViewportWidget::ViewportWidget(EditorController& editor, ViewportController& viewport, QWidget* parent)
    : QWidget(parent), editor_(editor), viewport_(viewport), quick_(new QQuickWidget(this)), fallback_(new QLabel(this)), info_(new QLabel(this)) {
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    auto* bar = new QWidget(this);
    auto* barLayout = new QHBoxLayout(bar);
    barLayout->setContentsMargins(8, 4, 8, 4);
    barLayout->setSpacing(4);
    buildToolbar(barLayout);
    v->addWidget(bar);

    auto* host = new QWidget(this);
    auto* stack = new QStackedLayout(host);
    stack->setStackingMode(QStackedLayout::StackAll);
    quick_->setResizeMode(QQuickWidget::SizeRootObjectToView);
    quick_->setMinimumSize(320, 240);
    quick_->setFocusPolicy(Qt::ClickFocus);
    quick_->rootContext()->setContextProperty(QStringLiteral("editorCtl"), &viewport_);
    fallback_->setAlignment(Qt::AlignCenter);
    fallback_->setWordWrap(true);
    fallback_->setMargin(24);
    fallback_->hide();
    stack->addWidget(quick_);
    stack->addWidget(fallback_);
    v->addWidget(host, 1);

    info_->setContentsMargins(10, 3, 10, 4);
    info_->setEnabled(false);
    v->addWidget(info_);

    connect(quick_, &QQuickWidget::statusChanged, this, [this] { onStatusChanged(); });
    connect(quick_, &QQuickWidget::sceneGraphError, this, [this](QQuickWindow::SceneGraphError, const QString& message) {
        failure_ = message;
        fallback_->setText(ETR("The 3D viewport could not start:\n%1\n\nThe tree, inspector and device mapping still work.").arg(message));
        fallback_->show();
    });
    connect(&viewport_, &ViewportController::modeChanged, this, [this] { syncToolbar(); });
    connect(&viewport_, &ViewportController::transformFieldsChanged, this, [this] { syncToolbar(); });
    connect(&viewport_, &ViewportController::infoChanged, this, [this] { info_->setText(viewport_.infoText()); });
    connect(&editor_.settings(), &EditorSettings::changed, this, [this] { syncToolbar(); });

    applyTheme();
    quick_->setSource(QUrl(QStringLiteral("qrc:/wpiemu/editor/qml/Viewport3D.qml")));
    info_->setText(viewport_.infoText());
    syncToolbar();
}

void ViewportWidget::buildToolbar(QLayout* layout) {
    auto* modes = new QButtonGroup(this);
    modes->setExclusive(true);
    const char* names[] = {QT_TRANSLATE_NOOP("Editor", "Move"), QT_TRANSLATE_NOOP("Editor", "Rotate"), QT_TRANSLATE_NOOP("Editor", "Scale")};
    const char* keys[] = {"W", "E", "R"};
    for (int i = 0; i < 3; ++i) {
        modeButtons_[i] = makeButton(QCoreApplication::translate("Editor", names[i]), ETR("%1 gizmo (%2)").arg(QCoreApplication::translate("Editor", names[i]), QString::fromLatin1(keys[i])), this, true);
        modes->addButton(modeButtons_[i], i);
        layout->addWidget(modeButtons_[i]);
        connect(modeButtons_[i], &QToolButton::clicked, this, [this, i] { viewport_.setGizmoMode(static_cast<GizmoMode>(i)); });
    }
    modeButtons_[0]->setChecked(true);

    layout->addWidget(separator(this));
    snapButton_ = makeButton(ETR("Snap"), ETR("Snap to the grid / 15 degrees. Hold Ctrl while dragging to toggle."), this, true);
    snapButton_->setChecked(editor_.settings().snapEnabled());
    connect(snapButton_, &QToolButton::toggled, this, [this](bool on) { editor_.settings().setSnapEnabled(on); });
    layout->addWidget(snapButton_);

    layout->addWidget(separator(this));
    const std::pair<const char*, const char*> views[] = {{QT_TRANSLATE_NOOP("Editor", "Front"), "front"}, {QT_TRANSLATE_NOOP("Editor", "Side"), "side"},
                                                          {QT_TRANSLATE_NOOP("Editor", "Top"), "top"}, {QT_TRANSLATE_NOOP("Editor", "3D"), "iso"}};
    for (const auto& [label, preset] : views) {
        auto* b = makeButton(QCoreApplication::translate("Editor", label), ETR("%1 view").arg(QCoreApplication::translate("Editor", label)), this);
        const QString p = QString::fromLatin1(preset);
        connect(b, &QToolButton::clicked, this, [this, p] { viewport_.setView(p); });
        layout->addWidget(b);
    }
    auto* frame = makeButton(ETR("Frame"), ETR("Frame the selection (F)"), this);
    connect(frame, &QToolButton::clicked, this, [this] { viewport_.frameSelection(); });
    layout->addWidget(frame);

    static_cast<QHBoxLayout*>(layout)->addStretch(1);

    fieldCaption_ = new QLabel(this);
    layout->addWidget(fieldCaption_);
    for (int i = 0; i < 3; ++i) {
        auto* tag = new QLabel(QString::fromLatin1(i == 0 ? "X" : i == 1 ? "Y" : "Z"), this);
        tag->setEnabled(false);
        layout->addWidget(tag);
        auto* spin = new QDoubleSpinBox(this);
        spin->setRange(-100000, 100000);
        spin->setDecimals(2);
        spin->setKeyboardTracking(true);
        spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spin->setAlignment(Qt::AlignRight);
        spin->setFixedWidth(84);
        fields_[static_cast<std::size_t>(i)] = spin;
        layout->addWidget(spin);
        connect(spin, &QDoubleSpinBox::valueChanged, this, [this, i](double shown) {
            const TransformFields f = viewport_.transformFields();
            if (!f.valid) return;
            double si = shown;
            if (f.mode != GizmoMode::Rotate) si = fromDisplay(shown, sdk::PropertyUnit::Meters, editor_.settings().units());
            viewport_.setTransformComponent(i, si);
        });
    }
}

void ViewportWidget::syncToolbar() {
    const TransformFields f = viewport_.transformFields();
    const auto effective = viewport_.effectiveMode();
    for (int i = 0; i < 3; ++i) {
        const QSignalBlocker block(modeButtons_[i]);
        modeButtons_[i]->setChecked(effective && static_cast<int>(*effective) == i);
    }
    {
        const QSignalBlocker block(snapButton_);
        snapButton_->setChecked(editor_.settings().snapEnabled());
    }
    const UnitSystem units = editor_.settings().units();
    fieldCaption_->setText(!f.valid ? QString()
                           : f.mode == GizmoMode::Move ? ETR("Position")
                           : f.mode == GizmoMode::Rotate ? ETR("Rotation")
                                                          : ETR("Size"));
    const QString suffix = f.mode == GizmoMode::Rotate ? QStringLiteral(" \xC2\xB0") : unitSuffix(sdk::PropertyUnit::Meters, units);
    for (int i = 0; i < 3; ++i) {
        QDoubleSpinBox* spin = fields_[static_cast<std::size_t>(i)];
        spin->setEnabled(f.valid);
        spin->setSuffix(suffix);
        spin->setSingleStep(f.mode == GizmoMode::Rotate ? 5.0 : unitDisplay(sdk::PropertyUnit::Meters, units).step);
        if (!f.valid || spin->hasFocus()) continue;
        const QSignalBlocker block(spin);
        const float raw = i == 0 ? f.value.x : i == 1 ? f.value.y : f.value.z;
        spin->setValue(f.mode == GizmoMode::Rotate ? static_cast<double>(raw) : toDisplay(static_cast<double>(raw), sdk::PropertyUnit::Meters, units));
    }
    info_->setText(viewport_.infoText());
}

void ViewportWidget::applyTheme() {
    const SceneTheme theme = SceneTheme::fromPalette(palette());
    viewport_.setTheme(theme);
    quick_->setClearColor(theme.background);
}

void ViewportWidget::changeEvent(QEvent* event) {
    QWidget::changeEvent(event);
    if (event->type() == QEvent::PaletteChange || event->type() == QEvent::ApplicationPaletteChange) applyTheme();
}

void ViewportWidget::onStatusChanged() {
    if (quick_->status() != QQuickWidget::Error) return;
    QStringList messages;
    for (const auto& e : quick_->errors()) messages << e.toString();
    failure_ = messages.join('\n');
    fallback_->setText(ETR("The 3D viewport could not load:\n%1").arg(failure_));
    fallback_->show();
}

bool ViewportWidget::sceneReady() const { return quick_->status() == QQuickWidget::Ready && failure_.isEmpty(); }

QImage ViewportWidget::grabScene() { return quick_->grabFramebuffer(); }

}  // namespace wpiemu::app::editor
