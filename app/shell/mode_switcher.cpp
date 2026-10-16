#include "shell/mode_switcher.h"

#include <QHBoxLayout>

namespace wpiemu::app {

ModeSwitcher::ModeSwitcher(const UiResources& ui, QWidget* parent)
    : QWidget(parent), ui_(ui), editorButton_(new QToolButton), runButton_(new QToolButton) {
    auto* group = new QButtonGroup(this);
    group->setExclusive(true);
    const auto setup = [&](QToolButton* b, const QString& text, const char* pos, const QString& tip) {
        b->setText(text);
        b->setToolTip(tip);
        b->setCheckable(true);
        b->setProperty("segment", true);
        b->setProperty("segmentPos", QString::fromLatin1(pos));
        b->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        b->setCursor(Qt::PointingHandCursor);
        group->addButton(b);
    };
    setup(editorButton_, tr("Editor"), "first", tr("Build and edit the robot model (Ctrl+1)"));
    setup(runButton_, tr("Run"), "last", tr("Run the robot on the field (Ctrl+2)"));
    editorButton_->setChecked(true);

    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(editorButton_);
    layout->addWidget(runButton_);

    connect(editorButton_, &QToolButton::clicked, this, [this] { emit modeRequested(AppMode::Editor); });
    connect(runButton_, &QToolButton::clicked, this, [this] { emit modeRequested(AppMode::Run); });
    connect(group, &QButtonGroup::buttonToggled, this, [this] { refreshIcons(); });
    connect(&ui_, &UiResources::themeChanged, this, &ModeSwitcher::refreshIcons);
    refreshIcons();
}

void ModeSwitcher::refreshIcons() {
    // Checked segments are filled with the accent colour, so their glyph uses the on-accent tone.
    editorButton_->setIcon(ui_.icon(QStringLiteral("edit-3d"), editorButton_->isChecked() ? IconTone::OnAccent : IconTone::Text));
    runButton_->setIcon(ui_.icon(QStringLiteral("run"), runButton_->isChecked() ? IconTone::OnAccent : IconTone::Text));
}

void ModeSwitcher::setMode(AppMode mode) {
    (mode == AppMode::Run ? runButton_ : editorButton_)->setChecked(true);
    refreshIcons();
}

void ModeSwitcher::setRunEnabled(bool enabled) {
    runButton_->setEnabled(enabled);
    runButton_->setToolTip(enabled ? tr("Run the robot on the field (Ctrl+2)") : tr("Set up a robot first"));
}

}  // namespace wpiemu::app
