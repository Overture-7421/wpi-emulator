#include "shell/mode_switcher.h"

#include <QHBoxLayout>
#include <QSignalBlocker>

namespace wpiemu::app {

ModeSwitcher::ModeSwitcher(const UiResources& ui, QWidget* parent) : QWidget(parent), ui_(ui), tabs_(new QTabBar) {
    tabs_->setDrawBase(false);
    tabs_->setExpanding(false);
    tabs_->setFocusPolicy(Qt::NoFocus);
    tabs_->setIconSize(QSize(16, 16));
    // One rounded track holding both options, like a two-state switch; the active half is filled with the accent colour.
    setObjectName(QStringLiteral("modeSwitch"));
    setAttribute(Qt::WA_StyledBackground);
    tabs_->setCursor(Qt::PointingHandCursor);
    setStyleSheet(QStringLiteral(
        "#modeSwitch { background: palette(button); border: 1px solid palette(mid); border-radius: 8px; }"
        "#modeSwitch QTabBar { background: transparent; }"
        "#modeSwitch QTabBar::tab { background: transparent; color: palette(button-text); border: none; border-radius: 6px;"
        " padding: 1px 14px; margin: 1px; }"
        "#modeSwitch QTabBar::tab:hover:!selected { background: palette(light); }"
        "#modeSwitch QTabBar::tab:selected { background: palette(highlight); color: palette(highlighted-text); font-weight: 600; }"
        "#modeSwitch QTabBar::tab:disabled { color: palette(mid); }"));

    tabs_->addTab(tr("Editor"));
    tabs_->addTab(tr("Run"));
    tabs_->setTabToolTip(kEditorTab, tr("Build and edit the robot model (Ctrl+1)"));
    tabs_->setTabToolTip(kRunTab, tr("Run the robot on the field (Ctrl+2)"));

    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(tabs_);

    connect(tabs_, &QTabBar::currentChanged, this, [this] {
        refreshIcons();
        emit modeRequested(mode());
    });
    connect(&ui_, &UiResources::themeChanged, this, &ModeSwitcher::refreshIcons);
    refreshIcons();
}

void ModeSwitcher::refreshIcons() {
    tabs_->setTabIcon(kEditorTab, ui_.icon(QStringLiteral("edit-3d"), mode() == AppMode::Editor ? IconTone::OnAccent : IconTone::Text));
    tabs_->setTabIcon(kRunTab, ui_.icon(QStringLiteral("run"), mode() == AppMode::Run ? IconTone::OnAccent : IconTone::Text));
}

void ModeSwitcher::setMode(AppMode mode) {
    {
        const QSignalBlocker block(tabs_);
        tabs_->setCurrentIndex(mode == AppMode::Run ? kRunTab : kEditorTab);
    }
    refreshIcons();
}

void ModeSwitcher::setRunEnabled(bool enabled) {
    tabs_->setTabEnabled(kRunTab, enabled);
    tabs_->setTabToolTip(kRunTab, enabled ? tr("Run the robot on the field (Ctrl+2)") : tr("Set up a robot first"));
}

}  // namespace wpiemu::app
