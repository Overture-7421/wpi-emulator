#include "shell/welcome_page.h"

#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "common/ui_helpers.h"
#include "wizard/project_drop_zone.h"

namespace wpiemu::app {

WelcomePage::WelcomePage(RecentProjects& recent, const UiResources& ui, QWidget* parent)
    : QWidget(parent),
      recent_(recent),
      ui_(ui),
      logo_(new QLabel),
      openButton_(new QPushButton(tr("Open WPILib project…"))),
      dropZone_(new ProjectDropZone),
      recentTitle_(makeLabel(tr("Recent projects"), "section")),
      recentList_(new QListWidget) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addStretch(1);

    auto* column = new QVBoxLayout;
    column->setSpacing(14);
    logo_->setAlignment(Qt::AlignCenter);
    auto* title = makeLabel(tr("WPI Emulator"), "title");
    title->setAlignment(Qt::AlignCenter);
    auto* subtitle = makeLabel(tr("Run your unmodified WPILib robot code against a simulated robot and field."), "subtitle", nullptr, true);
    subtitle->setAlignment(Qt::AlignCenter);
    column->addWidget(logo_);
    column->addWidget(title);
    column->addWidget(subtitle);

    openButton_->setProperty("primary", true);
    openButton_->setMinimumHeight(40);
    openButton_->setMinimumWidth(240);

    auto* dropLayout = new QVBoxLayout(dropZone_);
    dropLayout->setContentsMargins(24, 22, 24, 22);
    dropLayout->setSpacing(12);
    auto* dropText = makeLabel(tr("Drop a project folder here, or"), "muted");
    dropText->setAlignment(Qt::AlignCenter);
    dropLayout->addWidget(dropText);
    dropLayout->addWidget(openButton_, 0, Qt::AlignCenter);
    column->addWidget(dropZone_);

    column->addWidget(recentTitle_);
    recentList_->setMaximumHeight(190);
    recentList_->setTextElideMode(Qt::ElideMiddle);
    recentList_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    recentList_->setAccessibleName(tr("Recent projects"));
    column->addWidget(recentList_);

    auto* row = new QHBoxLayout;
    row->addStretch(1);
    auto* holder = new QWidget;
    holder->setLayout(column);
    holder->setFixedWidth(560);
    row->addWidget(holder);
    row->addStretch(1);
    outer->addLayout(row);
    outer->addStretch(2);

    connect(openButton_, &QPushButton::clicked, this, &WelcomePage::openRequested);
    connect(dropZone_, &ProjectDropZone::pathDropped, this, &WelcomePage::projectChosen);
    connect(recentList_, &QListWidget::itemActivated, this, [this](QListWidgetItem* item) { emit projectChosen(item->data(Qt::UserRole).toString()); });
    connect(recentList_, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) { emit projectChosen(item->data(Qt::UserRole).toString()); });
    connect(&recent_, &RecentProjects::changed, this, &WelcomePage::refreshRecent);
    connect(&ui_, &UiResources::themeChanged, this, &WelcomePage::refreshIcons);
    refreshIcons();
    refreshRecent();
}

void WelcomePage::refreshIcons() {
    logo_->setPixmap(ui_.logo().pixmap(72, 72));
    openButton_->setIcon(ui_.icon(QStringLiteral("folder-open"), IconTone::OnAccent));
    refreshRecent();
}

void WelcomePage::refreshRecent() {
    recentList_->clear();
    for (const QString& path : recent_.paths()) {
        auto* item = new QListWidgetItem(ui_.icon(QStringLiteral("folder-open"), IconTone::Muted),
                                         QStringLiteral("%1\n%2").arg(QFileInfo(path).fileName(), QDir::toNativeSeparators(path)));
        item->setData(Qt::UserRole, path);
        item->setToolTip(QDir::toNativeSeparators(path));
        recentList_->addItem(item);
    }
    recentTitle_->setVisible(recentList_->count() > 0);
    recentList_->setVisible(recentList_->count() > 0);
}

}  // namespace wpiemu::app
