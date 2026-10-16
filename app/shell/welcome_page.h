#pragma once

#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>

#include "common/recent_projects.h"
#include "common/ui_resources.h"

namespace wpiemu::app {

class ProjectDropZone;

// What shows before a project is open: open/drop a project, or pick a recent one.
class WelcomePage : public QWidget {
    Q_OBJECT
public:
    WelcomePage(RecentProjects& recent, const UiResources& ui, QWidget* parent = nullptr);

    QPushButton* openButton() const { return openButton_; }
    QListWidget* recentList() const { return recentList_; }
    ProjectDropZone* dropZone() const { return dropZone_; }

signals:
    void openRequested();                       // "Open project…" button
    void projectChosen(const QString& path);    // recent entry or dropped folder

private:
    void refreshRecent();
    void refreshIcons();

    RecentProjects& recent_;
    const UiResources& ui_;
    QLabel* logo_;
    QPushButton* openButton_;
    ProjectDropZone* dropZone_;
    QLabel* recentTitle_;
    QListWidget* recentList_;
};

}  // namespace wpiemu::app
