#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>

#include "common/ui_helpers.h"
#include "wizard/project_drop_zone.h"
#include "wizard/wizard_page.h"
#include "wizard/wizard_view_model.h"

namespace wpiemu::app {

// Step 1: choose the WPILib project folder (type, browse, drag and drop or pick a recent one) with live validation.
class ProjectPage : public WizardPage {
    Q_OBJECT
public:
    ProjectPage(WizardViewModel& model, RecentProjects& recent, const UiResources& ui, QWidget* parent = nullptr);

    QString title() const override { return tr("Choose your robot project"); }
    QString subtitle() const override { return tr("Pick the WPILib project folder (the one containing build.gradle). Nothing is changed in it."); }
    QWidget* initialFocus() const override { return pathEdit_; }

    QLineEdit* pathEdit() const { return pathEdit_; }
    ProjectDropZone* dropZone() const { return dropZone_; }
    Banner* banner() const { return banner_; }
    QListWidget* recentList() const { return recentList_; }
    // Applies what is typed right now (otherwise done after a short typing pause).
    void applyTypedPath();

private:
    void browse();
    void refresh();
    void refreshRecent();
    void refreshIcons();

    WizardViewModel& model_;
    RecentProjects& recent_;
    const UiResources& ui_;

    ProjectDropZone* dropZone_;
    QLabel* dropIcon_;
    QLineEdit* pathEdit_;
    QPushButton* browseButton_;
    Banner* banner_;
    QFrame* detailsCard_;
    QLabel* seasonValue_;
    QLabel* languageValue_;
    QLabel* teamValue_;
    QLabel* vendorValue_;
    QLabel* setupValue_;
    QLabel* recentTitle_;
    QListWidget* recentList_;
    QTimer typingTimer_;
};

}  // namespace wpiemu::app
