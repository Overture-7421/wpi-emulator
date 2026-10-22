#pragma once

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>

#include <array>
#include <memory>

#include "common/ui_helpers.h"
#include "wizard/wizard_page.h"
#include "wizard/wizard_view_model.h"

namespace wpiemu::app {

class ProjectPage;
class ToolchainPage;
class RobotPage;
class SummaryPage;

// The first-run / "set up robot" wizard: a sidebar of steps, one page at a time, Back / Next and a
// final "Open editor". Closing it at any point writes nothing.
class WizardDialog : public QDialog {
    Q_OBJECT
public:
    WizardDialog(AppServices& services, const UiResources& ui, QWidget* parent = nullptr);
    ~WizardDialog() override;

    WizardViewModel& model() { return model_; }
    // Only meaningful after the dialog was accepted.
    const WizardResult& wizardResult() const { return result_; }

    // Pre-fills step 1 (used when a folder is dropped on the welcome page or re-running setup).
    void setProjectPath(const QString& path);
    // Opens on the toolchain check (used when the run view sends the user here to fix a failed launch).
    void showToolchainStep();

    ProjectPage* projectPage() const { return projectPage_; }
    ToolchainPage* toolchainPage() const { return toolchainPage_; }
    RobotPage* robotPage() const { return robotPage_; }
    SummaryPage* summaryPage() const { return summaryPage_; }
    QPushButton* nextButton() const { return nextButton_; }
    QPushButton* backButton() const { return backButton_; }
    QPushButton* cancelButton() const { return cancelButton_; }
    WizardPage* currentPage() const;
    QPushButton* stepButton(WizardStep step) const { return stepButtons_[static_cast<std::size_t>(step)]; }

private:
    void onStepChanged();
    void onNext();
    void refreshChrome();
    void refreshStepIcons();

    AppServices& services_;
    const UiResources& ui_;
    WizardViewModel model_;
    WizardResult result_;

    QLabel* titleLabel_;
    QLabel* subtitleLabel_;
    QStackedWidget* stack_;
    Banner* errorBanner_;
    QPushButton* cancelButton_;
    QPushButton* backButton_;
    QPushButton* nextButton_;
    std::array<QPushButton*, kWizardStepCount> stepButtons_{};

    ProjectPage* projectPage_ = nullptr;
    ToolchainPage* toolchainPage_ = nullptr;
    RobotPage* robotPage_ = nullptr;
    SummaryPage* summaryPage_ = nullptr;
};

}  // namespace wpiemu::app
