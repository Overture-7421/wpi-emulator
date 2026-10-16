#pragma once

#include <QCheckBox>
#include <QLabel>

#include "common/ui_helpers.h"
#include "wizard/wizard_page.h"
#include "wizard/wizard_view_model.h"

namespace wpiemu::app {

// Step 4: review, then open the editor (optionally launching device detection right away).
class SummaryPage : public WizardPage {
    Q_OBJECT
public:
    SummaryPage(WizardViewModel& model, const UiResources& ui, QWidget* parent = nullptr);

    QString title() const override { return tr("You are ready to go"); }
    QString subtitle() const override { return tr("Review the setup. You can reopen this wizard any time from File ▸ Set up robot…"); }
    void onEnter() override { refresh(); }

    QCheckBox* detectCheck() const { return detectCheck_; }
    QLabel* detectHint() const { return detectHint_; }
    QString projectSummary() const { return projectValue_->text(); }
    QString robotSummary() const { return robotValue_->text(); }
    QString gameSummary() const { return gameValue_->text(); }

private:
    void refresh();

    WizardViewModel& model_;
    const UiResources& ui_;
    QLabel* projectValue_;
    QLabel* gameValue_;
    QLabel* toolchainValue_;
    QLabel* robotValue_;
    QLabel* locationValue_;
    QCheckBox* detectCheck_;
    QLabel* detectHint_;
};

}  // namespace wpiemu::app
