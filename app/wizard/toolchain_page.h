#pragma once

#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

#include <vector>

#include "common/ui_helpers.h"
#include "wizard/wizard_page.h"
#include "wizard/wizard_view_model.h"

namespace wpiemu::app {

// Step 2: is this machine able to build and launch the robot? Each requirement is a row with a
// status icon, what was found and how to fix it; "Re-check" runs everything again.
class ToolchainPage : public WizardPage {
    Q_OBJECT
public:
    ToolchainPage(WizardViewModel& model, const UiResources& ui, QWidget* parent = nullptr);

    QString title() const override { return tr("Check your toolchain"); }
    QString subtitle() const override { return tr("The emulator launches your robot with Gradle. Anything missing is explained here; you can also fix it later."); }
    void onEnter() override;

    QPushButton* recheckButton() const { return recheckButton_; }
    Banner* summaryBanner() const { return summary_; }
    int rowCount() const { return static_cast<int>(rows_.size()); }
    // Status shown for a row (exposed for tests).
    CheckStatus rowStatus(const QString& id) const;

private:
    struct Row {
        QString id;
        CheckStatus status = CheckStatus::Warning;
        QFrame* frame = nullptr;
        QLabel* icon = nullptr;
        QLabel* title = nullptr;
        QLabel* detail = nullptr;
        QLabel* hint = nullptr;
    };

    void refresh();
    void applyIcon(Row& row) const;
    void clearRows();

    WizardViewModel& model_;
    const UiResources& ui_;
    QPushButton* recheckButton_;
    QProgressBar* busy_;
    QLabel* busyLabel_;
    Banner* summary_;
    Banner* coreOnly_;
    QVBoxLayout* rowsLayout_;
    std::vector<Row> rows_;
};

}  // namespace wpiemu::app
