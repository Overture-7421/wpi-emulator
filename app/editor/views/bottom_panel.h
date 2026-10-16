#pragma once

// Bottom tabs: Console (robot / discovery output), Validation (click a row to jump to the problem)
// and Telemetry (live values while a robot is connected in discovery mode).

#include <QPlainTextEdit>
#include <QTableView>
#include <QTableWidget>
#include <QTabWidget>

#include "viewmodel/editor_controller.h"
#include "viewmodel/issue_model.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class BottomPanel : public QTabWidget {
    Q_OBJECT
public:
    BottomPanel(EditorController& controller, IssueModel& issues, const IconFactory& icons, QWidget* parent = nullptr);

    void appendLog(sdk::Severity severity, const QString& text);
    void showValidation();
    QPlainTextEdit* console() const { return console_; }
    QTableView* issueTable() const { return issueTable_; }
    QTableWidget* telemetryTable() const { return telemetry_; }
    void setSnapshot(const std::shared_ptr<const sdk::ISimSnapshot>& snapshot);

private:
    void updateValidationTab();
    int validationTabIndex_ = 1;

    EditorController& controller_;
    IssueModel& issues_;
    const IconFactory& icons_;
    QPlainTextEdit* console_;
    QTableView* issueTable_;
    QWidget* issueEmpty_;
    QTableWidget* telemetry_;
    QWidget* telemetryEmpty_;
};

}  // namespace wpiemu::app::editor
