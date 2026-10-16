#pragma once

// Validation results as a table (severity / message / where). Row activation is handled by the page
// (it asks the controller to focus the issue); this model only presents the data.

#include <QAbstractTableModel>

#include <vector>

#include "wpiemu/sdk/core/severity.h"

namespace wpiemu::app::editor {

class IssueModel : public QAbstractTableModel {
    Q_OBJECT
public:
    enum Column { ColSeverity, ColMessage, ColWhere, ColumnCount };
    enum Role { SeverityRole = Qt::UserRole + 1, CodeRole };

    explicit IssueModel(QObject* parent = nullptr) : QAbstractTableModel(parent) {}

    void setIssues(std::vector<sdk::Issue> issues);
    const sdk::Issue* issueAt(const QModelIndex& index) const;
    int count(sdk::Severity s) const;
    // "2 errors, 3 warnings" or "No problems".
    QString summary() const;

    int rowCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : static_cast<int>(issues_.size()); }
    int columnCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : ColumnCount; }
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

private:
    std::vector<sdk::Issue> issues_;
};

}  // namespace wpiemu::app::editor
