#include "viewmodel/issue_model.h"

#include <algorithm>

#include "model/tr.h"

namespace wpiemu::app::editor {

void IssueModel::setIssues(std::vector<sdk::Issue> issues) {
    beginResetModel();
    issues_ = std::move(issues);
    endResetModel();
}

const sdk::Issue* IssueModel::issueAt(const QModelIndex& index) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= static_cast<int>(issues_.size())) return nullptr;
    return &issues_[static_cast<std::size_t>(index.row())];
}

int IssueModel::count(sdk::Severity s) const {
    return static_cast<int>(std::count_if(issues_.begin(), issues_.end(), [s](const sdk::Issue& i) { return i.severity == s; }));
}

QString IssueModel::summary() const {
    const int e = count(sdk::Severity::Error);
    const int w = count(sdk::Severity::Warning);
    if (e == 0 && w == 0) return ETR("No problems");
    QStringList parts;
    if (e > 0) parts << QCoreApplication::translate("Editor", "%n error(s)", nullptr, e);
    if (w > 0) parts << QCoreApplication::translate("Editor", "%n warning(s)", nullptr, w);
    return parts.join(QStringLiteral(", "));
}

QVariant IssueModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return {};
    switch (section) {
        case ColSeverity: return ETR("Level");
        case ColMessage: return ETR("Problem");
        case ColWhere: return ETR("Where");
        default: return {};
    }
}

QVariant IssueModel::data(const QModelIndex& index, int role) const {
    const sdk::Issue* issue = issueAt(index);
    if (issue == nullptr) return {};
    if (role == SeverityRole) return static_cast<int>(issue->severity);
    if (role == CodeRole) return QString::fromStdString(issue->code);
    if (role == Qt::ToolTipRole)
        return QString::fromStdString(issue->message) + (issue->code.empty() ? QString() : QStringLiteral("\n[%1]").arg(QString::fromStdString(issue->code))) +
               ETR("\nClick to jump to it.");
    if (role != Qt::DisplayRole) return {};
    switch (index.column()) {
        case ColSeverity:
            return issue->severity == sdk::Severity::Error ? ETR("Error") : issue->severity == sdk::Severity::Warning ? ETR("Warning") : ETR("Info");
        case ColMessage: return QString::fromStdString(issue->message);
        case ColWhere: return QString::fromStdString(issue->path);
        default: return {};
    }
}

}  // namespace wpiemu::app::editor
