#pragma once

#include <QAbstractListModel>
#include <QSortFilterProxyModel>

#include <deque>

#include "run_backend.h"

namespace wpiemu::app::runview {

// Bounded log buffer shown by the console panel.
class ConsoleModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { LevelRole = Qt::UserRole + 1, StreamRole, SourceRole, TextRole, TimestampRole };

    explicit ConsoleModel(QObject* parent = nullptr, int capacity = 20000);

    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;

    void append(const RunLogEntry& entry);
    void clear();
    const RunLogEntry& at(int row) const { return entries_[static_cast<std::size_t>(row)]; }
    int capacity() const { return capacity_; }

    static QString formatLine(const RunLogEntry& entry);

private:
    std::deque<RunLogEntry> entries_;
    int capacity_;
};

// Level / stream / free-text filter. A line is visible when its level >= minimumLevel, its stream is enabled
// and (if a search text is set) it contains the text, case-insensitively.
class ConsoleFilterModel : public QSortFilterProxyModel {
    Q_OBJECT
public:
    explicit ConsoleFilterModel(QObject* parent = nullptr);

    sdk::LogLevel minimumLevel() const { return minimumLevel_; }

    void setMinimumLevel(sdk::LogLevel level);
    void setStreamEnabled(LogStream stream, bool enabled);
    void setSearchText(const QString& text);

    // Text of the given proxy rows (all visible rows when empty), one formatted line each.
    QString plainText(const QModelIndexList& rows = {}) const;

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent) const override;

private:
    sdk::LogLevel minimumLevel_ = sdk::LogLevel::Trace;
    bool streams_[3] = {true, true, true};
    QString search_;
};

QString levelName(sdk::LogLevel level);

}  // namespace wpiemu::app::runview
