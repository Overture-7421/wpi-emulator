#include "console_model.h"

#include <QCoreApplication>
#include <QDateTime>

#include <algorithm>

namespace wpiemu::app::runview {

QString levelName(sdk::LogLevel level) {
    switch (level) {
        case sdk::LogLevel::Trace: return QCoreApplication::translate("Console", "Trace");
        case sdk::LogLevel::Debug: return QCoreApplication::translate("Console", "Debug");
        case sdk::LogLevel::Info: return QCoreApplication::translate("Console", "Info");
        case sdk::LogLevel::Warn: return QCoreApplication::translate("Console", "Warning");
        case sdk::LogLevel::Error: return QCoreApplication::translate("Console", "Error");
    }
    return {};
}

ConsoleModel::ConsoleModel(QObject* parent, int capacity)
    : QAbstractListModel(parent), capacity_(std::max(1, capacity)) {}

int ConsoleModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : static_cast<int>(entries_.size());
}

QString ConsoleModel::formatLine(const RunLogEntry& e) {
    const QString time = QDateTime::fromMSecsSinceEpoch(e.timestampMs).toString(QStringLiteral("HH:mm:ss.zzz"));
    return e.source.isEmpty() ? QStringLiteral("%1  %2").arg(time, e.text)
                              : QStringLiteral("%1  [%2] %3").arg(time, e.source, e.text);
}

QVariant ConsoleModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const RunLogEntry& e = entries_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case Qt::DisplayRole: return formatLine(e);
        case LevelRole: return static_cast<int>(e.level);
        case StreamRole: return static_cast<int>(e.stream);
        case SourceRole: return e.source;
        case TextRole: return e.text;
        case TimestampRole: return e.timestampMs;
        default: return {};
    }
}

void ConsoleModel::append(const RunLogEntry& entry) {
    if (static_cast<int>(entries_.size()) >= capacity_) {
        beginRemoveRows({}, 0, 0);
        entries_.pop_front();
        endRemoveRows();
    }
    const int row = static_cast<int>(entries_.size());
    beginInsertRows({}, row, row);
    entries_.push_back(entry);
    endInsertRows();
}

void ConsoleModel::clear() {
    beginResetModel();
    entries_.clear();
    endResetModel();
}

ConsoleFilterModel::ConsoleFilterModel(QObject* parent) : QSortFilterProxyModel(parent) {
    setDynamicSortFilter(true);
}

void ConsoleFilterModel::setMinimumLevel(sdk::LogLevel level) {
    if (level == minimumLevel_) return;
    minimumLevel_ = level;
    invalidateFilter();
}

void ConsoleFilterModel::setStreamEnabled(LogStream stream, bool enabled) {
    bool& slot = streams_[static_cast<int>(stream)];
    if (slot == enabled) return;
    slot = enabled;
    invalidateFilter();
}

void ConsoleFilterModel::setSearchText(const QString& text) {
    if (text == search_) return;
    search_ = text;
    invalidateFilter();
}

bool ConsoleFilterModel::filterAcceptsRow(int sourceRow, const QModelIndex& parent) const {
    const QModelIndex idx = sourceModel()->index(sourceRow, 0, parent);
    if (idx.data(ConsoleModel::LevelRole).toInt() < static_cast<int>(minimumLevel_)) return false;
    if (!streams_[idx.data(ConsoleModel::StreamRole).toInt()]) return false;
    if (search_.isEmpty()) return true;
    return idx.data(Qt::DisplayRole).toString().contains(search_, Qt::CaseInsensitive);
}

QString ConsoleFilterModel::plainText(const QModelIndexList& rows) const {
    QStringList lines;
    if (rows.isEmpty()) {
        for (int r = 0; r < rowCount(); ++r) lines << index(r, 0).data(Qt::DisplayRole).toString();
    } else {
        QModelIndexList sorted = rows;
        std::sort(sorted.begin(), sorted.end(), [](const QModelIndex& a, const QModelIndex& b) { return a.row() < b.row(); });
        for (const QModelIndex& i : sorted) lines << i.data(Qt::DisplayRole).toString();
    }
    return lines.join(QLatin1Char('\n'));
}

}  // namespace wpiemu::app::runview
