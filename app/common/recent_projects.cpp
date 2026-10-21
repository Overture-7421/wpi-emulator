#include "common/recent_projects.h"

#include <QFileInfo>

#include "common/project_path.h"

namespace wpiemu::app {

namespace {
constexpr auto kKey = "projects/recent";
}  // namespace

RecentProjects::RecentProjects(IKeyValueStore& store, QObject* parent) : QObject(parent), store_(store) {
    paths_ = store_.value(kKey).toStringList();
    paths_.removeAll(QString());
}

void RecentProjects::touch(const QString& path) {
    const QString clean = normalizedProjectPath(path);
    if (clean.isEmpty()) return;
    for (qsizetype i = paths_.size() - 1; i >= 0; --i)
        if (sameProjectPath(paths_[i], clean)) paths_.removeAt(i);
    paths_.prepend(clean);
    while (paths_.size() > kMaxEntries) paths_.removeLast();
    save();
}

void RecentProjects::remove(const QString& path) {
    const QString clean = normalizedProjectPath(path);
    const auto before = paths_.size();
    for (qsizetype i = paths_.size() - 1; i >= 0; --i)
        if (sameProjectPath(paths_[i], clean)) paths_.removeAt(i);
    if (paths_.size() != before) save();
}

void RecentProjects::clear() {
    if (paths_.isEmpty()) return;
    paths_.clear();
    save();
}

int RecentProjects::pruneMissing() {
    const auto before = paths_.size();
    for (qsizetype i = paths_.size() - 1; i >= 0; --i)
        if (!QFileInfo(paths_[i]).isDir()) paths_.removeAt(i);
    const int removed = static_cast<int>(before - paths_.size());
    if (removed > 0) save();
    return removed;
}

void RecentProjects::save() {
    store_.setValue(kKey, paths_);
    store_.sync();
    emit changed();
}

}  // namespace wpiemu::app
