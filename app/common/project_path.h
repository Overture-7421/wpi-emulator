#pragma once

// How the UI identifies a project folder it remembers: the open session, the recent-projects list and the per-project
// preferences all use the same text. It is lexical (absolute, "." and ".." and separators cleaned) and does NOT resolve
// symlinks, so it stays the path the user opened. Where a config is stored, core resolves aliases instead
// (core/config/config_store.h projectKey).

#include <QString>

namespace wpiemu::app {

QString normalizedProjectPath(const QString& path);

// Windows paths are case-insensitive, other platforms are case-sensitive.
bool sameProjectPath(const QString& a, const QString& b);

// normalizedProjectPath folded to that case rule: the stable text to hash into a settings key.
QString projectPathKeyText(const QString& path);

}  // namespace wpiemu::app
