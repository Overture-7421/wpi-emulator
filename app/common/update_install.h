#pragma once

// How this copy of the app was installed, which decides what "update" can mean (docs/reference/app.md).

#include <QString>

#include "common/system_probe.h"

namespace wpiemu::app {

enum class InstallKind {
    Nsis,       // Windows installer: the update is the new installer
    MacBundle,  // an .app outside a mounted disk image: the update is the new DMG, opened for the user
    AppImage,   // Linux AppImage: the update is a new AppImage saved beside this one
    Portable,   // packaged layout without an installer (ZIP, TGZ): only a link to the release page
    Dev,        // build tree, mounted DMG, unknown layout or version 0.0.0: never checks
};

struct InstallInfo {
    InstallKind kind = InstallKind::Dev;
    QString appImagePath;   // the running AppImage file, for InstallKind::AppImage
};

// `appDir` is the directory of the running executable; `version` the app version (QCoreApplication::applicationVersion).
InstallInfo detectInstall(const ISystemProbe& probe, const QString& appDir, const QString& version);

}  // namespace wpiemu::app
