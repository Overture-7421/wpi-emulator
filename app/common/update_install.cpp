#include "common/update_install.h"

#include <QDir>

namespace wpiemu::app {

InstallInfo detectInstall(const ISystemProbe& probe, const QString& appDir, const QString& version, const std::vector<int>& seasonYears) {
    InstallInfo info;
    if (version.isEmpty() || version == QLatin1String("0.0.0")) return info;
    const QString dir = QDir::cleanPath(appDir);
    const QString root = QDir::cleanPath(dir + QStringLiteral("/.."));

    switch (probe.hostOs()) {
        case HostOs::Windows:
            // Installed: <root>\bin\wpiemu_app.exe with <root>\Uninstall.exe; the ZIP has the same layout without it.
            if (probe.fileExists(root + QStringLiteral("/Uninstall.exe"))) info.kind = InstallKind::Nsis;
            else {
                for (const int seasonYear : seasonYears) {
                    const QString year = QString::number(seasonYear);
                    if (probe.fileExists(root + QStringLiteral("/extensions/") + year +
                                         QStringLiteral("/halsim_wpiemu.dll"))) {
                        info.kind = InstallKind::Portable;
                        break;
                    }
                }
            }
            break;
        case HostOs::MacOs:
            // Running from a mounted disk image cannot be updated; an .app elsewhere can.
            if (dir.endsWith(QLatin1String(".app/Contents/MacOS")) && !dir.startsWith(QLatin1String("/Volumes/"))) info.kind = InstallKind::MacBundle;
            break;
        case HostOs::Linux: {
            const QString appImage = probe.env(QStringLiteral("APPIMAGE"));
            if (!appImage.isEmpty() && probe.fileExists(appImage)) {
                info.kind = InstallKind::AppImage;
                info.appImagePath = QDir::cleanPath(appImage);
            } else {
                for (const int seasonYear : seasonYears) {
                    const QString year = QString::number(seasonYear);
                    if (probe.fileExists(root + QStringLiteral("/lib/wpiemu/") + year +
                                         QStringLiteral("/libhalsim_wpiemu.so"))) {
                        info.kind = InstallKind::Portable;
                        break;
                    }
                }
            }
            break;
        }
    }
    return info;
}

}  // namespace wpiemu::app
