#pragma once

// Pure logic of the updater: what the GitHub release page and its files are called, and how to check a download.
// Nothing here touches the network. The contract it relies on (the `releases/latest` redirect, the installer names that
// CPack and build-appimage.sh produce, a `.sha256` sidecar per installer) is documented in docs/releasing.md.

#include <QByteArray>
#include <QString>
#include <QUrl>
#include <QVersionNumber>

#include <optional>

#include "common/system_probe.h"

namespace wpiemu::app {

inline constexpr const char* kRepositoryUrl = "https://github.com/Overture-7421/wpi-emulator";

// github.com redirects this to the newest published stable release (never a draft or pre-release).
QUrl latestReleaseUrl();
QUrl releasePageUrl(const QString& tag);

// "v0.0.4" or "0.0.4" -> 0.0.4. Anything else (a pre-release such as "v1.2.3-rc1", "v.0.0.1", "alpha.0.0.1") -> nullopt.
std::optional<QVersionNumber> versionFromTag(const QString& tag);
// The tag that a `.../releases/tag/<tag>` redirect points to, when it names a stable version of this repository.
std::optional<QString> tagFromReleaseLocation(const QString& location);

// The installer the release workflow publishes for this machine; nullopt when it publishes none (e.g. Windows on ARM).
std::optional<QString> installerFileName(const QVersionNumber& version, HostOs os, const QString& cpuArchitecture);
QUrl releaseAssetUrl(const QString& tag, const QString& fileName);

// Reads the `sha256sum` line "<64 hex>  <file name>" for `fileName`; lower-case hex, nullopt when absent or malformed.
std::optional<QString> parseSha256Sidecar(const QByteArray& text, const QString& fileName);
// SHA-256 of a file as lower-case hex; empty when it cannot be read.
QString sha256OfFile(const QString& path);

}  // namespace wpiemu::app
