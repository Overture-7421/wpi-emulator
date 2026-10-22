#include "common/update_offer.h"

#include <QCryptographicHash>
#include <QFile>
#include <QRegularExpression>

namespace wpiemu::app {
namespace {

const QString kRepositoryPath = QStringLiteral("/Overture-7421/wpi-emulator");

bool isX64(const QString& arch) {
    const QString a = arch.toLower();
    return a == QLatin1String("x86_64") || a == QLatin1String("amd64") || a == QLatin1String("x64");
}

}  // namespace

QUrl latestReleaseUrl() { return QUrl(QString::fromLatin1(kRepositoryUrl) + QStringLiteral("/releases/latest")); }

QUrl releasePageUrl(const QString& tag) { return QUrl(QString::fromLatin1(kRepositoryUrl) + QStringLiteral("/releases/tag/") + tag); }

std::optional<QVersionNumber> versionFromTag(const QString& tag) {
    static const QRegularExpression re(QStringLiteral("^v?(\\d+)\\.(\\d+)\\.(\\d+)$"));
    const QRegularExpressionMatch m = re.match(tag);
    if (!m.hasMatch()) return std::nullopt;
    bool ok = true;
    const int major = m.captured(1).toInt(&ok);
    if (!ok) return std::nullopt;
    const int minor = m.captured(2).toInt(&ok);
    if (!ok) return std::nullopt;
    const int patch = m.captured(3).toInt(&ok);
    if (!ok) return std::nullopt;
    return QVersionNumber(major, minor, patch);
}

std::optional<QString> tagFromReleaseLocation(const QString& location) {
    const QUrl url(location);
    if (!url.isValid()) return std::nullopt;
    // A redirect elsewhere (another host or repository) is never followed.
    if (!url.host().isEmpty() && url.host().compare(QLatin1String("github.com"), Qt::CaseInsensitive) != 0) return std::nullopt;
    const QString path = url.path();
    const QString marker = kRepositoryPath + QStringLiteral("/releases/tag/");
    if (!path.startsWith(marker, Qt::CaseInsensitive)) return std::nullopt;
    QString tag = path.mid(marker.size());
    while (tag.endsWith(QLatin1Char('/'))) tag.chop(1);
    if (!versionFromTag(tag)) return std::nullopt;
    return tag;
}

std::optional<QString> installerFileName(const QVersionNumber& version, HostOs os, const QString& cpuArchitecture) {
    if (version.isNull()) return std::nullopt;
    const QString prefix = QStringLiteral("wpi-emulator-") + version.toString();
    switch (os) {
        case HostOs::Windows:
            if (!isX64(cpuArchitecture)) return std::nullopt;
            return prefix + QStringLiteral("-windows-x64.exe");
        case HostOs::MacOs: return prefix + QStringLiteral("-macos-arm64-x64.dmg");
        case HostOs::Linux:
            if (!isX64(cpuArchitecture)) return std::nullopt;
            return prefix + QStringLiteral("-linux-x86_64.AppImage");
    }
    return std::nullopt;
}

QUrl releaseAssetUrl(const QString& tag, const QString& fileName) {
    return QUrl(QString::fromLatin1(kRepositoryUrl) + QStringLiteral("/releases/download/") + tag + QLatin1Char('/') + fileName);
}

std::optional<QString> parseSha256Sidecar(const QByteArray& text, const QString& fileName) {
    static const QRegularExpression line(QStringLiteral("^([0-9a-fA-F]{64})\\s+\\*?(.+?)\\s*$"));
    for (const QString& raw : QString::fromUtf8(text).split(QLatin1Char('\n'))) {
        const QRegularExpressionMatch m = line.match(raw.trimmed());
        if (m.hasMatch() && m.captured(2) == fileName) return m.captured(1).toLower();
    }
    return std::nullopt;
}

QString sha256OfFile(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file)) return {};
    return QString::fromLatin1(hash.result().toHex());
}

}  // namespace wpiemu::app
