#include "common/system_probe.h"

#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>
#include <QSysInfo>

#if defined(Q_OS_WIN)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <string>
#include <vector>
#endif

namespace wpiemu::app {

HostOs QtSystemProbe::hostOs() const {
#if defined(Q_OS_WIN)
    return HostOs::Windows;
#elif defined(Q_OS_MACOS)
    return HostOs::MacOs;
#else
    return HostOs::Linux;
#endif
}

QString QtSystemProbe::homeDir() const { return QDir::homePath(); }

QString QtSystemProbe::cpuArchitecture() const { return QSysInfo::currentCpuArchitecture(); }

QString QtSystemProbe::fileVersion(const QString& path) const {
#if defined(Q_OS_WIN)
    const std::wstring native = QDir::toNativeSeparators(path).toStdWString();
    DWORD ignored = 0;
    const DWORD size = GetFileVersionInfoSizeW(native.c_str(), &ignored);
    if (size == 0) return {};
    std::vector<char> data(size);
    if (!GetFileVersionInfoW(native.c_str(), 0, size, data.data())) return {};
    VS_FIXEDFILEINFO* info = nullptr;
    UINT length = 0;
    if (!VerQueryValueW(data.data(), L"\\", reinterpret_cast<void**>(&info), &length) || info == nullptr) return {};
    return QStringLiteral("%1.%2.%3.%4")
        .arg(HIWORD(info->dwProductVersionMS))
        .arg(LOWORD(info->dwProductVersionMS))
        .arg(HIWORD(info->dwProductVersionLS))
        .arg(LOWORD(info->dwProductVersionLS));
#else
    Q_UNUSED(path);
    return {};
#endif
}

QString QtSystemProbe::env(const QString& name) const { return qEnvironmentVariable(name.toUtf8().constData()); }

bool QtSystemProbe::fileExists(const QString& path) const { return QFileInfo(path).isFile(); }

bool QtSystemProbe::dirExists(const QString& path) const { return QFileInfo(path).isDir(); }

QString QtSystemProbe::findInPath(const QString& executable) const {
    return QStandardPaths::findExecutable(executable);
}

ProcessResult QtSystemProbe::run(const QString& program, const QStringList& args, int timeoutMs) const {
    ProcessResult result;
    QProcess process;
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start(program, args);
    if (!process.waitForStarted(timeoutMs)) return result;
    result.started = true;
    if (!process.waitForFinished(timeoutMs)) {
        result.timedOut = true;
        process.kill();
        process.waitForFinished(1000);
    } else {
        result.exitCode = process.exitCode();
    }
    result.output = QString::fromLocal8Bit(process.readAll());
    return result;
}

QString loadedVcRuntime(const ISystemProbe& probe, const QString& jdkBinDir) {
    if (probe.hostOs() != HostOs::Windows) return {};
    QString version = probe.fileVersion(QDir::cleanPath(jdkBinDir + QStringLiteral("/msvcp140.dll")));
    if (!version.isEmpty()) return version;
    const QString root = probe.env(QStringLiteral("SystemRoot"));
    return root.isEmpty() ? QString() : probe.fileVersion(QDir::cleanPath(root + QStringLiteral("/System32/msvcp140.dll")));
}

}  // namespace wpiemu::app
