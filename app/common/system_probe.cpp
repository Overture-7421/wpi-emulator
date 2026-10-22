#include "common/system_probe.h"

#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>
#include <QSysInfo>

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

}  // namespace wpiemu::app
