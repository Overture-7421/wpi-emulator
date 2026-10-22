#pragma once

#include <QString>
#include <QStringList>

namespace wpiemu::app {

struct ProcessResult {
    bool started = false;
    bool timedOut = false;
    int exitCode = -1;
    QString output;   // stdout and stderr merged
};

enum class HostOs { Windows, MacOs, Linux };

// Everything the toolchain checks need from the machine. Faked in tests so the check logic is
// exercised on any host.
class ISystemProbe {
public:
    virtual ~ISystemProbe() = default;
    virtual HostOs hostOs() const = 0;
    virtual QString homeDir() const = 0;
    virtual QString env(const QString& name) const = 0;
    virtual bool fileExists(const QString& path) const = 0;
    virtual bool dirExists(const QString& path) const = 0;
    // First match of `executable` on PATH, empty if absent.
    virtual QString findInPath(const QString& executable) const = 0;
    virtual ProcessResult run(const QString& program, const QStringList& args, int timeoutMs) const = 0;
    // "x86_64", "arm64", ... (QSysInfo naming). Fakes default to x86_64.
    virtual QString cpuArchitecture() const { return QStringLiteral("x86_64"); }
    // Windows: the product version of an executable or DLL ("14.44.35211.0"); empty when unreadable and on other systems.
    virtual QString fileVersion(const QString& path) const {
        Q_UNUSED(path);
        return {};
    }
};

class QtSystemProbe final : public ISystemProbe {
public:
    HostOs hostOs() const override;
    QString homeDir() const override;
    QString env(const QString& name) const override;
    bool fileExists(const QString& path) const override;
    bool dirExists(const QString& path) const override;
    QString findInPath(const QString& executable) const override;
    ProcessResult run(const QString& program, const QStringList& args, int timeoutMs) const override;
    QString cpuArchitecture() const override;
    QString fileVersion(const QString& path) const override;
};

// Windows: the version of msvcp140.dll that a java.exe in `jdkBinDir` loads, which WPILib compares with 14.40: the JDK's own copy
// when it ships one, else the system's. Empty when neither is readable (and on other systems).
QString loadedVcRuntime(const ISystemProbe& probe, const QString& jdkBinDir);

}  // namespace wpiemu::app
