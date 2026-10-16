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
};

}  // namespace wpiemu::app
