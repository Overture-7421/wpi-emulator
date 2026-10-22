#pragma once

// Starts Microsoft's own installer for the C++ build tools a C++ robot project needs on Windows. Only ever started by a click
// on the Toolchain page's "Install C++ build tools" button, never silent: the Visual Studio installer shows its progress and
// Windows asks for administrator permission. Everything it touches goes through IBuildToolsEffects so tests script it.

#include <QObject>
#include <QString>
#include <QUrl>

#include <functional>
#include <memory>

#include "common/system_probe.h"

namespace wpiemu::app {

class IBuildToolsEffects {
public:
    virtual ~IBuildToolsEffects() = default;
    virtual bool wingetAvailable() = 0;
    // Runs `winget <args>` without blocking; `done` gets the exit code (-1 when it could not start) and its output.
    virtual void runWinget(const QStringList& args, std::function<void(int, const QString&)> done) = 0;
    virtual void cancel() = 0;
    virtual bool openUrl(const QUrl& url) = 0;
};

std::unique_ptr<IBuildToolsEffects> createQtBuildToolsEffects();

enum class BuildToolsState { Idle, Installing, Done, Failed, OpenedDownload };

class BuildToolsInstaller : public QObject {
    Q_OBJECT
public:
    BuildToolsInstaller(const ISystemProbe& probe, std::unique_ptr<IBuildToolsEffects> effects, QObject* parent = nullptr);
    ~BuildToolsInstaller() override;

    // Only Windows has a Visual Studio to install; macOS and Linux keep their one-line hints.
    bool available() const { return probe_.hostOs() == HostOs::Windows; }
    BuildToolsState state() const { return state_; }
    bool busy() const { return state_ == BuildToolsState::Installing; }
    const QString& message() const { return message_; }   // what happened, for the page

    // The winget arguments, exposed so the exact command is reviewable and tested.
    static QStringList wingetArguments();
    static QUrl downloadPage();

    // winget when it exists, else the Build Tools download page in the browser. No-op while installing.
    void start();
    void cancel();

signals:
    void stateChanged();
    // After winget ended (ok = exit code 0) or the download page was opened (ok = false: the user finishes it by hand).
    void finished(bool ok);

private:
    void setState(BuildToolsState state, QString message);

    const ISystemProbe& probe_;
    std::unique_ptr<IBuildToolsEffects> effects_;
    BuildToolsState state_ = BuildToolsState::Idle;
    QString message_;
    quint64 generation_ = 0;
};

}  // namespace wpiemu::app
