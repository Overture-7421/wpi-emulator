#pragma once

// The updater's state machine. It owns no widgets: the Update button and the Help menu observe it (docs/reference/app.md).

#include <QObject>
#include <QString>
#include <QUrl>
#include <QVersionNumber>

#include <functional>
#include <memory>

#include "common/key_value_store.h"
#include "common/system_probe.h"
#include "common/update_install.h"
#include "common/update_transport.h"

namespace wpiemu::app {

// Everything that touches the machine once a file is downloaded. Faked in tests.
class IUpdateInstaller {
public:
    virtual ~IUpdateInstaller() = default;
    // Folder-creating path for the download of `fileName`; empty when no writable place exists.
    virtual QString targetPath(const InstallInfo& install, const QString& fileName) = 0;
    // Hashes on a worker thread; `done` runs on the owner's thread with lower-case hex ("" when unreadable).
    virtual void sha256(const QString& path, std::function<void(const QString&)> done) = 0;
    // Runs the verified installer: starts it (Windows, Linux) or opens the disk image (macOS).
    virtual bool launch(const InstallInfo& install, const QString& path, QString* error) = 0;
    virtual bool openUrl(const QUrl& url) = 0;
    // Deletes a downloaded or partial file, and leftovers of earlier updates when `path` is empty.
    virtual void discard(const QString& path) = 0;
};

std::unique_ptr<IUpdateInstaller> createQtUpdateInstaller();

// Does launching the update replace the running app (so it must quit first)?
bool updateRestartsApp(InstallKind kind);

enum class UpdateState {
    Idle,          // nothing known (never checked, a silent check failed, or this version was skipped)
    Checking,
    UpToDate,
    Unsupported,   // development build: updates are not offered
    Available,     // a newer stable release exists
    Downloading,
    Verifying,
    Ready,         // downloaded and verified
    Installing,    // the app is about to quit and run the installer
    Failed,        // the download or its verification failed; install() retries
};

struct UpdateOffer {
    QString tag;
    QVersionNumber version;
    QString fileName;     // empty for a portable install: only the release page is offered
    QUrl assetUrl;
    QUrl sidecarUrl;
    QUrl pageUrl;
    QString sha256;
};

class UpdateService : public QObject {
    Q_OBJECT
public:
    UpdateService(IKeyValueStore& store, const ISystemProbe& probe, std::unique_ptr<IUpdateTransport> transport,
                  std::unique_ptr<IUpdateInstaller> installer, QString appVersion, QString appDir,
                  std::vector<int> seasonYears = {}, QObject* parent = nullptr);
    ~UpdateService() override;

    UpdateState state() const { return state_; }
    const UpdateOffer& offer() const { return offer_; }
    const InstallInfo& installInfo() const { return install_; }
    // Download progress 0..100, or -1 when the size is unknown.
    int percent() const { return percent_; }
    // Why the last check or download failed (shown only for a manual check or a failed download).
    const QString& error() const { return error_; }
    QString currentVersion() const { return version_; }
    bool busy() const;

    // Looks for a newer release. A manual check also clears "skip this version" and reports every outcome.
    void check(bool manual);
    // Downloads the offered installer (or opens the release page for a portable install); retries after a failure.
    void download();
    void cancelDownload();
    // Runs the update: macOS opens the disk image; elsewhere the app is asked to quit (installRequested) and the
    // installer starts as it exits.
    void install();
    // The quit that install() asked for was refused (unsaved work, a running match): back to Ready.
    void abortInstall();
    void skipThisVersion();
    void showReleasePage();
    // Called as the application exits; starts the installer when install() is pending.
    void aboutToQuit();

signals:
    void stateChanged();
    void progressChanged(int percent);
    // After a check ends. `manual` tells the UI whether to report the outcome.
    void checkFinished(bool manual);
    void installRequested();
    // Opening the installer failed (macOS); the verified download stays.
    void installFailed(const QString& error);

private:
    void setState(UpdateState state);
    void endCheck(UpdateState state, const QString& error = {});
    void failDownload(const QString& error);
    void onLatest(const HttpResult& result);
    void onSidecar(const HttpResult& result);
    void onDownloaded(const HttpResult& result);

    IKeyValueStore& store_;
    const ISystemProbe& probe_;
    std::unique_ptr<IUpdateTransport> transport_;
    std::unique_ptr<IUpdateInstaller> installer_;
    QString version_;
    QString appDir_;
    std::vector<int> seasonYears_;   // the seasons a packaged layout carries extensions for
    InstallInfo install_;
    UpdateState state_ = UpdateState::Idle;
    UpdateOffer offer_;
    QString error_;
    QString downloadPath_;
    int percent_ = -1;
    bool manual_ = false;
};

}  // namespace wpiemu::app
