#include "common/update_service.h"

#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QPointer>
#include <QProcess>
#include <QStandardPaths>
#include <QThreadPool>

#include <algorithm>

#include "common/update_offer.h"

namespace wpiemu::app {
namespace {

constexpr auto kSkippedKey = "updates/skippedVersion";

class QtUpdateInstaller final : public IUpdateInstaller {
public:
    ~QtUpdateInstaller() override = default;

    QString targetPath(const InstallInfo& install, const QString& fileName) override {
        QString folder;
        switch (install.kind) {
            case InstallKind::Nsis: folder = scratchFolder(); break;
            case InstallKind::AppImage: {
                // Next to the running AppImage when its folder is writable, so the new one replaces it in place.
                const QFileInfo dir(QFileInfo(install.appImagePath).absolutePath());
                folder = dir.isWritable() ? dir.absoluteFilePath() : downloadsFolder();
                break;
            }
            default: folder = downloadsFolder(); break;
        }
        if (folder.isEmpty() || !QDir().mkpath(folder)) return {};
        return QDir(folder).filePath(fileName);
    }

    void sha256(const QString& path, std::function<void(const QString&)> done) override {
        QPointer<QObject> context(&context_);
        QThreadPool::globalInstance()->start([path, done = std::move(done), context]() mutable {
            const QString hash = sha256OfFile(path);
            if (!context) return;
            QMetaObject::invokeMethod(
                context.data(), [done = std::move(done), hash] { done(hash); }, Qt::QueuedConnection);
        });
    }

    bool launch(const InstallInfo& install, const QString& path, QString* error) override {
        switch (install.kind) {
            case InstallKind::AppImage: {
                if (!QFile::setPermissions(path, QFile::permissions(path) | QFileDevice::ExeOwner | QFileDevice::ExeUser)) {
                    *error = QObject::tr("Could not make %1 executable").arg(path);
                    return false;
                }
                if (!QProcess::startDetached(path, {})) {
                    *error = QObject::tr("Could not start %1").arg(path);
                    return false;
                }
                return true;
            }
            default:
                // ShellExecute rather than a child process: the Windows installer asks for administrator rights.
                if (!QDesktopServices::openUrl(QUrl::fromLocalFile(path))) {
                    *error = QObject::tr("Could not open %1").arg(path);
                    return false;
                }
                return true;
        }
    }

    bool openUrl(const QUrl& url) override { return QDesktopServices::openUrl(url); }

    void discard(const QString& path) override {
        if (!path.isEmpty()) {
            QFile::remove(path);
            QFile::remove(path + QStringLiteral(".part"));
            return;
        }
        QDir(scratchFolder()).removeRecursively();
    }

private:
    static QString scratchFolder() { return QDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation)).filePath(QStringLiteral("wpiemu-update")); }
    static QString downloadsFolder() {
        const QString downloads = QStandardPaths::writableLocation(QStandardPaths::DownloadLocation);
        return downloads.isEmpty() ? QDir::homePath() : downloads;
    }

    QObject context_;
};

}  // namespace

std::unique_ptr<IUpdateInstaller> createQtUpdateInstaller() { return std::make_unique<QtUpdateInstaller>(); }

bool updateRestartsApp(InstallKind kind) { return kind == InstallKind::Nsis || kind == InstallKind::AppImage; }

UpdateService::UpdateService(IKeyValueStore& store, const ISystemProbe& probe, std::unique_ptr<IUpdateTransport> transport,
                             std::unique_ptr<IUpdateInstaller> installer, QString appVersion, QString appDir, QObject* parent)
    : QObject(parent),
      store_(store),
      probe_(probe),
      transport_(std::move(transport)),
      installer_(std::move(installer)),
      version_(std::move(appVersion)),
      appDir_(std::move(appDir)) {
    install_ = detectInstall(probe_, appDir_, version_);
    if (install_.kind == InstallKind::Nsis) installer_->discard({});   // the folder of an earlier update
    if (auto* app = QCoreApplication::instance()) connect(app, &QCoreApplication::aboutToQuit, this, &UpdateService::aboutToQuit);
}

UpdateService::~UpdateService() = default;

bool UpdateService::busy() const {
    return state_ == UpdateState::Checking || state_ == UpdateState::Downloading || state_ == UpdateState::Verifying ||
           state_ == UpdateState::Installing;
}

void UpdateService::setState(UpdateState state) {
    if (state_ == state) return;
    state_ = state;
    emit stateChanged();
}

void UpdateService::endCheck(UpdateState state, const QString& error) {
    error_ = error;
    setState(state);
    emit checkFinished(manual_);
}

void UpdateService::check(bool manual) {
    if (busy() || state_ == UpdateState::Ready) return;
    manual_ = manual;
    error_.clear();
    offer_ = {};
    if (manual) store_.remove(QLatin1String(kSkippedKey));
    install_ = detectInstall(probe_, appDir_, version_);
    if (install_.kind == InstallKind::Dev || QVersionNumber::fromString(version_).isNull()) return endCheck(UpdateState::Unsupported);
    setState(UpdateState::Checking);
    transport_->head(latestReleaseUrl(), [this](const HttpResult& result) { onLatest(result); });
}

void UpdateService::onLatest(const HttpResult& result) {
    if (!result.ok) return endCheck(manual_ ? UpdateState::Failed : UpdateState::Idle, result.error);
    // No redirect to a stable release tag means nothing is published (yet).
    const auto tag = tagFromReleaseLocation(result.location);
    const auto latest = tag ? versionFromTag(*tag) : std::nullopt;
    const QVersionNumber current = QVersionNumber::fromString(version_);
    if (!latest || *latest <= current) return endCheck(UpdateState::UpToDate);
    if (!manual_ && store_.value(QLatin1String(kSkippedKey)).toString() == *tag) return endCheck(UpdateState::Idle);

    offer_ = {};
    offer_.tag = *tag;
    offer_.version = *latest;
    offer_.pageUrl = releasePageUrl(*tag);
    if (install_.kind == InstallKind::Portable) return endCheck(UpdateState::Available);

    const auto fileName = installerFileName(*latest, probe_.hostOs(), probe_.cpuArchitecture());
    if (!fileName) return endCheck(UpdateState::UpToDate);   // no installer for this computer
    offer_.fileName = *fileName;
    offer_.assetUrl = releaseAssetUrl(*tag, *fileName);
    offer_.sidecarUrl = releaseAssetUrl(*tag, *fileName + QStringLiteral(".sha256"));
    transport_->getText(offer_.sidecarUrl, [this](const HttpResult& r) { onSidecar(r); });
}

void UpdateService::onSidecar(const HttpResult& result) {
    // 404: the release has no installer for this system (yet). Not an error.
    if (!result.ok && result.status == 404) return endCheck(UpdateState::UpToDate);
    if (!result.ok) return endCheck(manual_ ? UpdateState::Failed : UpdateState::Idle, result.error);
    const auto hash = parseSha256Sidecar(result.body, offer_.fileName);
    if (!hash) return endCheck(manual_ ? UpdateState::Failed : UpdateState::Idle, tr("The checksum file of %1 is not valid").arg(offer_.fileName));
    offer_.sha256 = *hash;
    endCheck(UpdateState::Available);
}

void UpdateService::download() {
    if (state_ != UpdateState::Available && state_ != UpdateState::Failed) return;
    if (offer_.tag.isEmpty()) return;
    if (offer_.fileName.isEmpty()) return showReleasePage();
    downloadPath_ = installer_->targetPath(install_, offer_.fileName);
    if (downloadPath_.isEmpty()) return failDownload(tr("There is no writable folder to download the update to"));
    error_.clear();
    percent_ = -1;
    setState(UpdateState::Downloading);
    transport_->download(
        offer_.assetUrl, downloadPath_,
        [this](qint64 received, qint64 total) {
            const int next = total > 0 ? static_cast<int>(std::clamp<qint64>(received * 100 / total, 0, 100)) : -1;
            if (next == percent_) return;
            percent_ = next;
            emit progressChanged(next);
        },
        [this](const HttpResult& result) { onDownloaded(result); });
}

void UpdateService::onDownloaded(const HttpResult& result) {
    if (!result.ok) return failDownload(result.error);
    setState(UpdateState::Verifying);
    const QString path = downloadPath_;
    installer_->sha256(path, [this, path](const QString& hash) {
        if (state_ != UpdateState::Verifying || path != downloadPath_) return;
        if (hash.isEmpty() || hash != offer_.sha256) {
            failDownload(tr("The downloaded file does not match its checksum, so it was discarded"));
            return;
        }
        setState(UpdateState::Ready);
    });
}

void UpdateService::failDownload(const QString& error) {
    if (!downloadPath_.isEmpty()) installer_->discard(downloadPath_);
    downloadPath_.clear();
    error_ = error;
    setState(UpdateState::Failed);
}

void UpdateService::cancelDownload() {
    if (state_ != UpdateState::Downloading) return;
    transport_->abort();
    installer_->discard(downloadPath_);
    downloadPath_.clear();
    percent_ = -1;
    setState(UpdateState::Available);
}

void UpdateService::install() {
    if (state_ == UpdateState::Failed || state_ == UpdateState::Available) return download();
    if (state_ != UpdateState::Ready) return;
    if (!updateRestartsApp(install_.kind)) {
        QString problem;
        if (!installer_->launch(install_, downloadPath_, &problem)) {
            error_ = problem;
            emit installFailed(problem);
        }
        return;
    }
    setState(UpdateState::Installing);
    emit installRequested();
}

void UpdateService::abortInstall() {
    if (state_ == UpdateState::Installing) setState(UpdateState::Ready);
}

void UpdateService::aboutToQuit() {
    if (state_ != UpdateState::Installing) return;
    QString problem;
    installer_->launch(install_, downloadPath_, &problem);
}

void UpdateService::skipThisVersion() {
    if (offer_.tag.isEmpty() || busy()) return;
    store_.setValue(QLatin1String(kSkippedKey), offer_.tag);
    store_.sync();
    if (!downloadPath_.isEmpty()) installer_->discard(downloadPath_);
    downloadPath_.clear();
    setState(UpdateState::Idle);
}

void UpdateService::showReleasePage() {
    if (!offer_.pageUrl.isEmpty()) installer_->openUrl(offer_.pageUrl);
}

}  // namespace wpiemu::app
