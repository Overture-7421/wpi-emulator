#include "common/toolchain_provisioner.h"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QPointer>
#include <QProcess>
#include <QRegularExpression>
#include <QStorageInfo>
#include <QThreadPool>

#include <algorithm>

#include "common/update_offer.h"
#include "pins/toolchain_pins.h"
#include "wpiemu/core/launcher/jdk_policy.h"

namespace wpiemu::app {

namespace {

constexpr qint64 kSpaceFactor = 3;   // download + unpacked tree + headroom

QString join(const QString& a, const QString& b) { return QDir::cleanPath(a + QLatin1Char('/') + b); }

class QtToolchainEffects final : public IToolchainEffects {
public:
    void sha256(const QString& path, std::function<void(const QString&)> done) override {
        QPointer<QObject> context(&context_);
        QThreadPool::globalInstance()->start([path, done = std::move(done), context]() mutable {
            const QString hash = sha256OfFile(path);
            if (!context) return;
            QMetaObject::invokeMethod(
                context.data(), [done = std::move(done), hash] { done(hash); }, Qt::QueuedConnection);
        });
    }

    void extract(const QString& archive, const QString& destDir, std::function<void(const QString&)> done) override {
        cancelExtract();
        auto process = std::make_unique<QProcess>();
        process->setProcessChannelMode(QProcess::MergedChannels);
        QProcess* raw = process.get();
        process_ = std::move(process);
        QObject::connect(raw, &QProcess::errorOccurred, &context_, [this, raw, done](QProcess::ProcessError e) {
            if (e != QProcess::FailedToStart) return;
            const QString reason = QObject::tr("Could not start the unpacking tool (%1).").arg(raw->program());
            finishProcess(raw, reason, done);
        });
        QObject::connect(raw, &QProcess::finished, &context_, [this, raw, done](int code, QProcess::ExitStatus status) {
            const bool ok = status == QProcess::NormalExit && code == 0;
            finishProcess(raw, ok ? QString() : QObject::tr("Unpacking failed: %1").arg(QString::fromLocal8Bit(raw->readAll()).trimmed().left(300)), done);
        });
#if defined(Q_OS_WIN)
        // Windows' own bsdtar reads .zip; a tar from Git or MSYS on PATH does not.
        const QString root = qEnvironmentVariable("SystemRoot");
        raw->start(root.isEmpty() ? QStringLiteral("tar.exe") : join(root, QStringLiteral("System32/tar.exe")), {QStringLiteral("-xf"), QDir::toNativeSeparators(archive), QStringLiteral("-C"), QDir::toNativeSeparators(destDir)});
#else
        raw->start(QStringLiteral("tar"), {QStringLiteral("-xzf"), archive, QStringLiteral("-C"), destDir});
#endif
    }

    void cancelExtract() override {
        if (!process_) return;
        process_->disconnect();
        process_->kill();
        process_->waitForFinished(2000);
        process_.reset();
    }

    qint64 freeBytes(const QString& dir) override {
        const QStorageInfo info(dir);
        return info.isValid() ? info.bytesAvailable() : -1;
    }

private:
    void finishProcess(QProcess* raw, const QString& error, const std::function<void(const QString&)>& done) {
        if (process_.get() != raw) return;
        process_.release()->deleteLater();
        done(error);
    }

    QObject context_;
    std::unique_ptr<QProcess> process_;
};

// The folder holding bin/java(.exe), `root` itself or up to three levels below (macOS: jdk-17/Contents/Home).
QString findJdkRoot(const QString& root, bool windows) {
    const QString java = windows ? QStringLiteral("bin/java.exe") : QStringLiteral("bin/java");
    if (QFileInfo(join(root, java)).isFile()) return root;
    QDirIterator it(root, QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString dir = it.next();
        if (dir.mid(root.size()).count(QLatin1Char('/')) > 3) continue;
        if (QFileInfo(join(dir, java)).isFile()) return dir;
    }
    return {};
}

}  // namespace

std::unique_ptr<IToolchainEffects> createQtToolchainEffects() { return std::make_unique<QtToolchainEffects>(); }

std::vector<JavaAsset> ToolchainProvisioner::pinnedAssetsFor(HostOs os, const QString& cpu) {
    const bool x64 = cpu == QLatin1String("x86_64") || cpu == QLatin1String("amd64");
    const bool arm64 = cpu == QLatin1String("arm64") || cpu == QLatin1String("aarch64");
    QString key;
    switch (os) {
        case HostOs::Windows: key = x64 ? QStringLiteral("windows-x64") : QString(); break;
        case HostOs::Linux: key = x64 ? QStringLiteral("linux-x64") : QString(); break;
        case HostOs::MacOs: key = x64 ? QStringLiteral("macos-x64") : arm64 ? QStringLiteral("macos-arm64") : QString(); break;
    }
    std::vector<JavaAsset> found;
    if (key.isEmpty()) return found;
    for (const pins::JdkAsset& a : pins::kJdkAssets) {
        if (key != QLatin1String(a.platform)) continue;
        JavaAsset asset;
        asset.major = a.major;
        asset.platform = key;
        asset.version = QString::fromLatin1(a.version);
        asset.url = QUrl(QString::fromLatin1(a.url));
        asset.sha256 = QString::fromLatin1(a.sha256);
        asset.sizeBytes = a.sizeBytes;
        found.push_back(std::move(asset));
    }
    return found;
}

std::optional<int> ToolchainProvisioner::javaMajorForSeason(int wpilibYear) {
    for (const pins::JdkSeason& s : pins::kJdkSeasons)
        if (s.year == wpilibYear) return s.major;
    return std::nullopt;
}

const JavaAsset* ToolchainProvisioner::assetFor(int major) const {
    for (const JavaAsset& a : assets_)
        if (a.major == major) return &a;
    return nullptr;
}

ToolchainProvisioner::ToolchainProvisioner(const ISystemProbe& probe, std::unique_ptr<IUpdateTransport> transport,
                                           std::unique_ptr<IToolchainEffects> effects, QString toolchainsDir, QString appDir,
                                           std::vector<JavaAsset> assets, QObject* parent)
    : QObject(parent),
      probe_(probe),
      transport_(std::move(transport)),
      effects_(std::move(effects)),
      toolchainsDir_(QDir::cleanPath(std::move(toolchainsDir))),
      appDir_(QDir::cleanPath(std::move(appDir))),
      assets_(std::move(assets)) {
    // Leftovers of a run that was interrupted by closing the app.
    if (!toolchainsDir_.isEmpty()) cleanWork();
}

ToolchainProvisioner::~ToolchainProvisioner() {
    if (busy()) {
        transport_->abort();
        effects_->cancelExtract();
    }
}

bool ToolchainProvisioner::busy() const {
    return state_ == ProvisionState::Downloading || state_ == ProvisionState::Verifying || state_ == ProvisionState::Unpacking ||
           state_ == ProvisionState::Finishing;
}

QString ToolchainProvisioner::downloadFile() const {
    return join(toolchainsDir_, QStringLiteral(".download/") + (asset_ ? QFileInfo(asset_->url.path()).fileName() : QStringLiteral("java.bin")));
}

QString ToolchainProvisioner::stagingDir() const { return join(toolchainsDir_, QStringLiteral(".staging")); }

QString ToolchainProvisioner::installedHome(int major) const {
    const bool windows = probe_.hostOs() == HostOs::Windows;
    const auto home = core::launcher::findManagedJdk(std::filesystem::path(toolchainsDir_.toStdU16String()), windows, major);
    return home ? QString::fromStdU16String(home->generic_u16string()) : QString();
}

qint64 ToolchainProvisioner::installedBytes() const {
    qint64 total = 0;
    for (const QString& name : QDir(toolchainsDir_).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        QDirIterator it(join(toolchainsDir_, name), QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
            it.next();
            total += it.fileInfo().size();
        }
    }
    return total;
}

void ToolchainProvisioner::setState(ProvisionState state) {
    if (state == state_) return;
    state_ = state;
    emit stateChanged();
}

void ToolchainProvisioner::cleanWork() {
    QDir(join(toolchainsDir_, QStringLiteral(".download"))).removeRecursively();
    QDir(stagingDir()).removeRecursively();
}

void ToolchainProvisioner::fail(const QString& error) {
    ++generation_;
    cleanWork();
    error_ = error;
    percent_ = -1;
    setState(ProvisionState::Failed);
    emit finished(false);
}

void ToolchainProvisioner::cancel() {
    if (!busy()) return;
    ++generation_;
    transport_->abort();
    effects_->cancelExtract();
    cleanWork();
    error_.clear();
    percent_ = -1;
    setState(ProvisionState::Idle);
    emit finished(false);
}

void ToolchainProvisioner::start(int major) {
    if (busy()) return;
    error_.clear();
    const JavaAsset* pinned = assetFor(major);
    if (pinned == nullptr) return fail(tr("Downloading Java %1 is not available for this computer. Install the WPILib bundle or a JDK %1 instead.").arg(major));
    asset_ = *pinned;
    if (!installedHome(major).isEmpty()) {
        setState(ProvisionState::Ready);
        emit finished(true);
        return;
    }
    if (!QDir().mkpath(toolchainsDir_)) return fail(tr("Could not create %1.").arg(QDir::toNativeSeparators(toolchainsDir_)));
    cleanWork();
    const qint64 free = effects_->freeBytes(toolchainsDir_);
    if (free >= 0 && free < asset_->sizeBytes * kSpaceFactor)
        return fail(tr("Not enough free disk space: about %1 MB is needed.").arg(asset_->sizeBytes * kSpaceFactor / (1024 * 1024)));
    if (!QDir().mkpath(QFileInfo(downloadFile()).absolutePath())) return fail(tr("Could not create the download folder."));

    const quint64 gen = ++generation_;
    percent_ = 0;
    setState(ProvisionState::Downloading);
    emit progressChanged(0);
    transport_->download(
        asset_->url, downloadFile(),
        [this, gen](qint64 received, qint64 total) {
            if (gen != generation_) return;
            const qint64 denominator = total > 0 ? total : asset_->sizeBytes;
            const int percent = denominator > 0 ? static_cast<int>(std::clamp<qint64>(received * 100 / denominator, 0, 100)) : -1;
            if (percent == percent_) return;
            percent_ = percent;
            emit progressChanged(percent);
        },
        [this, gen](const HttpResult& result) {
            if (gen == generation_) onDownloaded(result);
        });
}

void ToolchainProvisioner::onDownloaded(const HttpResult& result) {
    if (!result.ok) return fail(tr("The download failed: %1").arg(result.error.isEmpty() ? tr("no answer") : result.error));
    percent_ = -1;
    setState(ProvisionState::Verifying);
    const quint64 gen = generation_;
    effects_->sha256(downloadFile(), [this, gen](const QString& hash) {
        if (gen == generation_) onHashed(hash);
    });
}

void ToolchainProvisioner::onHashed(const QString& hash) {
    if (hash.isEmpty() || hash.compare(asset_->sha256, Qt::CaseInsensitive) != 0)
        return fail(tr("The downloaded file does not match its published checksum, so it was deleted. Try again, or install Java yourself."));
    QDir(stagingDir()).removeRecursively();
    if (!QDir().mkpath(stagingDir())) return fail(tr("Could not create a folder to unpack Java."));
    setState(ProvisionState::Unpacking);
    const quint64 gen = generation_;
    effects_->extract(downloadFile(), stagingDir(), [this, gen](const QString& error) {
        if (gen == generation_) onExtracted(error);
    });
}

void ToolchainProvisioner::onExtracted(const QString& error) {
    if (!error.isEmpty()) return fail(error);
    setState(ProvisionState::Finishing);
    finish();
}

// Windows only. WPILib refuses a JVM that loads a Visual C++ runtime older than 14.40, and a JDK's own bin folder wins over the
// system's. So the copy next to the app (which the installer ships current) replaces the JDK's, but only when the runtime this
// Java would load is missing or too old: a computer with a current runtime and a clean JDK is left alone.
bool ToolchainProvisioner::overlayRuntime(const QString& jdkRoot, QString* error) {
    if (probe_.hostOs() != HostOs::Windows) return true;
    const QString min = QString::fromLatin1(core::launcher::kMinVcRuntime.data(), static_cast<int>(core::launcher::kMinVcRuntime.size()));
    const QString bin = join(jdkRoot, QStringLiteral("bin"));
    const auto current = [&](const QString& version) { return !version.isEmpty() && core::launcher::compareVersions(version.toStdString(), min.toStdString()) >= 0; };
    if (current(loadedVcRuntime(probe_, bin))) return true;

    const QString source = join(appDir_, QStringLiteral("msvcp140.dll"));
    if (!current(probe_.fileVersion(source))) {
        *error = tr("This computer's Visual C++ runtime is older than %1 and no newer copy came with WPI Emulator. "
                    "Install the latest \"Microsoft Visual C++ Redistributable\" (x64) and try again.").arg(min);
        return false;
    }
    const QStringList names = QDir(appDir_).entryList({QStringLiteral("msvcp140*.dll"), QStringLiteral("vcruntime140*.dll"), QStringLiteral("concrt140.dll")}, QDir::Files);
    for (const QString& name : names) {
        const QString target = join(bin, name);
        QFile::remove(target);
        if (!QFile::copy(join(appDir_, name), target)) {
            *error = tr("Could not update the Visual C++ runtime inside the downloaded Java (%1).").arg(name);
            return false;
        }
    }
    if (!current(probe_.fileVersion(join(bin, QStringLiteral("msvcp140.dll"))))) {
        *error = tr("The downloaded Java still loads an old Visual C++ runtime.");
        return false;
    }
    return true;
}

void ToolchainProvisioner::finish() {
    const bool windows = probe_.hostOs() == HostOs::Windows;
    const QString root = findJdkRoot(stagingDir(), windows);
    if (root.isEmpty()) return fail(tr("The downloaded Java is incomplete (no bin/java)."));

    QString problem;
    if (!overlayRuntime(root, &problem)) return fail(problem);

    const QString javaExe = join(root, windows ? QStringLiteral("bin/java.exe") : QStringLiteral("bin/java"));
    const QString javacExe = join(root, windows ? QStringLiteral("bin/javac.exe") : QStringLiteral("bin/javac"));
    if (!probe_.fileExists(javacExe)) return fail(tr("The downloaded Java is not a full JDK (no javac)."));
    const ProcessResult run = probe_.run(javaExe, {QStringLiteral("-version")}, 20000);
    const auto major = run.started ? core::launcher::parseJavaMajor(run.output.toStdString()) : std::nullopt;
    if (!major || *major < asset_->major)
        return fail(tr("The downloaded Java does not run on this computer: %1").arg(run.output.trimmed().left(300)));

    const QString finalDir = join(toolchainsDir_, QString::fromLatin1(pins::kJdkVendor) + QLatin1Char('-') + asset_->version);
    QDir(finalDir).removeRecursively();
    if (!QDir().rename(root, finalDir)) return fail(tr("Could not move Java into %1.").arg(QDir::toNativeSeparators(finalDir)));
    QFile marker(join(finalDir, QString::fromLatin1(core::launcher::kManagedJdkMarker.data(), static_cast<int>(core::launcher::kManagedJdkMarker.size()))));
    if (!marker.open(QIODevice::WriteOnly) || marker.write(asset_->version.toUtf8()) < 0) return fail(tr("Could not finish installing Java."));
    marker.close();

    // Older managed versions of this Java are replaced, not kept; other majors belong to other seasons.
    for (const QString& name : QDir(toolchainsDir_).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        const int digits = name.indexOf(QRegularExpression(QStringLiteral("\\d")));
        const bool sameMajor = digits >= 0 && name.mid(digits).section(QLatin1Char('.'), 0, 0).toInt() == asset_->major;
        if (sameMajor && join(toolchainsDir_, name) != finalDir) QDir(join(toolchainsDir_, name)).removeRecursively();
    }
    cleanWork();
    percent_ = -1;
    error_.clear();
    setState(ProvisionState::Ready);
    emit finished(true);
}

bool ToolchainProvisioner::remove() {
    if (busy()) return false;
    for (const QString& name : QDir(toolchainsDir_).entryList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::Hidden))
        QDir(join(toolchainsDir_, name)).removeRecursively();
    const bool gone = installedHome(0).isEmpty();
    setState(ProvisionState::Idle);
    return gone;
}

}  // namespace wpiemu::app
