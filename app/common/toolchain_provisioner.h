#pragma once

// Gets the emulator its own Java when the computer has none that can run a WPILib project
// (docs/reference/runtime.md).
// It runs only when asked: the Toolchain page offers it when no other JDK is usable, and never over an installed WPILib's JDK.

#include <QObject>
#include <QString>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "common/system_probe.h"
#include "common/update_transport.h"

namespace wpiemu::app {

// A pinned download from versions.json (the SHA-256 is the only trust anchor).
struct JavaAsset {
    int major = 0;      // the Java major this download is (17, 25...)
    QString platform;   // "windows-x64", "macos-arm64", ...
    QString version;    // "17.0.20.1"
    QUrl url;
    QString sha256;
    qint64 sizeBytes = 0;
};

// Everything the provisioner does to the machine besides plain file operations; scripted in tests.
class IToolchainEffects {
public:
    virtual ~IToolchainEffects() = default;
    // Hashes on a worker thread; `done` runs on the owner's thread with lower-case hex ("" when unreadable).
    virtual void sha256(const QString& path, std::function<void(const QString&)> done) = 0;
    // Unpacks a .zip or .tar.gz into `destDir` (which exists); `done` gets an empty string on success, else the reason.
    virtual void extract(const QString& archive, const QString& destDir, std::function<void(const QString&)> done) = 0;
    virtual void cancelExtract() = 0;
    virtual qint64 freeBytes(const QString& dir) = 0;   // -1 when unknown
};

std::unique_ptr<IToolchainEffects> createQtToolchainEffects();

enum class ProvisionState { Idle, Downloading, Verifying, Unpacking, Finishing, Ready, Failed };

class ToolchainProvisioner : public QObject {
    Q_OBJECT
public:
    // `toolchainsDir`: where installs live (core's managedToolchainsDir). `appDir`: the app's own folder, source of the
    // Visual C++ runtime a Windows install may need. `assets`: the pins for this computer, one per Java major (maybe none).
    ToolchainProvisioner(const ISystemProbe& probe, std::unique_ptr<IUpdateTransport> transport, std::unique_ptr<IToolchainEffects> effects,
                         QString toolchainsDir, QString appDir, std::vector<JavaAsset> assets, QObject* parent = nullptr);
    ~ToolchainProvisioner() override;

    // The pins for this computer ("windows-x64", "macos-arm64", "linux-x64"...) from the generated versions.json table.
    static std::vector<JavaAsset> pinnedAssetsFor(HostOs os, const QString& cpuArchitecture);
    // The Java major a WPILib season builds with (versions.json jdk.seasons); nullopt for a season the pins do not know, in which
    // case nothing is downloaded: guessing would install a Java the project cannot compile with.
    static std::optional<int> javaMajorForSeason(int wpilibYear);

    bool availableFor(int major) const { return assetFor(major) != nullptr; }
    const JavaAsset* assetFor(int major) const;
    ProvisionState state() const { return state_; }
    bool busy() const;
    int percent() const { return percent_; }                  // 0..100 while downloading, else -1
    const QString& error() const { return error_; }
    const QString& toolchainsDir() const { return toolchainsDir_; }
    // The finished install of Java `major` this computer would use (core's findManagedJdk); empty when there is none.
    QString installedHome(int major) const;
    qint64 installedBytes() const;   // every managed install

    // Downloads, verifies, unpacks and checks the pinned Java `major`. No-op while busy; a finished install is not repeated.
    void start(int major);
    void cancel();
    // Deletes every managed install (not while busy). True when nothing is left.
    bool remove();

signals:
    void stateChanged();
    void progressChanged(int percent);
    void finished(bool ok);   // after Ready or Failed; a cancel ends in Idle with finished(false)

private:
    void setState(ProvisionState state);
    void fail(const QString& error);
    void cleanWork();
    void onDownloaded(const HttpResult& result);
    void onHashed(const QString& hash);
    void onExtracted(const QString& error);
    void finish();
    bool overlayRuntime(const QString& jdkRoot, QString* error);
    QString downloadFile() const;
    QString stagingDir() const;

    const ISystemProbe& probe_;
    std::unique_ptr<IUpdateTransport> transport_;
    std::unique_ptr<IToolchainEffects> effects_;
    QString toolchainsDir_;
    QString appDir_;
    std::vector<JavaAsset> assets_;
    std::optional<JavaAsset> asset_;   // the download of the run in progress
    ProvisionState state_ = ProvisionState::Idle;
    QString error_;
    int percent_ = -1;
    quint64 generation_ = 0;   // callbacks of a cancelled run see a newer generation and do nothing
};

}  // namespace wpiemu::app
