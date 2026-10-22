#pragma once

// Scripted stand-ins for the updater's network and machine seams.

#include <QCryptographicHash>

#include <functional>
#include <map>
#include <vector>

#include "common/update_service.h"
#include "common/update_transport.h"

namespace wpiemu::app::testing {

inline const QString kInstaller = QStringLiteral("wpi-emulator-0.0.6-linux-x86_64.AppImage");
inline const QByteArray kPayload = "installer bytes";

inline QString sha(const QByteArray& bytes) { return QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex()); }

// Answers requests from a script, immediately; downloads can be held back to test cancelling.
class FakeTransport final : public IUpdateTransport {
public:
    HttpResult latest;                          // answer to the releases/latest HEAD
    std::map<QString, HttpResult> texts;        // by URL
    HttpResult downloadResult;
    bool holdDownload = false;
    std::vector<QString> requests;
    Done pendingDone;
    Progress pendingProgress;
    int aborts = 0;

    // A published v0.0.6 with an installer for Linux and macOS.
    FakeTransport() {
        latest = {true, 302, {}, QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases/tag/v0.0.6"), {}};
        for (const QString& name : {kInstaller, QStringLiteral("wpi-emulator-0.0.6-macos-arm64-x64.dmg")})
            texts[QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases/download/v0.0.6/") + name + QStringLiteral(".sha256")] =
                {true, 200, {}, {}, (sha(kPayload) + QStringLiteral("  ") + name + QStringLiteral("\n")).toUtf8()};
        downloadResult = {true, 200, {}, {}, {}};
    }

    void head(const QUrl& url, Done done) override {
        requests.push_back(url.toString());
        done(latest);
    }
    void getText(const QUrl& url, Done done) override {
        requests.push_back(url.toString());
        const auto it = texts.find(url.toString());
        done(it == texts.end() ? HttpResult{false, 404, QStringLiteral("not found"), {}, {}} : it->second);
    }
    void download(const QUrl& url, const QString&, Progress progress, Done done) override {
        requests.push_back(url.toString());
        if (holdDownload) {
            pendingDone = std::move(done);
            pendingProgress = std::move(progress);
            return;
        }
        progress(50, 100);
        done(downloadResult);
    }
    void abort() override { ++aborts; }
};

class FakeInstaller final : public IUpdateInstaller {
public:
    QString target = QStringLiteral("/tmp/update/") + kInstaller;
    QString hash = sha(kPayload);
    std::vector<QString> launched, discarded, opened;
    bool launchOk = true;

    QString targetPath(const InstallInfo&, const QString&) override { return target; }
    void sha256(const QString&, std::function<void(const QString&)> done) override { done(hash); }
    bool launch(const InstallInfo&, const QString& path, QString* error) override {
        launched.push_back(path);
        if (!launchOk) *error = QStringLiteral("refused");
        return launchOk;
    }
    bool openUrl(const QUrl& url) override {
        opened.push_back(url.toString());
        return true;
    }
    void discard(const QString& path) override { discarded.push_back(path); }
};

}  // namespace wpiemu::app::testing
