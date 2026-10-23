#include <doctest/doctest.h>

#include <QCryptographicHash>
#include <QTemporaryDir>

#include <map>
#include <vector>

#include "common/key_value_store.h"
#include "common/update_install.h"
#include "common/update_offer.h"
#include "common/update_service.h"
#include "common/update_transport.h"
#include "test_support.h"
#include "update_fakes.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;

namespace {

const std::vector<int> kSeasons{2026, 2027};   // the seasons whose extensions a packaged layout carries

struct Rig {
    MemoryKeyValueStore store;
    FakeProbe probe;
    FakeTransport* transport = nullptr;
    FakeInstaller* installer = nullptr;
    std::unique_ptr<UpdateService> service;

    explicit Rig(const QString& version = QStringLiteral("0.0.5"), const std::function<void(FakeProbe&)>& tweak = {},
                 const QString& appDir = QStringLiteral("/tmp/.mount_x/usr/bin")) {
        probe.os = HostOs::Linux;
        probe.environment[QStringLiteral("APPIMAGE")] = QStringLiteral("/apps/WPI-Emulator.AppImage");
        probe.files.insert(QStringLiteral("/apps/WPI-Emulator.AppImage"));
        if (tweak) tweak(probe);
        auto t = std::make_unique<FakeTransport>();
        auto i = std::make_unique<FakeInstaller>();
        transport = t.get();
        installer = i.get();
        service = std::make_unique<UpdateService>(store, probe, std::move(t), std::move(i), version, appDir, kSeasons);
    }
};

}  // namespace

TEST_CASE("update offer parses tags, names installers and reads checksums") {
    CHECK(versionFromTag(QStringLiteral("v0.0.4")) == QVersionNumber(0, 0, 4));
    CHECK(versionFromTag(QStringLiteral("1.12.3")) == QVersionNumber(1, 12, 3));
    for (const char* odd : {"v1.2.3-rc1", "v.0.0.1", "alpha.0.0.1", "v1.2", "latest", ""}) CHECK_FALSE(versionFromTag(QString::fromLatin1(odd)));

    CHECK(tagFromReleaseLocation(QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases/tag/v0.0.4")) == QStringLiteral("v0.0.4"));
    CHECK_FALSE(tagFromReleaseLocation(QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases")));
    CHECK_FALSE(tagFromReleaseLocation(QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases/tag/v1.0.0-beta")));
    CHECK_FALSE(tagFromReleaseLocation(QStringLiteral("https://github.com/someone/else/releases/tag/v9.9.9")));
    CHECK_FALSE(tagFromReleaseLocation(QStringLiteral("https://evil.example/Overture-7421/wpi-emulator/releases/tag/v9.9.9")));

    const QVersionNumber v(0, 0, 6);
    CHECK(installerFileName(v, HostOs::Windows, QStringLiteral("x86_64")) == QStringLiteral("wpi-emulator-0.0.6-windows-x64.exe"));
    CHECK(installerFileName(v, HostOs::MacOs, QStringLiteral("arm64")) == QStringLiteral("wpi-emulator-0.0.6-macos-arm64-x64.dmg"));
    CHECK(installerFileName(v, HostOs::Linux, QStringLiteral("x86_64")) == kInstaller);
    CHECK_FALSE(installerFileName(v, HostOs::Windows, QStringLiteral("arm64")));
    CHECK_FALSE(installerFileName(v, HostOs::Linux, QStringLiteral("arm64")));

    const QString good = QStringLiteral("%1  %2\n").arg(QString(64, QLatin1Char('A')), kInstaller);
    CHECK(parseSha256Sidecar(good.toUtf8(), kInstaller) == QString(64, QLatin1Char('a')));
    CHECK_FALSE(parseSha256Sidecar(good.toUtf8(), QStringLiteral("other.exe")));
    CHECK_FALSE(parseSha256Sidecar("not a checksum", kInstaller));
    CHECK_FALSE(parseSha256Sidecar(QStringLiteral("abc123  %1").arg(kInstaller).toUtf8(), kInstaller));

    QTemporaryDir dir;
    const QString file = dir.filePath(QStringLiteral("f.bin"));
    wpiemu::app::testing::writeFile(file, kPayload);
    CHECK(sha256OfFile(file) == sha(kPayload));
    CHECK(sha256OfFile(dir.filePath(QStringLiteral("missing"))).isEmpty());
}

TEST_CASE("the real installer hashes on a worker thread and finds a writable place") {
    auto installer = createQtUpdateInstaller();
    QTemporaryDir dir;
    const QString file = dir.filePath(QStringLiteral("payload.bin"));
    writeFile(file, kPayload);
    QString hash;
    bool done = false;
    installer->sha256(file, [&](const QString& h) {
        hash = h;
        done = true;
    });
    REQUIRE(waitFor([&] { return done; }));
    CHECK(hash == sha(kPayload));

    InstallInfo appImage;
    appImage.kind = InstallKind::AppImage;
    appImage.appImagePath = dir.filePath(QStringLiteral("WPI.AppImage"));
    CHECK(installer->targetPath(appImage, kInstaller) == dir.filePath(kInstaller));   // beside the running AppImage

    installer->discard(file);
    CHECK_FALSE(QFileInfo::exists(file));
}

TEST_CASE("update transport only talks to GitHub over https") {
    CHECK(isAllowedUpdateUrl(QUrl(QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases/latest"))));
    CHECK(isAllowedUpdateUrl(QUrl(QStringLiteral("https://release-assets.githubusercontent.com/github-production-release-asset/1"))));
    CHECK_FALSE(isAllowedUpdateUrl(QUrl(QStringLiteral("http://github.com/x"))));
    CHECK_FALSE(isAllowedUpdateUrl(QUrl(QStringLiteral("https://api.github.com.evil.example/x"))));
    CHECK_FALSE(isAllowedUpdateUrl(QUrl(QStringLiteral("https://evilgithubusercontent.com/x"))));
    CHECK_FALSE(isAllowedUpdateUrl(QUrl(QStringLiteral("https://user@github.com/x"))));
    CHECK_FALSE(isAllowedUpdateUrl(QUrl(QStringLiteral("https://github.com:8443/x"))));
}

TEST_CASE("update transport e2e (opt-in): GitHub answers the lookups without its API") {
    if (!qEnvironmentVariableIsSet("WPIEMU_E2E_NETWORK")) {
        MESSAGE("skipped: set WPIEMU_E2E_NETWORK=1 to talk to github.com");
        return;
    }
    auto transport = createQtUpdateTransport(QStringLiteral("0.0.0-test"));
    HttpResult result;
    bool done = false;
    const auto finish = [&](const HttpResult& r) {
        result = r;
        done = true;
    };
    const auto run = [&](const std::function<void()>& start) {
        done = false;
        start();
        REQUIRE(waitFor([&] { return done; }, 60000));
    };

    run([&] { transport->head(latestReleaseUrl(), finish); });
    REQUIRE(result.ok);
    CHECK(result.status == 302);
    const auto tag = tagFromReleaseLocation(result.location);
    REQUIRE(tag);

    const QString sidecar = *installerFileName(*versionFromTag(*tag), HostOs::Windows, QStringLiteral("x86_64")) + QStringLiteral(".sha256");
    run([&] { transport->getText(releaseAssetUrl(*tag, sidecar), finish); });   // redirects to the asset host
    REQUIRE(result.ok);
    CHECK(parseSha256Sidecar(result.body, sidecar.chopped(7)));

    run([&] { transport->getText(releaseAssetUrl(*tag, QStringLiteral("no-such-file.sha256")), finish); });
    CHECK_FALSE(result.ok);
    CHECK(result.status == 404);

    QTemporaryDir dir;
    const QString target = dir.filePath(QStringLiteral("sidecar.txt"));
    run([&] { transport->download(releaseAssetUrl(*tag, sidecar), target, {}, finish); });
    CHECK(result.ok);
    CHECK(QFileInfo(target).size() > 64);
    CHECK_FALSE(QFileInfo::exists(target + QStringLiteral(".part")));
}

TEST_CASE("install kind follows the packaged layout") {
    FakeProbe probe;
    probe.os = HostOs::Windows;
    const QString bin = QStringLiteral("C:/Program Files/WPI Emulator/bin");
    CHECK(detectInstall(probe, bin, QStringLiteral("0.0.5"), kSeasons).kind == InstallKind::Dev);
    probe.files.insert(QStringLiteral("C:/Program Files/WPI Emulator/extensions/2026/halsim_wpiemu.dll"));
    CHECK(detectInstall(probe, bin, QStringLiteral("0.0.5"), kSeasons).kind == InstallKind::Portable);
    probe.files.insert(QStringLiteral("C:/Program Files/WPI Emulator/Uninstall.exe"));
    CHECK(detectInstall(probe, bin, QStringLiteral("0.0.5"), kSeasons).kind == InstallKind::Nsis);
    CHECK(detectInstall(probe, bin, QStringLiteral("0.0.0"), kSeasons).kind == InstallKind::Dev);

    probe.os = HostOs::MacOs;
    CHECK(detectInstall(probe, QStringLiteral("/Applications/wpiemu_app.app/Contents/MacOS"), QStringLiteral("0.0.5"), kSeasons).kind == InstallKind::MacBundle);
    CHECK(detectInstall(probe, QStringLiteral("/Volumes/WPI Emulator/wpiemu_app.app/Contents/MacOS"), QStringLiteral("0.0.5"), kSeasons).kind == InstallKind::Dev);

    probe.os = HostOs::Linux;
    probe.environment[QStringLiteral("APPIMAGE")] = QStringLiteral("/apps/WPI.AppImage");
    CHECK(detectInstall(probe, QStringLiteral("/tmp/.mount/usr/bin"), QStringLiteral("0.0.5"), kSeasons).kind == InstallKind::Dev);
    probe.files.insert(QStringLiteral("/apps/WPI.AppImage"));
    const InstallInfo appImage = detectInstall(probe, QStringLiteral("/tmp/.mount/usr/bin"), QStringLiteral("0.0.5"), kSeasons);
    CHECK(appImage.kind == InstallKind::AppImage);
    CHECK(appImage.appImagePath == QStringLiteral("/apps/WPI.AppImage"));
    probe.environment.clear();
    probe.files.insert(QStringLiteral("/opt/wpi/lib/wpiemu/2027/libhalsim_wpiemu.so"));
    CHECK(detectInstall(probe, QStringLiteral("/opt/wpi/bin"), QStringLiteral("0.0.5"), kSeasons).kind ==
          InstallKind::Portable);
}

TEST_CASE("a check finds the newer release without the GitHub API") {
    Rig rig;
    int finished = 0;
    bool manual = false;
    QObject::connect(rig.service.get(), &UpdateService::checkFinished, [&](bool m) { ++finished; manual = m; });
    rig.service->check(false);
    CHECK(rig.service->state() == UpdateState::Available);
    CHECK(rig.service->offer().tag == QStringLiteral("v0.0.6"));
    CHECK(rig.service->offer().fileName == kInstaller);
    CHECK(finished == 1);
    CHECK_FALSE(manual);
    for (const QString& url : rig.transport->requests) CHECK_FALSE(url.contains(QStringLiteral("api.github.com")));
}

TEST_CASE("a check ends quietly when there is nothing to install") {
    SUBCASE("same version") {
        Rig rig(QStringLiteral("0.0.6"));
        rig.service->check(true);
        CHECK(rig.service->state() == UpdateState::UpToDate);
    }
    SUBCASE("a newer local build") {
        Rig rig(QStringLiteral("0.1.0"));
        rig.service->check(false);
        CHECK(rig.service->state() == UpdateState::UpToDate);
    }
    SUBCASE("no stable release") {
        Rig rig;
        rig.transport->latest = {true, 404, {}, {}, {}};
        rig.service->check(false);
        CHECK(rig.service->state() == UpdateState::UpToDate);
    }
    SUBCASE("the installer of this system is not published") {
        Rig rig;
        rig.transport->texts.clear();
        rig.service->check(true);
        CHECK(rig.service->state() == UpdateState::UpToDate);
    }
    SUBCASE("a development build never asks") {
        Rig rig;
        rig.probe.environment.clear();
        rig.service->check(false);
        CHECK(rig.service->state() == UpdateState::Unsupported);
        CHECK(rig.transport->requests.empty());
    }
}

TEST_CASE("only a manual check reports failures") {
    Rig rig;
    rig.transport->latest = {false, 0, QStringLiteral("offline"), {}, {}};
    rig.service->check(false);
    CHECK(rig.service->state() == UpdateState::Idle);
    rig.service->check(true);
    CHECK(rig.service->state() == UpdateState::Failed);
    CHECK(rig.service->error() == QStringLiteral("offline"));

    Rig bad;
    bad.transport->texts.begin()->second.body = "garbage";
    bad.service->check(false);
    CHECK(bad.service->state() == UpdateState::Idle);
    bad.service->check(true);
    CHECK(bad.service->state() == UpdateState::Failed);
}

TEST_CASE("a skipped version stays quiet until a manual check") {
    Rig rig;
    rig.service->check(false);
    rig.service->skipThisVersion();
    CHECK(rig.service->state() == UpdateState::Idle);
    rig.service->check(false);
    CHECK(rig.service->state() == UpdateState::Idle);
    rig.service->check(true);
    CHECK(rig.service->state() == UpdateState::Available);
    rig.service->skipThisVersion();
    rig.service->check(false);
    CHECK(rig.service->state() == UpdateState::Idle);
}

TEST_CASE("a verified download is installed as the app quits") {
    Rig rig;
    rig.service->check(false);
    std::vector<int> percents;
    QObject::connect(rig.service.get(), &UpdateService::progressChanged, [&](int p) { percents.push_back(p); });
    int installRequests = 0;
    QObject::connect(rig.service.get(), &UpdateService::installRequested, [&] { ++installRequests; });

    rig.service->download();
    CHECK(rig.service->state() == UpdateState::Ready);
    CHECK(percents == std::vector<int>{50});
    CHECK(rig.installer->launched.empty());

    rig.service->install();
    CHECK(rig.service->state() == UpdateState::Installing);
    CHECK(installRequests == 1);
    CHECK(rig.installer->launched.empty());   // only once the app exits

    rig.service->abortInstall();
    CHECK(rig.service->state() == UpdateState::Ready);
    rig.service->aboutToQuit();
    CHECK(rig.installer->launched.empty());

    rig.service->install();
    rig.service->aboutToQuit();
    CHECK(rig.installer->launched == std::vector<QString>{rig.installer->target});
}

TEST_CASE("a download that fails or does not match its checksum is discarded") {
    Rig rig;
    rig.service->check(false);

    rig.installer->hash = sha("tampered");
    rig.service->download();
    CHECK(rig.service->state() == UpdateState::Failed);
    CHECK(rig.service->error().contains(QStringLiteral("checksum")));
    CHECK(rig.installer->discarded == std::vector<QString>{rig.installer->target});

    rig.installer->hash = sha(kPayload);
    rig.transport->downloadResult = {false, 0, QStringLiteral("connection lost"), {}, {}};
    rig.service->install();   // retries
    CHECK(rig.service->state() == UpdateState::Failed);
    CHECK(rig.service->error() == QStringLiteral("connection lost"));

    rig.transport->downloadResult = {true, 200, {}, {}, {}};
    rig.service->install();
    CHECK(rig.service->state() == UpdateState::Ready);
}

TEST_CASE("cancelling a download returns to the offer") {
    Rig rig;
    rig.service->check(false);
    rig.transport->holdDownload = true;
    rig.service->download();
    CHECK(rig.service->state() == UpdateState::Downloading);
    rig.service->cancelDownload();
    CHECK(rig.service->state() == UpdateState::Available);
    CHECK(rig.transport->aborts == 1);
    CHECK_FALSE(rig.installer->discarded.empty());
}

TEST_CASE("macOS opens the disk image and a portable install opens the release page") {
    Rig mac(QStringLiteral("0.0.5"),
            [](FakeProbe& p) {
                p.os = HostOs::MacOs;
                p.environment.clear();
                p.files.clear();
            },
            QStringLiteral("/Applications/wpiemu_app.app/Contents/MacOS"));
    CHECK(mac.service->installInfo().kind == InstallKind::MacBundle);
    int installRequests = 0;
    QObject::connect(mac.service.get(), &UpdateService::installRequested, [&] { ++installRequests; });
    mac.service->check(false);
    CHECK(mac.service->offer().fileName == QStringLiteral("wpi-emulator-0.0.6-macos-arm64-x64.dmg"));
    mac.service->download();
    CHECK(mac.service->state() == UpdateState::Ready);
    mac.service->install();
    CHECK(installRequests == 0);
    CHECK(mac.installer->launched.size() == 1);
    CHECK(mac.service->state() == UpdateState::Ready);

    mac.installer->launchOk = false;
    QString failure;
    QObject::connect(mac.service.get(), &UpdateService::installFailed, [&](const QString& e) { failure = e; });
    mac.service->install();
    CHECK(failure == QStringLiteral("refused"));

    Rig portable(QStringLiteral("0.0.5"),
                 [](FakeProbe& p) {
                     p.environment.clear();
                     p.files.insert(QStringLiteral("/opt/wpi/lib/wpiemu/2026/libhalsim_wpiemu.so"));
                 },
                 QStringLiteral("/opt/wpi/bin"));
    portable.service->check(false);
    CHECK(portable.service->state() == UpdateState::Available);
    CHECK(portable.service->offer().fileName.isEmpty());
    portable.service->download();
    CHECK(portable.installer->opened == std::vector<QString>{QStringLiteral("https://github.com/Overture-7421/wpi-emulator/releases/tag/v0.0.6")});
}
