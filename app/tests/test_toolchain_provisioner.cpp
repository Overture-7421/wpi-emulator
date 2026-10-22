#include <doctest/doctest.h>

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include "common/toolchain_provisioner.h"
#include "test_support.h"
#include "wpiemu/core/launcher/jdk_policy.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;

namespace {

const QByteArray kArchive = "pretend this is a JDK archive";

QString shaOf(const QByteArray& bytes) { return QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex()); }

class FakeDownloads final : public IUpdateTransport {
public:
    QByteArray payload = kArchive;
    bool fail = false;
    bool hold = false;
    int aborts = 0;
    std::vector<QString> requests;
    Done pending;

    void head(const QUrl&, Done done) override { done({}); }
    void getText(const QUrl&, Done done) override { done({}); }
    void download(const QUrl& url, const QString& file, Progress progress, Done done) override {
        requests.push_back(url.toString());
        progress(50, 100);
        if (hold) {
            pending = std::move(done);
            return;
        }
        if (fail) return done({false, 0, QStringLiteral("no route to host"), {}, {}});
        QFile out(file);
        REQUIRE(out.open(QIODevice::WriteOnly));
        out.write(payload);
        out.close();
        progress(100, 100);
        done({true, 200, {}, {}, {}});
    }
    void abort() override { ++aborts; }
};

// Unpacking is scripted: it lays out a JDK (or not) where the provisioner asked, and registers it with the fake machine.
class FakeEffects final : public IToolchainEffects {
public:
    FakeProbe* probe = nullptr;
    QString layout = QStringLiteral("bin");   // sub-folder of the unpacked archive holding the JDK ("" = directly in it)
    QString extractError;
    QString dllVersion;                       // Windows: version text written into the unpacked bin/msvcp140.dll ("" = no such dll)
    qint64 free = 100ll * 1024 * 1024 * 1024;
    int extracts = 0;

    void sha256(const QString& path, std::function<void(const QString&)> done) override {
        QString hash;
        {
            QFile f(path);
            if (f.open(QIODevice::ReadOnly)) hash = shaOf(f.readAll());
        }   // closed before the answer: Windows cannot delete an open file
        done(hash);
    }
    void extract(const QString&, const QString& destDir, std::function<void(const QString&)> done) override {
        ++extracts;
        if (!extractError.isEmpty()) return done(extractError);
        const QString root = layout.isEmpty() ? destDir : QDir::cleanPath(destDir + QLatin1Char('/') + layout);
        const bool windows = probe->os == HostOs::Windows;
        const QString java = root + (windows ? "/bin/java.exe" : "/bin/java");
        writeFile(java, "x");
        writeFile(root + (windows ? "/bin/javac.exe" : "/bin/javac"), "x");
        if (!dllVersion.isEmpty()) writeFile(root + "/bin/msvcp140.dll", dllVersion.toUtf8());
        probe->files.insert(QDir::cleanPath(java));
        probe->files.insert(QDir::cleanPath(root + (windows ? "/bin/javac.exe" : "/bin/javac")));
        ProcessResult run;
        run.started = true;
        run.output = QStringLiteral("openjdk version \"17.0.20\" 2026-07-21");
        probe->processes[QDir::cleanPath(java)] = run;
        done({});
    }
    void cancelExtract() override {}
    qint64 freeBytes(const QString&) override { return free; }
};

// The tests run the Java 17 of the 2026 season; this keeps their calls short.
struct Handle {
    std::unique_ptr<ToolchainProvisioner> real;
    static constexpr int kMajor = 17;
    Handle* operator->() { return this; }
    ToolchainProvisioner* get() { return real.get(); }
    void start() { real->start(kMajor); }
    QString installedHome() const { return real->installedHome(kMajor); }
    bool available() const { return real->availableFor(kMajor); }
    ProvisionState state() const { return real->state(); }
    const QString& error() const { return real->error(); }
    bool busy() const { return real->busy(); }
    void cancel() { real->cancel(); }
    bool remove() { return real->remove(); }
};

struct Fixture {
    QTemporaryDir dir;
    FakeProbe probe;
    FakeDownloads* downloads = nullptr;
    FakeEffects* effects = nullptr;
    Handle provisioner;
    int finishedOk = 0, finishedFailed = 0, lastPercent = -2;

    QString toolchains() const { return QDir::cleanPath(dir.path() + "/toolchains"); }
    QString appDir() const { return QDir::cleanPath(dir.path() + "/app"); }

    explicit Fixture(HostOs os = HostOs::Linux, bool withAsset = true) {
        probe.os = os;
        probe.versionHook = [](const QString& path) {   // DLLs here are small files whose text is their version
            QFile f(path);
            return f.open(QIODevice::ReadOnly) ? QString::fromUtf8(f.readAll()).trimmed() : QString();
        };
        auto d = std::make_unique<FakeDownloads>();
        auto e = std::make_unique<FakeEffects>();
        downloads = d.get();
        effects = e.get();
        effects->probe = &probe;
        JavaAsset asset;
        asset.major = 17;
        asset.platform = QStringLiteral("test");
        asset.version = QStringLiteral("17.0.20");
        asset.url = QUrl(QStringLiteral("https://github.com/adoptium/temurin17-binaries/releases/download/jdk-17.0.20/jdk.zip"));
        asset.sha256 = shaOf(kArchive);
        asset.sizeBytes = static_cast<qint64>(kArchive.size());
        std::vector<JavaAsset> assets;
        if (withAsset) assets.push_back(asset);
        provisioner.real = std::make_unique<ToolchainProvisioner>(probe, std::move(d), std::move(e), toolchains(), appDir(), std::move(assets));
        QObject::connect(provisioner.get(), &ToolchainProvisioner::finished, [this](bool ok) { ok ? ++finishedOk : ++finishedFailed; });
        QObject::connect(provisioner.get(), &ToolchainProvisioner::progressChanged, [this](int p) { lastPercent = p; });
    }

    bool leftovers() const { return QDir(toolchains() + "/.download").exists() || QDir(toolchains() + "/.staging").exists(); }
};

}  // namespace

TEST_CASE("provisioner: downloads, verifies, unpacks and installs a finished JDK") {
    Fixture f;
    CHECK(f.provisioner->installedHome().isEmpty());
    f.provisioner->start();
    CHECK(f.provisioner->state() == ProvisionState::Ready);
    CHECK(f.finishedOk == 1);
    CHECK(f.lastPercent == 100);
    CHECK(f.downloads->requests.size() == 1);
    CHECK(f.downloads->requests[0].startsWith("https://github.com/"));
    const QString home = f.provisioner->installedHome();
    CHECK(home == f.toolchains() + "/temurin-17.0.20");
    CHECK(QFile::exists(home + "/.wpiemu-ready"));
    CHECK(QFile::exists(home + "/bin/java"));
    CHECK(!f.leftovers());
}

TEST_CASE("provisioner: a JDK nested in the archive's top folder is installed from there") {
    Fixture f;
    f.effects->layout = QStringLiteral("jdk-17.0.20+1");
    f.provisioner->start();
    REQUIRE(f.provisioner->state() == ProvisionState::Ready);
    CHECK(QFile::exists(f.provisioner->installedHome() + "/bin/java"));
    CHECK(!f.leftovers());
}

TEST_CASE("provisioner: a download that does not match the pinned checksum is deleted and never unpacked") {
    Fixture f;
    f.downloads->payload = "tampered";
    f.provisioner->start();
    CHECK(f.provisioner->state() == ProvisionState::Failed);
    CHECK(f.provisioner->error().contains("checksum"));
    CHECK(f.effects->extracts == 0);
    CHECK(f.finishedFailed == 1);
    CHECK(f.provisioner->installedHome().isEmpty());
    CHECK(!f.leftovers());
}

TEST_CASE("provisioner: network, disk space and unpacking failures leave nothing behind") {
    {
        Fixture f;
        f.downloads->fail = true;
        f.provisioner->start();
        CHECK(f.provisioner->state() == ProvisionState::Failed);
        CHECK(f.provisioner->error().contains("no route to host"));
        CHECK(!f.leftovers());
    }
    {
        Fixture f;
        f.effects->free = 10;   // less than three times the (tiny) archive
        f.provisioner->start();
        CHECK(f.provisioner->state() == ProvisionState::Failed);
        CHECK(f.provisioner->error().contains("disk space"));
        CHECK(f.downloads->requests.empty());
    }
    {
        Fixture f;
        f.effects->extractError = QStringLiteral("Unpacking failed: bad archive");
        f.provisioner->start();
        CHECK(f.provisioner->state() == ProvisionState::Failed);
        CHECK(f.provisioner->error().contains("bad archive"));
        CHECK(!f.leftovers());
    }
    {
        Fixture f(HostOs::Linux, false);
        f.provisioner->start();
        CHECK(f.provisioner->state() == ProvisionState::Failed);
        CHECK(!f.provisioner->available());
    }
}

TEST_CASE("provisioner: cancel stops the download and removes its files; an installed Java is not downloaded again") {
    Fixture f;
    f.downloads->hold = true;
    f.provisioner->start();
    CHECK(f.provisioner->state() == ProvisionState::Downloading);
    CHECK(f.provisioner->busy());
    f.provisioner->cancel();
    CHECK(f.provisioner->state() == ProvisionState::Idle);
    CHECK(f.downloads->aborts == 1);
    CHECK(!f.leftovers());
    f.downloads->pending({true, 200, {}, {}, {}});   // a late answer of the cancelled run changes nothing
    CHECK(f.provisioner->state() == ProvisionState::Idle);

    f.downloads->hold = false;
    f.provisioner->start();
    REQUIRE(f.provisioner->state() == ProvisionState::Ready);
    const std::size_t requests = f.downloads->requests.size();
    f.provisioner->start();
    CHECK(f.downloads->requests.size() == requests);
    CHECK(f.provisioner->state() == ProvisionState::Ready);

    CHECK(f.provisioner->remove());
    CHECK(f.provisioner->installedHome().isEmpty());
    CHECK(f.provisioner->state() == ProvisionState::Idle);
}

TEST_CASE("provisioner (Windows): an old Visual C++ runtime in the JDK is replaced by the app's current one") {
    Fixture f(HostOs::Windows);
    f.effects->dllVersion = QStringLiteral("14.36.32532.0");
    writeFile(f.appDir() + "/msvcp140.dll", "14.44.35211.0");
    writeFile(f.appDir() + "/vcruntime140.dll", "14.44.35211.0");
    f.provisioner->start();
    REQUIRE(f.provisioner->state() == ProvisionState::Ready);
    const QString bin = f.provisioner->installedHome() + "/bin";
    CHECK(f.probe.fileVersion(bin + "/msvcp140.dll") == QStringLiteral("14.44.35211.0"));
    CHECK(QFile::exists(bin + "/vcruntime140.dll"));
}

TEST_CASE("provisioner (Windows): a current runtime is left alone, so nothing is copied") {
    Fixture f(HostOs::Windows);
    f.probe.environment[QStringLiteral("SystemRoot")] = f.dir.path() + "/Windows";
    writeFile(f.dir.path() + "/Windows/System32/msvcp140.dll", "14.51.36247.0");   // no dll in the JDK: the system's is loaded
    writeFile(f.appDir() + "/msvcp140.dll", "14.44.35211.0");
    f.provisioner->start();
    REQUIRE(f.provisioner->state() == ProvisionState::Ready);
    CHECK(!QFile::exists(f.provisioner->installedHome() + "/bin/msvcp140.dll"));
}

TEST_CASE("provisioner (Windows): when no current runtime exists the install is refused rather than kept broken") {
    Fixture f(HostOs::Windows);
    f.effects->dllVersion = QStringLiteral("14.36.32532.0");
    f.provisioner->start();   // the app folder has no runtime to offer
    CHECK(f.provisioner->state() == ProvisionState::Failed);
    CHECK(f.provisioner->error().contains("Redistributable"));
    CHECK(f.provisioner->installedHome().isEmpty());
    CHECK(!f.leftovers());
}

TEST_CASE("pinned Java downloads exist for the supported computers and only come from github.com") {
    const auto windows = ToolchainProvisioner::pinnedAssetsFor(HostOs::Windows, QStringLiteral("x86_64"));
    REQUIRE(!windows.empty());
    CHECK(windows[0].major == 17);
    CHECK(windows[0].platform == QStringLiteral("windows-x64"));
    CHECK(windows[0].url.host() == QStringLiteral("github.com"));
    CHECK(windows[0].sha256.size() == 64);
    CHECK(windows[0].sizeBytes > 0);
    CHECK(ToolchainProvisioner::pinnedAssetsFor(HostOs::MacOs, QStringLiteral("arm64"))[0].platform == QStringLiteral("macos-arm64"));
    CHECK(ToolchainProvisioner::pinnedAssetsFor(HostOs::Linux, QStringLiteral("x86_64"))[0].platform == QStringLiteral("linux-x64"));
    CHECK(ToolchainProvisioner::pinnedAssetsFor(HostOs::Linux, QStringLiteral("arm64")).empty());
}

TEST_CASE("the Java of a season is pinned per season; a season the pins do not know gets no download") {
    CHECK(ToolchainProvisioner::javaMajorForSeason(2026) == 17);
    CHECK(!ToolchainProvisioner::javaMajorForSeason(2099).has_value());   // never guess: a wrong Java cannot compile the robot
    Fixture f;
    CHECK(f.provisioner.get()->availableFor(17));
    CHECK(!f.provisioner.get()->availableFor(25));
    f.provisioner.get()->start(25);
    CHECK(f.provisioner->state() == ProvisionState::Failed);
    CHECK(f.provisioner->error().contains("Java 25"));
    CHECK(f.downloads->requests.empty());
}
