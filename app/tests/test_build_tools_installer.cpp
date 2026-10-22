#include <doctest/doctest.h>

#include "common/build_tools_installer.h"
#include "test_support.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;

namespace {

class FakeEffects final : public IBuildToolsEffects {
public:
    bool winget = true;
    bool openOk = true;
    bool hold = false;
    int cancels = 0;
    QStringList lastArgs;
    std::vector<QUrl> opened;
    std::function<void(int, const QString&)> pending;
    int exitCode = 0;
    QString output;

    bool wingetAvailable() override { return winget; }
    void runWinget(const QStringList& args, std::function<void(int, const QString&)> done) override {
        lastArgs = args;
        if (hold) {
            pending = std::move(done);
            return;
        }
        done(exitCode, output);
    }
    void cancel() override { ++cancels; }
    bool openUrl(const QUrl& url) override {
        opened.push_back(url);
        return openOk;
    }
};

struct Fixture {
    FakeProbe probe;
    FakeEffects* effects = nullptr;
    std::unique_ptr<BuildToolsInstaller> installer;
    std::vector<bool> results;

    explicit Fixture(HostOs os = HostOs::Windows) {
        probe.os = os;
        auto e = std::make_unique<FakeEffects>();
        effects = e.get();
        installer = std::make_unique<BuildToolsInstaller>(probe, std::move(e));
        QObject::connect(installer.get(), &BuildToolsInstaller::finished, [this](bool ok) { results.push_back(ok); });
    }
};

}  // namespace

TEST_CASE("build tools: the exact winget command installs only the C++ workload and is never silent") {
    const QStringList args = BuildToolsInstaller::wingetArguments();
    CHECK(args.contains(QStringLiteral("Microsoft.VisualStudio.2022.BuildTools")));
    CHECK(args.contains(QStringLiteral("--accept-package-agreements")));
    const QString override = args.at(args.indexOf(QStringLiteral("--override")) + 1);
    CHECK(override.contains(QStringLiteral("--add Microsoft.VisualStudio.Workload.VCTools")));
    CHECK(override.contains(QStringLiteral("--passive")));   // the installer shows its progress
    CHECK(!override.contains(QStringLiteral("--quiet")));
    CHECK(BuildToolsInstaller::downloadPage().host() == QStringLiteral("aka.ms"));
}

TEST_CASE("build tools: only Windows offers the install, and nothing runs until start()") {
    Fixture mac(HostOs::MacOs);
    CHECK(!mac.installer->available());
    mac.installer->start();
    CHECK(mac.effects->lastArgs.isEmpty());
    CHECK(mac.effects->opened.empty());

    Fixture windows;
    CHECK(windows.installer->available());
    CHECK(windows.effects->lastArgs.isEmpty());   // constructing and checking start nothing
    CHECK(windows.installer->state() == BuildToolsState::Idle);
}

TEST_CASE("build tools: winget success, failure and cancellation") {
    {
        Fixture f;
        f.installer->start();
        CHECK(f.installer->state() == BuildToolsState::Done);
        CHECK(f.results == std::vector<bool>{true});
        CHECK(f.effects->lastArgs == BuildToolsInstaller::wingetArguments());
    }
    {
        Fixture f;
        f.effects->exitCode = 1602;
        f.effects->output = QStringLiteral("cancelled by the user");
        f.installer->start();
        CHECK(f.installer->state() == BuildToolsState::Failed);
        CHECK(f.installer->message().contains(QStringLiteral("1602")));
        CHECK(f.installer->message().contains(QStringLiteral("Re-check")));
        CHECK(f.results == std::vector<bool>{false});
    }
    {
        Fixture f;
        f.effects->hold = true;
        f.installer->start();
        CHECK(f.installer->busy());
        f.installer->start();   // a second click while installing does nothing
        f.installer->cancel();
        CHECK(!f.installer->busy());
        CHECK(f.effects->cancels == 1);
        f.effects->pending(0, {});   // the cancelled run answers late: ignored
        CHECK(f.installer->state() == BuildToolsState::Idle);
        CHECK(f.results == std::vector<bool>{false});
    }
}

TEST_CASE("build tools: without winget the Build Tools download page is opened and the user finishes by hand") {
    Fixture f;
    f.effects->winget = false;
    f.installer->start();
    REQUIRE(f.effects->opened.size() == 1);
    CHECK(f.effects->opened[0] == BuildToolsInstaller::downloadPage());
    CHECK(f.installer->state() == BuildToolsState::OpenedDownload);
    CHECK(f.installer->message().contains(QStringLiteral("Desktop development with C++")));
    CHECK(f.effects->lastArgs.isEmpty());

    Fixture blocked;
    blocked.effects->winget = false;
    blocked.effects->openOk = false;
    blocked.installer->start();
    CHECK(blocked.installer->state() == BuildToolsState::Failed);
}
