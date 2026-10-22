#include <doctest/doctest.h>

#include "common/robot_basics.h"
#include "test_support.h"
#include "wpiemu/core/config/config_validator.h"
#ifdef WPIEMU_APP_HAS_RUNVIEW
#include "runview/sfx_catalog.h"
#endif

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

TEST_CASE("services register the built-in content") {
    TestEnvironment env;
    AppServices& s = *env.services;
    CHECK(s.startupProblems().isEmpty());
    CHECK(s.plugins().size() >= 1);
    CHECK(s.plugins().findByYear(2026) != nullptr);
    CHECK(s.drivetrains().contains(sdk::drivetrain_kind::kSwerve));
    CHECK(s.drivetrains().contains(sdk::drivetrain_kind::kTank));
    CHECK(s.drivetrains().contains(sdk::drivetrain_kind::kMecanum));
    CHECK(s.mechanisms().size() >= 1);
    CHECK(!s.demo());
}

#ifdef WPIEMU_APP_HAS_RUNVIEW
TEST_CASE("every built-in game plugin ships the sounds it declares") {
    TestEnvironment env;
    const auto& plugins = env.services->plugins().list();
    REQUIRE_FALSE(plugins.empty());
    for (const sdk::IGameModule* module : plugins) {
        CAPTURE(module->metadata().id);
        wpiemu::app::runview::SfxCatalog catalog;
        const QStringList problems = catalog.set(QString::fromStdString(module->metadata().id), module->contributeSoundCues());
        CHECK(problems.join(QLatin1Char('\n')).toStdString().empty());
        for (const sdk::SoundCueInfo& cue : module->contributeSoundCues())
            CHECK(catalog.resolve(QString::fromStdString(cue.id)).has_value());
    }
    // REBUILT's alliance-shift sound is the plugin's own file, not one of the app's.
    const sdk::IGameModule* rebuilt = env.services->plugins().findByYear(2026);
    REQUIRE(rebuilt != nullptr);
    wpiemu::app::runview::SfxCatalog catalog;
    (void)catalog.set(QStringLiteral("game_2026"), rebuilt->contributeSoundCues());
    CHECK(catalog.resolve(QStringLiteral("game_2026.shift_change")).value().toString() ==
          QLatin1String("qrc:/wpiemu/plugins/game_2026/sfx/shift_change.wav"));
}
#endif

TEST_CASE("a robot built by the wizard passes core validation apart from device mappings") {
    TestEnvironment env;
    AppServices& s = *env.services;
    for (const char* kind : {sdk::drivetrain_kind::kSwerve, sdk::drivetrain_kind::kTank, sdk::drivetrain_kind::kMecanum}) {
        CAPTURE(kind);
        ProjectInspection in;
        in.path = QStringLiteral("/p");
        in.wpilibYear = 2026;
        in.language = QStringLiteral("java");
        const sdk::RobotConfig config = defaultRobotConfig(s, in, kind);
        // the game plugin's mechanisms must be registered before validating against them
        if (auto* module = s.plugins().findByYear(2026)) CHECK(s.pluginRegistrar().ensureRegistered(*module).ok());
        const auto issues = s.validator().validate(config, s.mechanisms(), s.drivetrains(), s.plugins());
        for (const auto& i : issues) {
            CAPTURE(i.code);
            CAPTURE(i.message);
            if (i.severity == sdk::Severity::Error) CHECK(i.code == wpiemu::core::config::issue_code::kSlotRequiredUnmapped);
        }
    }
}

TEST_CASE("build context follows the inspected project and the settings") {
    TestEnvironment env;
    AppServices& s = *env.services;
    AppSettingsData data = s.settings().data();
    data.physicsRateHz = 500;
    s.settings().update(data);
    ProjectInspection in;
    in.path = QStringLiteral("/p");
    in.wpilibYear = 2026;
    in.language = QStringLiteral("cpp");
    in.gradleRioVersion = QStringLiteral("2026.1.1");
    const RobotBuildContext ctx = s.buildContext(in);
    CHECK(ctx.stepUs == 2000);
    CHECK(ctx.language == QStringLiteral("cpp"));
    CHECK(ctx.gameModule != nullptr);
    in.wpilibYear = 2040;
    CHECK(s.buildContext(in).gameModule == nullptr);
}
