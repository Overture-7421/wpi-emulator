#include "common/app_services.h"

#include <QDir>
#include <QCoreApplication>

#include <thread>

#include "common/demo_runtime_host.h"
#include "common/real_runtime_host.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/jdk_policy.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/season/builtin_seasons.h"

namespace wpiemu::app {


namespace {

std::filesystem::path defaultAppDataRoot() {
    using namespace core::config;
    if (auto root = appDataRoot(currentPlatform(), &sdk::readEnvironment); root) return root.value();
    return std::filesystem::path(QDir::homePath().toStdU16String()) / ".wpiemu";
}

std::filesystem::path defaultLocalDataRoot(const std::filesystem::path& fallback) {
    using namespace core::config;
    if (auto root = localDataRoot(currentPlatform(), &sdk::readEnvironment); root) return root.value();
    return fallback;
}

}  // namespace

AppServices::AppServices(AppServicesOptions options)
    : demo_(options.demo),
      registrar_(mechanisms_),
      validator_(&tireModels_, {}),
      pipeline_(configCodec_, validator_, registrar_, drivetrains_, plugins_) {
    // Built-in year plugins are registered through the aggregation seam only.
    const auto note = [this](const char* what, const sdk::Status& status) {
        if (!status) startupProblems_.push_back(QStringLiteral("%1: %2").arg(QLatin1String(what), QString::fromStdString(status.error().message)));
    };
    note("game plugins", sdk::registerBuiltinGamePlugins(plugins_));
    note("seasons", sdk::registerBuiltinSeasons(seasons_));
    note("drivetrains", core::physics::registerPhysicsComponents(drivetrains_, tireModels_));
    note("mechanisms", core::mechanisms::registerGenericMechanisms(mechanisms_));

    store_ = options.store ? std::move(options.store) : std::make_unique<QSettingsKeyValueStore>();
    settings_ = std::make_unique<AppSettings>(*store_);
    recent_ = std::make_unique<RecentProjects>(*store_);
    prefs_ = std::make_unique<ProjectPrefsStore>(*store_);
    probe_ = options.probe ? std::move(options.probe) : std::make_unique<QtSystemProbe>();
    inspector_ = options.inspector ? std::move(options.inspector) : std::make_unique<GradleProjectInspector>();
    toolchain_ = std::make_unique<ToolchainChecker>(*probe_, plugins_, seasons_);

    const std::filesystem::path root = options.appDataRoot.empty() ? defaultAppDataRoot() : options.appDataRoot;
    const std::filesystem::path localRoot = !options.localDataRoot.empty() ? options.localDataRoot
                                            : !options.appDataRoot.empty() ? options.appDataRoot   // tests keep everything in one folder
                                                                           : defaultLocalDataRoot(root);
    const QString appDir = options.appDir.isEmpty() && QCoreApplication::instance() ? QCoreApplication::applicationDirPath() : options.appDir;
    const QString appVersion = options.appVersion.isEmpty() ? QStringLiteral(WPIEMU_APP_VERSION) : options.appVersion;
    if (options.provisioner) {
        provisioner_ = std::move(options.provisioner);
    } else {
        provisioner_ = std::make_unique<ToolchainProvisioner>(
            *probe_, createQtUpdateTransport(appVersion), createQtToolchainEffects(),
            QString::fromStdU16String(core::launcher::managedToolchainsDir(localRoot).generic_u16string()), appDir,
            ToolchainProvisioner::pinnedAssetsFor(probe_->hostOs(), probe_->cpuArchitecture()));
    }
    configs_ = std::make_unique<ConfigRepository>(root, textCodec_, configCodec_, pipeline_);
    buildTools_ = std::make_unique<BuildToolsInstaller>(
        *probe_, options.buildToolsEffects ? std::move(options.buildToolsEffects) : createQtBuildToolsEffects());
    session_ = std::make_unique<ProjectSession>(*inspector_, *configs_, *recent_, *prefs_, plugins_);

    if (options.runtime) {
        runtime_ = std::move(options.runtime);
    } else if (options.demo) {
        runtime_ = std::make_unique<DemoRuntimeHost>();
    } else {
        host::Registries registries;
        registries.plugins = &plugins_;
        registries.seasons = &seasons_;
        registries.mechanisms = &mechanisms_;
        registries.drivetrains = &drivetrains_;
        registries.tireModels = &tireModels_;
        registries.registrar = &registrar_;
        registries.pipeline = &pipeline_;
        registries.json = &textCodec_;
        RealRuntimeOptions real;
        real.scratchDir = root / "scratch";
        real.toolchainOverrides = [this] {
            const auto folder = [](const QString& text) { return text.isEmpty() ? std::filesystem::path() : std::filesystem::path(text.toStdU16String()); };
            RealRuntimeOptions::ToolchainOverrides overrides;
            overrides.wpilibHome = folder(settings_->data().wpilibHome);
            // The Java the Toolchain page validated is the Java the launch uses.
            const ToolchainInput input = toolchainInput(session_->inspection());
            const JdkResolution jdk = toolchain_->resolveJdk(input);
            if (jdk.found) {
                overrides.javaHome = folder(jdk.home);
            } else {
                QStringList reasons;
                for (const JdkSkip& skip : jdk.skipped) reasons << skip.reason;
                overrides.blocked = QCoreApplication::translate("AppServices", "No Java that can run this project was found.");
                if (!reasons.isEmpty()) overrides.blocked += QLatin1Char('\n') + reasons.join(QLatin1Char('\n'));
                overrides.problem = jdk.explicitFailed ? QStringLiteral("java_setting_unusable") : QStringLiteral("no_usable_java");
            }
            return overrides;
        };
        real.physicsWorkers = [this] {
            return core::physics::resolveWorkerCount(settings_->data().physicsThreads, std::thread::hardware_concurrency());
        };
        runtime_ = std::make_unique<RealRuntimeHost>(registries, std::move(real));
    }

    updates_ = std::make_unique<UpdateService>(
        *store_, *probe_, options.updateTransport ? std::move(options.updateTransport) : createQtUpdateTransport(appVersion),
        options.updateInstaller ? std::move(options.updateInstaller) : createQtUpdateInstaller(), appVersion, appDir, [this] {
            std::vector<int> years;
            for (const sdk::SeasonProfile& season : seasons_.list()) years.push_back(season.year);
            return years;
        }());
}

AppServices::~AppServices() = default;

ToolchainInput AppServices::toolchainInput(const ProjectInspection& inspection) const {
    ToolchainInput in;
    in.projectPath = inspection.path;
    in.language = inspection.language;
    in.wpilibYear = inspection.wpilibYear;
    in.wpilibVersion = inspection.gradleRioVersion;
    in.gradleVersion = inspection.gradleVersion;
    // An unknown project year tries the default season's extension, as the launch does.
    const sdk::SeasonProfile* fallback = seasons_.defaultSeason();
    const int extensionYear = inspection.wpilibYear > 0 ? inspection.wpilibYear : (fallback != nullptr ? fallback->year : 0);
    in.extensionLibrary = QString::fromStdU16String(RealRuntimeHost::installedExtension(extensionYear).u16string());
    for (const sdk::SeasonProfile& season : seasons_.list())
        if (!RealRuntimeHost::installedExtension(season.year).empty()) in.installedExtensionSeasons << QString::number(season.year);
    in.wpilibHomeOverride = settings_->data().wpilibHome;
    in.jdkHomeOverride = settings_->data().jdkHome;
    in.buildToolsInstallAvailable = buildTools_->available();
    // The Java comes from the season: an unknown season has no download and keeps the default minimum.
    if (const sdk::SeasonProfile* season = seasons_.find(inspection.wpilibYear); season != nullptr && season->javaMajor > 0) {
        in.minJavaMajor = season->javaMajor;
        in.managedJdkHome = provisioner_->installedHome(season->javaMajor);
        in.managedJavaAvailable = provisioner_->availableFor(season->javaMajor);
    }
    return in;
}

void AppServices::bindRuntime(std::shared_ptr<const sdk::RobotConfig> config) {
    HostProject project;
    project.path = session_->path();
    project.config = std::move(config);
    project.game = session_->gameModule();
    project.defaultStation = sdk::proto::stationOf(session_->prefs().alliance, session_->prefs().station);
    runtime_->setProject(project);
}

RobotBuildContext AppServices::buildContext(const ProjectInspection& inspection) const {
    RobotBuildContext ctx;
    ctx.language = inspection.language;
    ctx.wpilibYear = inspection.wpilibYear;
    ctx.gradleRioVersion = inspection.gradleRioVersion;
    ctx.stepUs = settings_->stepUs();
    ctx.gameModule = inspection.wpilibYear > 0 ? plugins_.findByYear(inspection.wpilibYear) : nullptr;
    return ctx;
}

}  // namespace wpiemu::app
