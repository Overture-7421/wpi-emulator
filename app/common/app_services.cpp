#include "common/app_services.h"

#include <QDir>

#include <thread>

#include "common/demo_runtime_host.h"
#include "common/real_runtime_host.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/sdk/game/builtin_plugins.h"

namespace wpiemu::app {


namespace {

std::filesystem::path defaultAppDataRoot() {
    using namespace core::config;
    if (auto root = appDataRoot(currentPlatform(), &sdk::readEnvironment); root) return root.value();
    return std::filesystem::path(QDir::homePath().toStdU16String()) / ".wpiemu";
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
    note("drivetrains", core::physics::registerPhysicsComponents(drivetrains_, tireModels_));
    note("mechanisms", core::mechanisms::registerGenericMechanisms(mechanisms_));

    store_ = options.store ? std::move(options.store) : std::make_unique<QSettingsKeyValueStore>();
    settings_ = std::make_unique<AppSettings>(*store_);
    recent_ = std::make_unique<RecentProjects>(*store_);
    prefs_ = std::make_unique<ProjectPrefsStore>(*store_);
    probe_ = options.probe ? std::move(options.probe) : std::make_unique<QtSystemProbe>();
    inspector_ = options.inspector ? std::move(options.inspector) : std::make_unique<GradleProjectInspector>();
    toolchain_ = std::make_unique<ToolchainChecker>(*probe_, plugins_);

    const std::filesystem::path root = options.appDataRoot.empty() ? defaultAppDataRoot() : options.appDataRoot;
    configs_ = std::make_unique<ConfigRepository>(root, textCodec_, configCodec_, pipeline_);
    session_ = std::make_unique<ProjectSession>(*inspector_, *configs_, *recent_, *prefs_, plugins_);

    if (options.runtime) {
        runtime_ = std::move(options.runtime);
    } else if (options.demo) {
        runtime_ = std::make_unique<DemoRuntimeHost>();
    } else {
        host::Registries registries;
        registries.plugins = &plugins_;
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
            return RealRuntimeOptions::ToolchainOverrides{folder(settings_->data().jdkHome), folder(settings_->data().wpilibHome)};
        };
        real.physicsWorkers = [this] {
            return core::physics::resolveWorkerCount(settings_->data().physicsThreads, std::thread::hardware_concurrency());
        };
        runtime_ = std::make_unique<RealRuntimeHost>(registries, std::move(real));
    }
}

AppServices::~AppServices() = default;

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
    ctx.projectPath = inspection.path;
    ctx.language = inspection.language;
    ctx.wpilibYear = inspection.wpilibYear;
    ctx.gradleRioVersion = inspection.gradleRioVersion;
    ctx.stepUs = settings_->stepUs();
    ctx.gameModule = inspection.wpilibYear > 0 ? plugins_.findByYear(inspection.wpilibYear) : nullptr;
    return ctx;
}

}  // namespace wpiemu::app
