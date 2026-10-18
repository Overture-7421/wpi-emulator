#pragma once

#include <filesystem>
#include <memory>

#include "common/app_settings.h"
#include "common/config_repository.h"
#include "common/key_value_store.h"
#include "common/project_inspector.h"
#include "common/project_prefs.h"
#include "common/project_session.h"
#include "common/recent_projects.h"
#include "common/runtime_host.h"
#include "common/system_probe.h"
#include "common/toolchain_checker.h"
#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::app {

// Seams the composition root can replace (tests, --demo). Anything left empty gets the real thing.
struct AppServicesOptions {
    std::unique_ptr<IKeyValueStore> store;           // default: native QSettings
    std::filesystem::path appDataRoot;               // default: core's per-user data folder
    std::unique_ptr<ISystemProbe> probe;             // default: QtSystemProbe
    std::unique_ptr<IProjectInspector> inspector;    // default: GradleProjectInspector
    std::unique_ptr<IRuntimeHost> runtime;           // default: RealRuntimeHost, or DemoRuntimeHost when `demo`
    bool demo = false;
};

// The one object that wires the application together. Created once in main(), then handed to the
// shell by reference: there are no singletons, and every collaborator is reachable only through it.
class AppServices {
public:
    explicit AppServices(AppServicesOptions options = {});
    ~AppServices();
    AppServices(const AppServices&) = delete;
    AppServices& operator=(const AppServices&) = delete;

    bool demo() const { return demo_; }
    // Problems found while registering built-in content (shown by the shell; empty in a healthy build).
    const QStringList& startupProblems() const { return startupProblems_; }

    IKeyValueStore& store() { return *store_; }
    AppSettings& settings() { return *settings_; }
    RecentProjects& recentProjects() { return *recent_; }
    ProjectPrefsStore& projectPrefs() { return *prefs_; }
    ProjectSession& session() { return *session_; }
    ConfigRepository& configs() { return *configs_; }
    IProjectInspector& inspector() { return *inspector_; }
    const ISystemProbe& probe() const { return *probe_; }
    const ToolchainChecker& toolchain() const { return *toolchain_; }
    IRuntimeHost& runtime() { return *runtime_; }

    const sdk::GamePluginRegistry& plugins() const { return plugins_; }
    sdk::MechanismFactory& mechanisms() { return mechanisms_; }
    sdk::DrivetrainRegistry& drivetrains() { return drivetrains_; }
    sdk::TireModelRegistry& tireModels() { return tireModels_; }
    core::config::PluginRegistrar& pluginRegistrar() { return registrar_; }
    const core::config::ConfigValidator& validator() const { return validator_; }

    // Converts the persisted model into what the build/run needs from the wizard's context.
    RobotBuildContext buildContext(const ProjectInspection& inspection) const;

private:
    bool demo_ = false;
    QStringList startupProblems_;
    sdk::GamePluginRegistry plugins_;
    sdk::MechanismFactory mechanisms_;
    sdk::DrivetrainRegistry drivetrains_;
    sdk::TireModelRegistry tireModels_;
    core::config::PluginRegistrar registrar_;
    core::config::JsonValueCodec textCodec_;
    core::config::RobotConfigCodec configCodec_;
    core::config::ConfigValidator validator_;
    core::config::ConfigPipeline pipeline_;

    std::unique_ptr<IKeyValueStore> store_;
    std::unique_ptr<AppSettings> settings_;
    std::unique_ptr<RecentProjects> recent_;
    std::unique_ptr<ProjectPrefsStore> prefs_;
    std::unique_ptr<ISystemProbe> probe_;
    std::unique_ptr<IProjectInspector> inspector_;
    std::unique_ptr<ToolchainChecker> toolchain_;
    std::unique_ptr<ConfigRepository> configs_;
    std::unique_ptr<ProjectSession> session_;
    std::unique_ptr<IRuntimeHost> runtime_;
};

}  // namespace wpiemu::app
