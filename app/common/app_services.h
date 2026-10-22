#pragma once

#include <filesystem>
#include <memory>

#include "common/app_settings.h"
#include "common/build_tools_installer.h"
#include "common/config_repository.h"
#include "common/key_value_store.h"
#include "common/project_inspector.h"
#include "common/project_prefs.h"
#include "common/project_session.h"
#include "common/recent_projects.h"
#include "common/system_probe.h"
#include "common/toolchain_checker.h"
#include "common/toolchain_provisioner.h"
#include "common/update_service.h"
#include "contract/runtime_host.h"
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
    std::unique_ptr<IProjectRuntimeHost> runtime;    // default: RealRuntimeHost, or DemoRuntimeHost when `demo`
    std::unique_ptr<IUpdateTransport> updateTransport;   // default: QNetworkAccessManager against github.com
    std::unique_ptr<IUpdateInstaller> updateInstaller;   // default: runs the installer through the OS
    std::unique_ptr<ToolchainProvisioner> provisioner;   // default: downloads the pinned Java through github.com
    std::unique_ptr<IBuildToolsEffects> buildToolsEffects;   // default: winget, else the Build Tools download page
    std::filesystem::path localDataRoot;             // default: core's per-machine data folder (the downloaded Java lives here)
    QString appDir;                                  // default: the running executable's folder
    QString appVersion;                              // default: the version this binary was built as
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
    UpdateService& updates() { return *updates_; }
    ToolchainProvisioner& provisioner() { return *provisioner_; }
    BuildToolsInstaller& buildTools() { return *buildTools_; }
    // What the Toolchain checks need to know about `inspection`'s project on this computer (Settings, the emulator's own Java).
    ToolchainInput toolchainInput(const ProjectInspection& inspection) const;
    // Tells the runtime which project is open: `config` (the saved model, or one still being edited), the project's game
    // module and the wizard's Driver Station choice. Call whenever the open project or its model changes.
    void bindRuntime(std::shared_ptr<const sdk::RobotConfig> config);

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
    std::unique_ptr<IProjectRuntimeHost> runtime_;
    std::unique_ptr<UpdateService> updates_;
    std::unique_ptr<ToolchainProvisioner> provisioner_;
    std::unique_ptr<BuildToolsInstaller> buildTools_;
};

}  // namespace wpiemu::app
