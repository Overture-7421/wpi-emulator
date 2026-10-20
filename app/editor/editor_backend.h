#pragma once

// The narrow, Qt-based seam between the robot editor and the rest of the application.
//
// The editor (EditorPage) NEVER reaches into core, ipc or the plugins: everything it needs is
// pulled through this interface and everything it produces is pushed through it. The shell
// supplies the real adapter (config store + validator + device hub + registries); tests and the
// demo use FakeEditorBackend. All members are called on the UI thread.
//
// Data flow
//   backend -> editor : catalog() (what can be built), config() (what exists), devices() and the
//                       discovery signals (what the robot code declared), issues (runtime warnings)
//   editor  -> backend: submitConfig() on every edit (live), saveConfig() for autosave / Ctrl+S,
//                       startDetection() / stopDetection() for the "Detect devices" button

#include <QMetaType>
#include <QObject>
#include <QString>

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/discovered_device.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_value.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::editor {

struct DrivetrainOption {
    std::string kind;              // DrivetrainSpec::kind ("swerve", "tank", "mecanum", ...)
    sdk::DrivetrainInfo info;
};

// Everything the editor may offer to build, gathered from the registries by the backend: the
// mechanism palette, the drivetrain kinds and the SELECTED game plugin's config schema and panels.
// The editor renders all of it generically; it never knows a concrete template or season.
struct EditorCatalog {
    std::vector<sdk::MechanismTemplateInfo> mechanismTemplates;   // core + plugin templates, registration order
    std::vector<DrivetrainOption> drivetrains;
    std::string pluginId;                                          // selected game plugin; empty = none ("unsupported year")
    std::string pluginDisplayName;
    sdk::PluginConfigSchema pluginSchema;                          // edited under RobotConfig::plugins[pluginId]
    std::vector<sdk::EditorPanelDescriptor> pluginPanels;
    std::vector<sdk::ZoneKindInfo> zoneKinds;                      // zone kinds the selected plugin owns (pickup, muzzle, ...)

    std::vector<sdk::GameValueInfo> gameValues;                    // values the selected plugin publishes (sensor sources)
    sdk::FieldFrame fieldFrame;                                    // symmetry of the season's field (start position copy tool)

    const sdk::GameValueInfo* findGameValue(std::string_view id) const {
        for (const auto& v : gameValues)
            if (v.id == id) return &v;
        return nullptr;
    }
    const sdk::MechanismTemplateInfo* findTemplate(std::string_view type) const {
        for (const auto& t : mechanismTemplates)
            if (t.type == type) return &t;
        return nullptr;
    }
    const sdk::DrivetrainInfo* findDrivetrain(std::string_view kind) const {
        for (const auto& d : drivetrains)
            if (d.kind == kind) return &d.info;
        return nullptr;
    }
    const sdk::ZoneKindInfo* findZoneKind(std::string_view kind) const {
        for (const auto& k : zoneKinds)
            if (k.kind == kind) return &k;
        return nullptr;
    }
    bool hasPlugin() const { return !pluginId.empty(); }
};

class IEditorBackend : public QObject {
    Q_OBJECT
public:
    explicit IEditorBackend(QObject* parent = nullptr) : QObject(parent) {}
    ~IEditorBackend() override = default;

    // ---- queries -------------------------------------------------------------------------------
    virtual sdk::RobotConfig config() const = 0;                       // the persisted model to edit
    virtual EditorCatalog catalog() const = 0;
    virtual std::vector<sdk::DiscoveredDevice> devices() const = 0;    // everything announced so far
    // Whole-model validation (IConfigValidator). Cheap; called after edits. Runtime warnings that
    // need the hub arrive through issuesChanged() instead.
    virtual std::vector<sdk::Issue> validate(const sdk::RobotConfig& config) const = 0;
    virtual bool detecting() const = 0;

    // ---- commands ------------------------------------------------------------------------------
    // Live-applies the edited model (the backend may re-bind slots in a running session). Does not
    // have to persist.
    virtual void submitConfig(const sdk::RobotConfig& config) = 0;
    // Persists the last submitted model (app-data copy + <project>/.wpiemu/robot.json).
    // Success means the authoritative copy was saved. A failed project export is a warning only.
    virtual sdk::Status saveConfig() = 0;
    // "Detect devices": run the robot in discovery mode; devices then arrive through the signals.
    virtual void startDetection() = 0;
    virtual void stopDetection() = 0;

Q_SIGNALS:
    // The backend replaced the model (project switched, file reloaded): the editor reloads
    // config() and clears its undo history.
    void configReplaced();
    // Plugin selection / registries changed: the editor re-reads catalog().
    void catalogChanged();
    // Bulk refresh of devices() (e.g. a new run started).
    void devicesChanged();
    // Live discovery: first announce of a device, and later changes (alias state, liveness...).
    void deviceDiscovered(wpiemu::sdk::DiscoveredDevice device);
    void deviceUpdated(wpiemu::sdk::DiscoveredDevice device);
    // Asynchronous / runtime issues (hub warnings, validator results computed off-thread). They
    // are merged with validate() results and the editor's own slot checks. Replace-all semantics.
    void issuesChanged(std::vector<wpiemu::sdk::Issue> issues);
    void detectionStateChanged(bool running);
    // Console line for the bottom panel (robot stdout, gradle output, discovery progress).
    void logMessage(wpiemu::sdk::Severity severity, QString text);
    // Optional live telemetry for the Telemetry tab while a robot is connected in discovery mode.
    void snapshotChanged(std::shared_ptr<const wpiemu::sdk::ISimSnapshot> snapshot);
};

}  // namespace wpiemu::app::editor

Q_DECLARE_METATYPE(wpiemu::sdk::DiscoveredDevice)
Q_DECLARE_METATYPE(std::vector<wpiemu::sdk::Issue>)
Q_DECLARE_METATYPE(wpiemu::sdk::Severity)
Q_DECLARE_METATYPE(std::shared_ptr<const wpiemu::sdk::ISimSnapshot>)
