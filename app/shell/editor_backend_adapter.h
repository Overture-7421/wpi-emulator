#pragma once

#include <optional>
#include <vector>

#include "common/app_services.h"
#include "editor/editor_backend.h"

namespace wpiemu::app {

// Connects the robot editor to the application: config load/save through the project session,
// validation through core's validator, the catalog from the registries and the selected game
// plugin, and device discovery through the shared runtime host.
class EditorBackendAdapter final : public editor::IEditorBackend {
    Q_OBJECT
public:
    explicit EditorBackendAdapter(AppServices& services, QObject* parent = nullptr);

    sdk::RobotConfig config() const override;
    editor::EditorCatalog catalog() const override;
    std::vector<sdk::DiscoveredDevice> devices() const override { return devices_; }
    std::vector<sdk::Issue> validate(const sdk::RobotConfig& config) const override;
    bool detecting() const override;

    void submitConfig(const sdk::RobotConfig& config) override;
    sdk::Status saveConfig() override;
    void startDetection() override;
    void stopDetection() override;

private:
    void onSessionChanged();
    void onSnapshot();
    void onStatus();
    void onLog(const HostLogLine& line);
    static bool sameDevice(const sdk::DiscoveredDevice& a, const sdk::DiscoveredDevice& b);

    AppServices& services_;
    std::optional<sdk::RobotConfig> pending_;   // last config submitted by the editor (unsaved edits)
    bool committing_ = false;
    bool lastDetecting_ = false;
    std::uint64_t lastRunGeneration_ = 0;
    QString lastProject_;
    QString pendingProject_;
    std::vector<sdk::DiscoveredDevice> devices_;
    std::vector<sdk::Issue> lastIssues_;
};

}  // namespace wpiemu::app
