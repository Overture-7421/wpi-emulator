#pragma once

// A complete in-memory IEditorBackend for tests and the standalone demo: a realistic catalog (the
// core "mechanism" template, two plain sample templates, the game plugin's zone kinds, a plugin config
// schema and panels), a swerve sample robot with two zones, a device list and a scripted "Detect devices" run that
// announces devices over time. Validation uses the real SDK helpers (property schemas, attachment
// and alias contracts), so the editor behaves as it will against the real validator.

#include <QTimer>

#include <vector>

#include "editor_backend.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::app::editor {

class FakeEditorBackend : public IEditorBackend {
    Q_OBJECT
public:
    explicit FakeEditorBackend(QObject* parent = nullptr);

    // ---- IEditorBackend ----
    sdk::RobotConfig config() const override { return config_; }
    EditorCatalog catalog() const override { return catalog_; }
    std::vector<sdk::DiscoveredDevice> devices() const override { return devices_; }
    std::vector<sdk::Issue> validate(const sdk::RobotConfig& config) const override;
    bool detecting() const override { return detecting_; }
    void submitConfig(const sdk::RobotConfig& config) override;
    sdk::Status saveConfig() override;
    void setSaveError(QString error) { saveError_ = std::move(error); }
    void startDetection() override;
    void stopDetection() override;

    // ---- scripting hooks for tests / the demo ----
    void setConfig(sdk::RobotConfig config);             // emits configReplaced()
    void setCatalog(EditorCatalog catalog);              // emits catalogChanged()
    void setDevices(std::vector<sdk::DiscoveredDevice> devices);   // emits devicesChanged()
    void announce(const sdk::DiscoveredDevice& device);  // emits deviceDiscovered()/deviceUpdated()
    void setDetectionStepMs(int ms) { stepMs_ = ms; }
    void log(sdk::Severity severity, const QString& text) { Q_EMIT logMessage(severity, text); }
    void pushIssues(std::vector<sdk::Issue> issues) { Q_EMIT issuesChanged(std::move(issues)); }

    int submitCount() const { return submitCount_; }
    int saveCount() const { return saveCount_; }
    const sdk::RobotConfig& lastSubmitted() const { return config_; }

    // Building blocks (also useful for tests that need a catalog or robot without the backend).
    static EditorCatalog sampleCatalog();
    static sdk::RobotConfig sampleConfig();
    static std::vector<sdk::DiscoveredDevice> sampleDevices();
    static sdk::DiscoveredDevice makeDevice(const std::string& family, const std::string& type, int id, const std::string& bus,
                                            sdk::CapabilitySet caps, const std::string& vendor = "ctre");

private:
    void detectionStep();
    std::vector<sdk::Issue> validateZones(const sdk::RobotConfig& config) const;
    std::vector<sdk::Issue> validateCameras(const sdk::RobotConfig& config) const;
    std::vector<sdk::Issue> validateSensors(const sdk::RobotConfig& config) const;

    sdk::RobotConfig config_;
    EditorCatalog catalog_;
    std::vector<sdk::DiscoveredDevice> devices_;
    std::vector<sdk::DiscoveredDevice> script_;   // devices announced during a detection run
    sdk::MechanismFactory factory_;               // templates only (for the SDK attachment validator)
    QTimer timer_;
    std::size_t scriptIndex_ = 0;
    bool detecting_ = false;
    int stepMs_ = 700;
    int submitCount_ = 0;
    int saveCount_ = 0;
    QString saveError_;
};

}  // namespace wpiemu::app::editor
