#include "shell/editor_backend_adapter.h"

#include <algorithm>

#include "host/robot_session.h"

namespace wpiemu::app {

namespace {

sdk::Severity severityFor(sdk::LogLevel level) {
    switch (level) {
        case sdk::LogLevel::Warn: return sdk::Severity::Warning;
        case sdk::LogLevel::Error: return sdk::Severity::Error;
        case sdk::LogLevel::Trace:
        case sdk::LogLevel::Debug:
        case sdk::LogLevel::Info: break;
    }
    return sdk::Severity::Info;
}

}  // namespace

EditorBackendAdapter::EditorBackendAdapter(AppServices& services, QObject* parent)
    : editor::IEditorBackend(parent), services_(services) {
    connect(&services_.session(), &ProjectSession::configChanged, this, &EditorBackendAdapter::onSessionChanged);
    connect(&services_.session(), &ProjectSession::projectChanged, this, &EditorBackendAdapter::onSessionChanged);
    connect(&services_.runtime(), &IRuntimeHost::snapshotPublished, this, &EditorBackendAdapter::onSnapshot);
    connect(&services_.runtime(), &IRuntimeHost::statusChanged, this, &EditorBackendAdapter::onStatus);
    connect(&services_.runtime(), &IRuntimeHost::logLine, this, &EditorBackendAdapter::onLog);
    lastProject_ = services_.session().path();
}

sdk::RobotConfig EditorBackendAdapter::config() const {
    if (pending_) return *pending_;
    if (const auto stored = services_.session().config()) return *stored;
    return {};
}

editor::EditorCatalog EditorBackendAdapter::catalog() const {
    editor::EditorCatalog out;
    services_.mechanisms().forEach([&out](const std::string&, const sdk::MechanismTemplateInfo& info) { out.mechanismTemplates.push_back(info); });
    services_.drivetrains().forEach([&out](const std::string& kind, const sdk::DrivetrainInfo& info) { out.drivetrains.push_back({kind, info}); });
    const sdk::IGameModule* module = services_.session().gameModule();
    if (module != nullptr) {
        out.pluginId = module->metadata().id;
        out.pluginDisplayName = module->metadata().displayName;
        out.pluginSchema = module->contributeConfigSchema();
        out.pluginPanels = module->contributeEditorPanels();
        out.zoneKinds = module->contributeZoneKinds();
        out.fiducialLayouts = module->contributeFiducialLayouts();
    }
    out.defaultStartPose = defaultStartPose(module);
    return out;
}

sdk::StartPose EditorBackendAdapter::defaultStartPose(const sdk::IGameModule* module) const {
    // The same spot RobotSession puts the robot when the config sets no start pose: inside the plugin's field, else the origin.
    const std::string id = module != nullptr ? module->metadata().id : std::string();
    if (id != fieldPluginId_ || (module != nullptr && field_ == nullptr)) {
        field_ = module != nullptr ? host::describeField(services_.plugins().find(id)) : nullptr;
        fieldPluginId_ = id;
    }
    const host::StartPlacement start = host::startPlacementFor(field_.get());
    return sdk::StartPose{start.xM, start.yM, 0.f};
}

std::vector<sdk::Issue> EditorBackendAdapter::validate(const sdk::RobotConfig& config) const {
    return services_.validator().validate(config, services_.mechanisms(), services_.drivetrains(), services_.plugins());
}

bool EditorBackendAdapter::detecting() const {
    const HostStatus s = services_.runtime().status();
    return s.process == HostProcessState::Starting || (s.process == HostProcessState::Running && !s.programStarted);
}

void EditorBackendAdapter::submitConfig(const sdk::RobotConfig& config) {
    pending_ = config;
    pendingProject_ = services_.session().path();
    services_.runtime().setProject(services_.session().path(), std::make_shared<const sdk::RobotConfig>(config),
                                   services_.session().gameModule());
}

sdk::Status EditorBackendAdapter::saveConfig() {
    ProjectSession& session = services_.session();
    if (!pending_) return sdk::okStatus();
    if (!session.hasProject() || session.path() != pendingProject_)
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "The edited project is no longer open.");
    committing_ = true;
    const SaveOutcome outcome = session.commit(*pending_, session.prefs());
    committing_ = false;
    if (!outcome.ok()) {
        emit logMessage(sdk::Severity::Error, tr("Could not save the robot configuration: %1").arg(outcome.appDataError));
        return sdk::makeError(sdk::ErrorCode::IoError, outcome.appDataError.toStdString());
    }
    pending_.reset();   // what is stored is now what the editor shows
    if (outcome.hasWarning())
        emit logMessage(sdk::Severity::Warning, tr("Saved, but the project copy %1 could not be written: %2").arg(outcome.exportPath, outcome.exportError));
    else
        emit logMessage(sdk::Severity::Info, tr("Saved robot configuration to %1").arg(outcome.exportPath));
    return sdk::okStatus();
}

void EditorBackendAdapter::startDetection() { services_.runtime().restart(); }

void EditorBackendAdapter::stopDetection() { services_.runtime().stop(); }

void EditorBackendAdapter::onSessionChanged() {
    if (committing_) return;
    const QString project = services_.session().path();
    if (project != lastProject_) {
        lastProject_ = project;
        devices_.clear();
        lastIssues_.clear();
        emit devicesChanged();
        emit issuesChanged({});
        if (!cameras_.empty()) {
            cameras_.clear();
            emit detectedCamerasChanged();
        }
    }
    pending_.reset();
    emit catalogChanged();
    emit configReplaced();
}

bool EditorBackendAdapter::sameDevice(const sdk::DiscoveredDevice& a, const sdk::DiscoveredDevice& b) {
    return a.id == b.id && a.live == b.live && a.mapped == b.mapped && a.aliasState == b.aliasState && a.observedBus == b.observedBus &&
           a.aliases == b.aliases && a.descriptor == b.descriptor;
}

void EditorBackendAdapter::onSnapshot() {
    const auto snapshot = services_.runtime().latestSnapshot();
    if (!snapshot) return;

    auto reconciled = devices_;
    for (auto& d : reconciled) {
        d.live = false;
        d.mapped = false;
    }
    std::vector<sdk::DiscoveredDevice> newDevices;
    for (const sdk::DiscoveredDevice& d : snapshot->devices()) {
        const auto it = std::find_if(reconciled.begin(), reconciled.end(), [&d](const auto& known) {
            return known.descriptor.physicalId() == d.descriptor.physicalId();
        });
        if (it == reconciled.end()) {
            reconciled.push_back(d);
            newDevices.push_back(d);
        } else {
            *it = d;
        }
    }
    if (devices_.size() != reconciled.size() ||
        !std::equal(devices_.begin(), devices_.end(), reconciled.begin(), sameDevice)) {
        devices_ = std::move(reconciled);
        for (const auto& d : newDevices) emit deviceDiscovered(d);
        emit devicesChanged();
    }

    // The cameras the robot process reports (including ones the model does not list). Like devices, a camera that is no
    // longer reported stays until the next robot launch; entries are told apart by vendor and NetworkTables name.
    bool camerasChanged = false;
    for (const sdk::CameraRobotState& seen : snapshot->detectedCameras()) {
        const auto it = std::find_if(cameras_.begin(), cameras_.end(), [&seen](const sdk::CameraRobotState& known) {
            return known.vendor == seen.vendor && known.ntName == seen.ntName;
        });
        if (it == cameras_.end()) {
            cameras_.push_back(seen);
            camerasChanged = true;
            continue;
        }
        sdk::CameraRobotState previous = *it;
        previous.framesPublished = seen.framesPublished;   // a counter that moves on every frame is not news for the editor
        camerasChanged = camerasChanged || !(previous == seen);
        *it = seen;
    }
    if (camerasChanged) emit detectedCamerasChanged();

    std::vector<sdk::Issue> issues;
    for (const sdk::HubWarning& w : snapshot->warnings()) {
        sdk::Issue issue;
        issue.severity = w.severity;
        issue.path = w.device.valid() ? "devices[" + std::to_string(w.device.value) + "]" : std::string();
        issue.message = w.message;
        issue.code = w.code;
        issues.push_back(std::move(issue));
    }
    const auto same = [](const sdk::Issue& a, const sdk::Issue& b) { return a.code == b.code && a.path == b.path && a.message == b.message && a.severity == b.severity; };
    if (issues.size() != lastIssues_.size() || !std::equal(issues.begin(), issues.end(), lastIssues_.begin(), same)) {
        lastIssues_ = issues;
        emit issuesChanged(std::move(issues));
    }
    emit snapshotChanged(snapshot);
}

void EditorBackendAdapter::onStatus() {
    const auto generation = services_.runtime().status().runGeneration;
    if (generation != lastRunGeneration_) {
        lastRunGeneration_ = generation;
        for (auto& d : devices_) {
            d.live = false;
            d.mapped = false;
        }
        lastIssues_.clear();
        emit devicesChanged();
        emit issuesChanged({});
        if (!cameras_.empty()) {
            cameras_.clear();
            emit detectedCamerasChanged();
        }
    }
    const bool now = detecting();
    if (now == lastDetecting_) return;
    lastDetecting_ = now;
    emit detectionStateChanged(now);
}

void EditorBackendAdapter::onLog(const HostLogLine& line) {
    const QString text = line.source.isEmpty() ? line.text : QStringLiteral("[%1] %2").arg(line.source, line.text);
    emit logMessage(severityFor(line.level), text);
}

}  // namespace wpiemu::app
