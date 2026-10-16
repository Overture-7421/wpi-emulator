#include "shell/editor_backend_adapter.h"

#include <algorithm>

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
    if (const sdk::IGameModule* module = services_.session().gameModule()) {
        out.pluginId = module->metadata().id;
        out.pluginDisplayName = module->metadata().displayName;
        out.pluginSchema = module->contributeConfigSchema();
        out.pluginPanels = module->contributeEditorPanels();
    }
    return out;
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
    services_.runtime().setProject(services_.session().path(), std::make_shared<const sdk::RobotConfig>(config),
                                   services_.session().gameModule());
}

void EditorBackendAdapter::saveConfig() {
    ProjectSession& session = services_.session();
    if (!session.hasProject() || !pending_) return;
    committing_ = true;
    const SaveOutcome outcome = session.commit(*pending_, session.prefs());
    committing_ = false;
    if (!outcome.ok()) {
        emit logMessage(sdk::Severity::Error, tr("Could not save the robot configuration: %1").arg(outcome.appDataError));
        return;
    }
    pending_.reset();   // what is stored is now what the editor shows
    if (outcome.hasWarning())
        emit logMessage(sdk::Severity::Warning, tr("Saved, but the project copy %1 could not be written: %2").arg(outcome.exportPath, outcome.exportError));
    else
        emit logMessage(sdk::Severity::Info, tr("Saved robot configuration to %1").arg(outcome.exportPath));
}

void EditorBackendAdapter::startDetection() { services_.runtime().restart(); }

void EditorBackendAdapter::stopDetection() { services_.runtime().stop(); }

void EditorBackendAdapter::onSessionChanged() {
    if (committing_) return;
    const QString project = services_.session().path();
    if (project != lastProject_) {
        lastProject_ = project;
        devices_.clear();
        deviceIndex_.clear();
        lastIssues_.clear();
        emit devicesChanged();
        emit issuesChanged({});
    }
    pending_.reset();
    emit catalogChanged();
    emit configReplaced();
}

bool EditorBackendAdapter::sameDevice(const sdk::DiscoveredDevice& a, const sdk::DiscoveredDevice& b) {
    return a.live == b.live && a.mapped == b.mapped && a.aliasState == b.aliasState && a.observedBus == b.observedBus &&
           a.aliases == b.aliases && a.descriptor == b.descriptor;
}

void EditorBackendAdapter::onSnapshot() {
    const auto snapshot = services_.runtime().latestSnapshot();
    if (!snapshot) return;

    for (const sdk::DiscoveredDevice& d : snapshot->devices()) {
        const auto it = deviceIndex_.constFind(d.id.value);
        if (it == deviceIndex_.constEnd()) {
            deviceIndex_.insert(d.id.value, static_cast<int>(devices_.size()));
            devices_.push_back(d);
            emit deviceDiscovered(d);
        } else if (!sameDevice(devices_[static_cast<std::size_t>(*it)], d)) {
            devices_[static_cast<std::size_t>(*it)] = d;
            emit deviceUpdated(d);
        }
    }

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
