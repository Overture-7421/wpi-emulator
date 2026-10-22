#include "wizard/wizard_view_model.h"

#include <QCoreApplication>
#include <QMetaObject>
#include <QThread>

#include <algorithm>

namespace wpiemu::app {

// ---- ToolchainRunner ----------------------------------------------------------------------------------

ToolchainRunner::ToolchainRunner(const ToolchainChecker& checker, QObject* parent) : QObject(parent), checker_(checker) {}

ToolchainRunner::~ToolchainRunner() {
    ++generation_;   // nothing may be delivered to a dying object
    for (QThread* t : threads_) {
        t->wait();
        delete t;
    }
}

void ToolchainRunner::run(const ToolchainInput& input) {
    const quint64 generation = ++generation_;
    running_ = true;
    emit started();

    // Finished threads are reaped lazily so the list stays short.
    for (auto it = threads_.begin(); it != threads_.end();) {
        if ((*it)->isFinished()) {
            delete *it;
            it = threads_.erase(it);
        } else {
            ++it;
        }
    }

    QThread* thread = QThread::create([this, input, generation] {
        ToolchainReport report = checker_.run(input);
        QMetaObject::invokeMethod(
            this, [this, generation, report = std::move(report)] { deliver(generation, report); }, Qt::QueuedConnection);
    });
    threads_.push_back(thread);
    thread->start();
}

void ToolchainRunner::deliver(quint64 generation, const ToolchainReport& report) {
    if (generation != generation_) return;   // superseded by a newer run
    running_ = false;
    emit finished(report);
}

// ---- WizardViewModel ----------------------------------------------------------------------------------

WizardViewModel::WizardViewModel(AppServices& services, QObject* parent)
    : QObject(parent), services_(services), runner_(services.toolchain(), this) {
    connect(&runner_, &ToolchainRunner::started, this, &WizardViewModel::toolchainChanged);
    connect(&runner_, &ToolchainRunner::finished, this, [this](const ToolchainReport& report) {
        toolchain_ = report;
        toolchainChecked_ = true;
        emit toolchainChanged();
        emit summaryChanged();
    });
    basics_ = defaultBasicsFor(sdk::drivetrain_kind::kSwerve);
    revalidateBasics();

    // Re-opening the wizard on the current project starts from what is stored.
    if (services_.session().hasProject()) setProjectPath(services_.session().path());
}

WizardViewModel::~WizardViewModel() = default;

UnitFormatter WizardViewModel::units() const { return UnitFormatter(services_.settings().data().units); }

// ---- navigation ----

bool WizardViewModel::stepComplete(WizardStep step) const {
    switch (step) {
        case WizardStep::Project: return projectValid();
        case WizardStep::Toolchain: return toolchainChecked_ && !runner_.running();
        case WizardStep::Robot: return basicsValid();
        case WizardStep::Summary: return false;
    }
    return false;
}

WizardStep WizardViewModel::furthestReachable() const {
    int reach = 0;
    while (reach < kWizardStepCount - 1 && stepComplete(static_cast<WizardStep>(reach))) ++reach;
    return static_cast<WizardStep>(reach);
}

bool WizardViewModel::canGoNext() const { return !isLastStep() && stepComplete(step_); }

void WizardViewModel::goTo(WizardStep target) {
    const int wanted = static_cast<int>(target);
    if (wanted > static_cast<int>(furthestReachable()) && wanted > static_cast<int>(step_)) return;
    if (target == step_) return;
    step_ = target;
    if (step_ == WizardStep::Toolchain && !toolchainChecked_ && !runner_.running() && projectValid()) startToolchainCheck();
    emit stepChanged(step_);
}

void WizardViewModel::next() {
    if (canGoNext()) goTo(static_cast<WizardStep>(static_cast<int>(step_) + 1));
}

void WizardViewModel::back() {
    if (canGoBack()) goTo(static_cast<WizardStep>(static_cast<int>(step_) - 1));
}

// ---- project ----

void WizardViewModel::setProjectPath(const QString& path) {
    const QString trimmed = path.trimmed();
    ProjectInspection inspection;
    if (trimmed.isEmpty()) {
        projectPath_.clear();
        inspection_ = {};
        existingConfig_ = false;
        toolchainChecked_ = false;
        emit projectChanged();
        emit summaryChanged();
        return;
    }
    const QString root = GradleProjectInspector::resolveProjectRoot(trimmed);
    applyProject(services_.inspector().inspect(root));
}

void WizardViewModel::applyProject(ProjectInspection inspection) {
    const bool sameProject = inspection.path == inspection_.path && inspection.usable() == inspection_.usable();
    projectPath_ = inspection.path;
    inspection_ = std::move(inspection);
    existingConfig_ = false;
    if (projectValid()) {
        existingConfig_ = services_.configs().exists(inspection_.path);
        loadExistingBasics();
        if (!sameProject) {
            toolchainChecked_ = false;
            toolchain_ = {};
        }
        startToolchainCheck();
    } else {
        toolchainChecked_ = false;
        toolchain_ = {};
    }
    emit projectChanged();
    emit summaryChanged();
}

void WizardViewModel::loadExistingBasics() {
    const ProjectPrefs prefs = services_.projectPrefs().load(inspection_.path);
    if (existingConfig_) {
        if (auto loaded = services_.configs().load(inspection_.path)) {
            basics_ = basicsFromConfig(loaded.value().config);
        } else {
            existingConfig_ = false;   // unreadable: treat as a fresh setup, the old file is replaced on finish
        }
    }
    if (!existingConfig_ && !basicsTouched_) {
        basics_ = defaultBasicsFor(basics_.drivetrain);
        basics_.name = inspection_.projectName.isEmpty() ? QStringLiteral("Robot") : inspection_.projectName;
    }
    basics_.alliance = prefs.alliance;
    basics_.station = prefs.station;
    revalidateBasics();
    emit basicsChanged();
}

// ---- toolchain ----

ToolchainInput WizardViewModel::toolchainInput() const {
    ToolchainInput in;
    in.projectPath = inspection_.path;
    in.language = inspection_.language;
    in.wpilibYear = inspection_.wpilibYear;
    in.wpilibHomeOverride = services_.settings().data().wpilibHome;
    in.jdkHomeOverride = services_.settings().data().jdkHome;
    return in;
}

void WizardViewModel::startToolchainCheck() { runner_.run(toolchainInput()); }

void WizardViewModel::recheckToolchain() {
    if (!projectValid()) return;
    toolchainChecked_ = false;
    startToolchainCheck();
}

// ---- robot basics ----

void WizardViewModel::revalidateBasics() { issues_ = validateBasics(basics_); }

void WizardViewModel::setBasics(const RobotBasics& basics) {
    if (basics == basics_) return;
    basics_ = basics;
    basicsTouched_ = true;
    revalidateBasics();
    emit basicsChanged();
    emit summaryChanged();
}

void WizardViewModel::setDrivetrainKind(const std::string& kind) {
    if (kind == basics_.drivetrain) return;
    RobotBasics next = defaultBasicsFor(kind);
    next.name = basics_.name;
    next.alliance = basics_.alliance;
    next.station = basics_.station;
    setBasics(next);
}

// ---- summary ----

void WizardViewModel::setDetectDevicesAfter(bool detect) {
    if (detect == detectAfter_) return;
    detectAfter_ = detect;
    emit summaryChanged();
}

bool WizardViewModel::discoveryPossible() const { return projectValid() && toolchainChecked_ && toolchain_.canRunRobot(); }

RobotBuildContext WizardViewModel::buildContext() const { return services_.buildContext(inspection_); }

sdk::RobotConfig WizardViewModel::previewConfig() const { return buildRobotConfig(basics_, buildContext()); }

WizardResult WizardViewModel::finish() {
    WizardResult result;
    if (!projectValid()) {
        result.message = tr("Pick a WPILib project first.");
        return result;
    }
    if (!basicsValid()) {
        result.message = tr("Fix the highlighted robot values first.");
        return result;
    }

    ProjectSession& session = services_.session();
    if (auto flushed = session.flushPendingChanges(); !flushed) {
        result.message = QString::fromStdString(flushed.error().message);
        return result;
    }
    if (!session.hasProject() || session.path() != inspection_.path) {
        if (auto opened = session.open(inspection_.path); !opened) {
            result.message = QString::fromStdString(opened.error().message);
            return result;
        }
    }

    const std::shared_ptr<const sdk::RobotConfig> existing = session.config();
    const sdk::RobotConfig config = updateRobotConfig(existing.get(), basics_, buildContext());
    ProjectPrefs prefs;
    prefs.alliance = basics_.alliance;
    prefs.station = basics_.station;
    result.outcome = session.commit(config, prefs);
    if (!result.outcome.ok()) {
        result.message = tr("Could not save the robot configuration: %1").arg(result.outcome.error);
        return result;
    }
    result.ok = true;
    result.detectDevices = detectAfter_ && discoveryPossible();
    return result;
}

}  // namespace wpiemu::app
