#include "client/client_controller.h"

#include <algorithm>

#include "wpiemu/sdk/match/robot_appearance.h"

namespace wpiemu::app::client {
namespace {

namespace mc = core::match;

constexpr double kDefaultTickSeconds = 1.0 / 60.0;
constexpr double kMaxTickSeconds = 0.1;

std::filesystem::path pathOf(const QString& text) { return std::filesystem::path(text.toStdU16String()); }

}  // namespace

ClientController::ClientController(ClientServices& services, IKeyValueStore& store, IClientInput* input, ClientSessionOptions session,
                                   QObject* parent)
    : ClientController(services, store, input, std::make_unique<ClientSession>(services, std::move(session)), parent) {}

ClientController::ClientController(ClientServices& services, IKeyValueStore& store, IClientInput* input,
                                   std::unique_ptr<IClientSession> session, QObject* parent)
    : QObject(parent), services_(services), settings_(store), input_(input), ownedSession_(std::move(session)),
      session_(*ownedSession_), prefs_(settings_.load()) {}

ClientController::~ClientController() = default;

void ClientController::setPrefs(const ClientPrefs& prefs) {
    prefs_ = prefs;
    prefs_.drive = normalizedDriveConfig(prefs_.drive);
    settings_.save(prefs_);
}

void ClientController::chooseRobot(const QString& path) {
    robot_ = path.trimmed().isEmpty() ? RobotFileReport{} : loadRobotFile(pathOf(path.trimmed()), services_);
    emit robotChanged();
}

JoinValidation ClientController::validate(const JoinFormInput& input) const { return validateJoin(input, relayPresets()); }

JoinValidation ClientController::join(const JoinFormInput& input) {
    JoinValidation result = validate(input);
    const QString wanted = QString::fromStdString(input.robotPath).trimmed();
    if (!wanted.isEmpty()) {
        if (robot_.path != utf8Of(pathOf(wanted))) chooseRobot(wanted);
        if (!robot_.ok) {
            result.plan.reset();
            result.errors.push_back({JoinField::Robot, "robot.invalid", robot_.errors.empty() ? std::string() : robot_.errors.front()});
        }
    }
    if (!result.ok()) return result;

    ClientPrefs next = prefs_;
    next.displayName = input.displayName;
    next.directAddress = input.directAddress;
    next.relay = input.relay;
    next.customRelay = input.customRelay;
    next.robotPath = input.robotPath;
    setPrefs(next);   // the invite is deliberately not part of the preferences

    uiZoneFlags_ = 0;
    session_.join(*result.plan, *robot_.config);
    return result;
}

void ClientController::leave() {
    session_.leave();
    uiZoneFlags_ = 0;
}

void ClientController::reconnect() { session_.reconnect(); }

void ClientController::setZoneButton(std::size_t zone, bool active) {
    if (zone >= 32) return;
    if (active) uiZoneFlags_ |= 1u << zone;
    else uiZoneFlags_ &= ~(1u << zone);
}

void ClientController::moveZone(std::size_t zone, const sdk::Vec3& position, float yawOffsetRad) {
    if (!zonePoses_.set(zone, position, yawOffsetRad)) return;
    (void)session_.setZonePose(zone, zonePoses_.pose(zone));
}

void ClientController::resetZone(std::size_t zone) {
    if (zonePoses_.reset(zone)) session_.clearZonePose(zone);
}

void ClientController::refreshPicture() {
    auto v = session_.view();
    if (!v) {
        if (view_ != nullptr) {
            view_.reset();
            snapshot_.reset();
            ownAppearance_.reset();
            field_.reset();
            sceneConfig_.reset();
            accepted_.reset();
            roster_.reset();
            pluginId_.clear();
            ++sceneRevision_;
            ++rosterRevision_;
            emit sceneChanged();
            emit viewChanged();
        }
        return;
    }
    if (v == view_) return;
    view_ = v;

    auto field = session_.fieldDescription();
    const std::string plugin = session_.gamePluginId();
    bool sceneDirty = false;
    if (v->own.appearance != ownAppearance_ || field != field_ || plugin != pluginId_) {
        ownAppearance_ = v->own.appearance;
        field_ = std::move(field);
        pluginId_ = plugin;
        sceneConfig_ = sceneConfigOf(v->own);
        const sdk::IGameModule* game = services_.gameOf(plugin);
        scene_ = makeClientScene(game, field_, sceneConfig_, v->own.station);
        phaseKey_ = phaseHudKeyOf(scene_.hud);
        accepted_ = session_.acceptedSpec();
        if (accepted_ && ownAppearance_) {
            limits_ = driveLimitsOf(*accepted_);
            zonePoses_.configure(accepted_->chassis, sdk::match::appearanceZoneRestPoses(*ownAppearance_));
        }
        ++sceneRevision_;
        sceneDirty = true;
    }
    auto roster = rosterOf(*v);
    if (!roster_ || !(*roster == *roster_)) {
        roster_ = std::move(roster);
        ++rosterRevision_;
        sceneDirty = true;
    }
    if (sceneDirty) emit sceneChanged();

    snapshot_ = snapshotOf(*v, sceneConfig_.get(), phaseKey_);
    emit viewChanged();
}

void ClientController::sendCommand(double dtSeconds) {
    if (!presentation_.controlsEnabled || !accepted_) {
        if (commandHeld_) {
            (void)session_.driveRobotRelative({});
            session_.setZoneFlags(0);
            commandHeld_ = false;
        }
        return;
    }
    commandHeld_ = true;
    const sdk::proto::JoystickState stick = input_ != nullptr ? input_->sample() : sdk::proto::JoystickState{};
    const sdk::ChassisSpeeds speeds = mapDrive(stick, prefs_.drive, limits_);
    (void)(prefs_.drive.fieldRelative ? session_.driveFieldRelative(speeds) : session_.driveRobotRelative(speeds));
    session_.setZoneFlags((zoneFlagsOf(stick, zonePoses_.zoneCount()) | uiZoneFlags_) &
                          (zonePoses_.zoneCount() >= 32 ? ~0u : (1u << zonePoses_.zoneCount()) - 1u));
    const int moved = zonePoses_.nudge(dtSeconds, stick, prefs_.drive.deadband);
    if (moved >= 0) (void)session_.setZonePose(static_cast<std::size_t>(moved), zonePoses_.pose(static_cast<std::size_t>(moved)));
}

void ClientController::tick(double dtSeconds) {
    const auto now = std::chrono::steady_clock::now();
    if (!(dtSeconds > 0.0)) {
        dtSeconds = lastTick_ == std::chrono::steady_clock::time_point{}
                        ? kDefaultTickSeconds
                        : std::chrono::duration<double>(now - lastTick_).count();
    }
    lastTick_ = now;
    dtSeconds = std::min(dtSeconds, kMaxTickSeconds);

    const Presentation p = present(session_.state());
    if (!(p == presentation_)) {
        presentation_ = p;
        emit presentationChanged();
    }
    refreshPicture();
    sendCommand(dtSeconds);
}

}  // namespace wpiemu::app::client
