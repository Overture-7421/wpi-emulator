#include "common/hosting_service.h"

#include <algorithm>

namespace wpiemu::app {

namespace {
constexpr auto kMode = "hosting/mode";
constexpr auto kPort = "hosting/port";
constexpr auto kRelay = "hosting/relayEndpoint";
constexpr auto kPlayers = "hosting/maxPlayers";
constexpr auto kBind = "hosting/bindHost";
constexpr auto kPhaseKey = "hosting/phaseHudKey";
}  // namespace

HostingService::HostingService(IKeyValueStore& store, QObject* parent)
    : QObject(parent), store_(store), status_(std::make_shared<const host::HostingStatus>()) {
    load();
}

void HostingService::load() {
    host::HostingOptions o;
    o.mode = host::hostingModeFromKey(store_.value(kMode, QStringLiteral("off")).toString().toStdString());
    const int port = store_.value(kPort, static_cast<int>(host::kDefaultHostingPort)).toInt();
    o.port = port >= 0 && port <= 65535 ? static_cast<std::uint16_t>(port) : host::kDefaultHostingPort;
    o.relayEndpoint = store_.value(kRelay).toString().toStdString();
    o.maxPlayers = static_cast<std::size_t>(std::max(0, store_.value(kPlayers, static_cast<int>(o.maxPlayers)).toInt()));
    o.bindHost = store_.value(kBind, QStringLiteral("::")).toString().toStdString();
    o.phaseHudKey = store_.value(kPhaseKey).toString().toStdString();
    options_ = host::normalizedHosting(std::move(o));
}

void HostingService::save() const {
    store_.setValue(kMode, QString::fromLatin1(host::hostingModeKey(options_.mode)));
    store_.setValue(kPort, static_cast<int>(options_.port));
    store_.setValue(kRelay, QString::fromStdString(options_.relayEndpoint));
    store_.setValue(kPlayers, static_cast<int>(options_.maxPlayers));
    store_.setValue(kBind, QString::fromStdString(options_.bindHost));
    store_.setValue(kPhaseKey, QString::fromStdString(options_.phaseHudKey));
    store_.sync();
}

void HostingService::setOptions(const host::HostingOptions& options) {
    const host::HostingOptions clean = host::normalizedHosting(options);
    if (clean == options_) return;
    options_ = clean;
    save();
    emit optionsChanged();
}

void HostingService::setPhaseHudKey(const std::string& key) {
    host::HostingOptions next = options_;
    next.phaseHudKey = key;
    setOptions(next);
}

void HostingService::kick(sdk::match::ParticipantId participant, const QString& reason) {
    if (!status_->hosting() || participant == sdk::match::kHostParticipantId) return;
    emit kickRequested(participant, reason);
}

void HostingService::sessionStarted(const host::HostingOptions& applied) {
    applied_ = applied;
    emit optionsChanged();
}

void HostingService::sessionEnded() {
    applied_.reset();
    publish(std::make_shared<const host::HostingStatus>());
    emit optionsChanged();
}

void HostingService::publish(std::shared_ptr<const host::HostingStatus> status) {
    if (!status) status = std::make_shared<const host::HostingStatus>();
    if (status == status_ || *status == *status_) return;
    status_ = std::move(status);
    emit statusChanged();
}

}  // namespace wpiemu::app
