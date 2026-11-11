#include "client/client_session.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

#include "wpiemu/core/match/match_client.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <timeapi.h>
#endif

namespace wpiemu::app::client {
namespace {

namespace mc = core::match;

// The worker wakes every couple of milliseconds; the default Windows timer would round that up to ~15 ms.
class ScopedTimerResolution {
public:
    ScopedTimerResolution() {
#ifdef _WIN32
        active_ = timeBeginPeriod(1) == TIMERR_NOERROR;
#endif
    }
    ~ScopedTimerResolution() {
#ifdef _WIN32
        if (active_) timeEndPeriod(1);
#endif
    }
    ScopedTimerResolution(const ScopedTimerResolution&) = delete;
    ScopedTimerResolution& operator=(const ScopedTimerResolution&) = delete;

private:
    [[maybe_unused]] bool active_ = false;
};

bool joined(mc::ConnectionState state) {
    return state == mc::ConnectionState::Lobby || state == mc::ConnectionState::Live || state == mc::ConnectionState::Paused ||
           state == mc::ConnectionState::HostStalled;
}

}  // namespace

struct ClientSession::Impl {
    Impl(const ClientServices& s, ClientSessionOptions o, sdk::ILogger* l) : services(s), options(std::move(o)), logger(l) {
        if (!options.opener)
            options.opener = [link = options.link](const JoinPlan& plan) { return openSecureLink(plan.endpoint, plan.secret, link); };
        worker = std::thread([this] { run(); });
    }

    ~Impl() {
        stopping = true;
        ++generation;
        wake.notify_all();
        if (worker.joinable()) worker.join();
        if (auto c = current()) c->leave();
    }

    std::shared_ptr<mc::MatchClient> current() const {
        std::lock_guard lock(mutex);
        return client;
    }

    void post(std::function<void()> command) {
        {
            std::lock_guard lock(mutex);
            commands.push_back(std::move(command));
        }
        wake.notify_all();
    }

    void run() {
        const ScopedTimerResolution timer;
        while (!stopping) {
            std::function<void()> command;
            {
                std::lock_guard lock(mutex);
                if (!commands.empty()) {
                    command = std::move(commands.front());
                    commands.pop_front();
                }
            }
            if (command) {
                command();
                continue;
            }
            if (const auto c = current()) {
                c->update();
                if (joined(c->status().state)) {
                    std::lock_guard lock(mutex);
                    everJoined = true;
                }
            }
            std::unique_lock lock(mutex);
            if (commands.empty() && !stopping) wake.wait_for(lock, options.updateInterval);
        }
    }

    void setOpening() {
        std::lock_guard lock(mutex);
        phase = LinkPhase::Opening;
        openError.clear();
        everJoined = false;
        reopening = false;
    }

    void doJoin(std::uint64_t gen, JoinPlan plan, sdk::RobotConfig robot) {
        if (auto old = current()) {
            old->leave();
            std::lock_guard lock(mutex);
            client.reset();
        }
        if (gen != generation) return;
        setOpening();
        auto link = options.opener(plan);
        if (gen != generation) return;   // left (or replaced) while dialling: the link is dropped, the state was already reset
        if (!link) {
            std::lock_guard lock(mutex);
            phase = LinkPhase::Failed;
            openError = link.error().message;
            return;
        }
        mc::MatchClientConfig config = options.match;
        config.displayName = plan.displayName;
        mc::MatchClientServices matchServices = services.matchServices(robot.gamePluginId, logger);
        matchServices.reconnect = [this, plan] { return options.opener(plan); };
        auto created = std::make_shared<mc::MatchClient>(std::move(config), std::move(matchServices));
        {
            std::lock_guard lock(mutex);
            pluginId = robot.gamePluginId;
            lastPlan = std::move(plan);
        }
        (void)created->connect(std::move(link).value(), robot);   // a refusal shows in the client's status (BadSpec)
        std::lock_guard lock(mutex);
        client = std::move(created);
        phase = LinkPhase::Active;
    }

    void doLeave() {
        std::shared_ptr<mc::MatchClient> old;
        {
            std::lock_guard lock(mutex);
            old = std::move(client);
            client.reset();
            phase = LinkPhase::Idle;
            openError.clear();
            everJoined = false;
            reopening = false;
        }
        if (old) old->leave();
    }

    void doReconnect(std::uint64_t gen) {
        std::optional<JoinPlan> plan;
        {
            std::lock_guard lock(mutex);
            plan = lastPlan;
            reopening = true;
            openError.clear();
        }
        const auto c = current();
        if (!c || !plan) {
            std::lock_guard lock(mutex);
            reopening = false;
            return;
        }
        auto link = options.opener(*plan);
        if (gen != generation) {
            std::lock_guard lock(mutex);
            reopening = false;
            return;
        }
        if (!link) {
            std::lock_guard lock(mutex);
            reopening = false;
            openError = link.error().message;
            return;
        }
        (void)c->resume(std::move(link).value());
        std::lock_guard lock(mutex);
        reopening = false;
    }

    const ClientServices& services;
    ClientSessionOptions options;
    sdk::ILogger* logger;

    mutable std::mutex mutex;
    std::condition_variable wake;
    std::deque<std::function<void()>> commands;
    std::shared_ptr<mc::MatchClient> client;
    LinkPhase phase = LinkPhase::Idle;
    std::string openError;
    bool everJoined = false;
    bool reopening = false;
    std::string pluginId;
    std::optional<JoinPlan> lastPlan;
    std::atomic<std::uint64_t> generation{0};
    std::atomic<bool> stopping{false};
    std::thread worker;
};

ClientSession::ClientSession(const ClientServices& services, ClientSessionOptions options, sdk::ILogger* logger)
    : impl_(std::make_unique<Impl>(services, std::move(options), logger)) {}

ClientSession::~ClientSession() = default;

void ClientSession::join(JoinPlan plan, sdk::RobotConfig robot) {
    const std::uint64_t gen = ++impl_->generation;
    {
        std::lock_guard lock(impl_->mutex);
        impl_->phase = LinkPhase::Opening;
        impl_->openError.clear();
        impl_->everJoined = false;
    }
    impl_->post([impl = impl_.get(), gen, plan = std::move(plan), robot = std::move(robot)]() mutable {
        impl->doJoin(gen, std::move(plan), std::move(robot));
    });
}

void ClientSession::leave() {
    ++impl_->generation;
    impl_->post([impl = impl_.get()] { impl->doLeave(); });
}

void ClientSession::reconnect() {
    const std::uint64_t gen = impl_->generation.load();
    impl_->post([impl = impl_.get(), gen] { impl->doReconnect(gen); });
}

SessionState ClientSession::state() const {
    SessionState s;
    std::shared_ptr<mc::MatchClient> c;
    {
        std::lock_guard lock(impl_->mutex);
        s.phase = impl_->phase;
        s.openError = impl_->openError;
        s.everJoined = impl_->everJoined;
        s.reopening = impl_->reopening;
        c = impl_->client;
    }
    if (c) s.status = c->status();
    if (s.reopening) {
        s.status.state = mc::ConnectionState::Connecting;
        s.status.reconnecting = true;
    }
    return s;
}

std::shared_ptr<const core::match::MatchClientView> ClientSession::view() const {
    const auto c = impl_->current();
    return c ? c->view() : nullptr;
}

std::shared_ptr<const sdk::RobotConfig> ClientSession::acceptedSpec() const {
    const auto c = impl_->current();
    return c ? c->acceptedSpec() : nullptr;
}

std::shared_ptr<const sdk::FieldDescription> ClientSession::fieldDescription() const {
    const auto c = impl_->current();
    return c ? c->fieldDescription() : nullptr;
}

std::string ClientSession::gamePluginId() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->pluginId;
}

sdk::Status ClientSession::driveRobotRelative(const sdk::ChassisSpeeds& speeds) {
    const auto c = impl_->current();
    return c ? c->driveRobotRelative(speeds) : sdk::makeError(sdk::ErrorCode::InvalidState, "not joined");
}

sdk::Status ClientSession::driveFieldRelative(const sdk::ChassisSpeeds& fieldSpeeds) {
    const auto c = impl_->current();
    return c ? c->driveFieldRelative(fieldSpeeds) : sdk::makeError(sdk::ErrorCode::InvalidState, "not joined");
}

void ClientSession::setZoneFlags(std::uint32_t flags) {
    if (const auto c = impl_->current()) c->setZoneFlags(flags);
}

sdk::Status ClientSession::setZonePose(std::size_t zone, const sdk::Pose3& chassisPose) {
    const auto c = impl_->current();
    return c ? c->setZonePose(zone, chassisPose) : sdk::makeError(sdk::ErrorCode::InvalidState, "not joined");
}

void ClientSession::clearZonePose(std::size_t zone) {
    if (const auto c = impl_->current()) c->clearZonePose(zone);
}

}  // namespace wpiemu::app::client
