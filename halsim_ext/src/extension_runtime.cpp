#include "wpiemu/halsim_ext/extension_runtime.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <utility>
#include <variant>

#include "wpiemu/halsim_ext/ipc_link.h"
#include "wpiemu/halsim_ext/sim_session.h"

namespace wpiemu::halsim {

namespace proto = sdk::proto;
using sdk::ErrorCode;

namespace {
constexpr const char* kLog = "extension";
constexpr std::chrono::milliseconds kJoinTimeout{3000};
constexpr std::uint32_t kDefaultStepUs = 5000;
}  // namespace

sdk::Result<proto::HelloAck> performHandshake(proto::MessageChannel& channel, const RobotInfo& robot,
                                              const ExtensionConfig& config, sdk::ILogger& logger) {
    proto::Hello hello;
    hello.language = robot.language;
    hello.wpilibYear = robot.wpilibYear;
    hello.wpilibVersion = robot.wpilibVersion;
    hello.extensionVersion = robot.extensionVersion;
    hello.pid = robot.pid;
    hello.programName = config.programName.empty() ? robot.programName : config.programName;
    hello.halPortCounts = robot.halPortCounts;
    hello.proposedWatchdogMs = config.proposedWatchdogMs;
    hello.supportsArmed = true;
    if (const sdk::Status sent = channel.send(proto::Message(std::move(hello))); !sent) return sent.error();

    const auto deadline = std::chrono::steady_clock::now() + config.handshakeTimeout;
    for (;;) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return sdk::makeError(ErrorCode::Timeout, "no HelloAck from the emulator");
        const auto slice = std::min(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now),
                                    std::chrono::milliseconds(100));
        proto::Message message;
        const sdk::Result<bool> got = channel.receive(message, slice);
        if (!got) {
            if (got.error().code == ErrorCode::Unsupported) continue;  // unknown message type: skip
            return got.error();
        }
        if (!got.value()) continue;
        if (auto* ack = std::get_if<proto::HelloAck>(&message)) {
            if (!ack->accepted) return sdk::makeError(ErrorCode::InvalidState, "emulator refused the connection: " + ack->reason);
            if (ack->protocolVersion < proto::kMinCompatibleProtocolVersion)
                return sdk::makeError(ErrorCode::VersionMismatch,
                                      "emulator speaks protocol v" + std::to_string(ack->protocolVersion));
            if (ack->stepUs == 0) {
                logger.warn(kLog, "HelloAck.stepUs is 0; using 5000");
                ack->stepUs = kDefaultStepUs;
            }
            return std::move(*ack);
        }
        if (auto* bye = std::get_if<proto::Goodbye>(&message))
            return sdk::makeError(ErrorCode::Cancelled, "emulator closed the connection: " + bye->reason);
        logger.debug(kLog, "ignoring message received before HelloAck");
    }
}

struct ExtensionRuntime::Impl {
    explicit Impl(RuntimeDeps d)
        : deps(std::move(d)),
          logger(deps.config.logLevel, deps.console),
          bridges(std::move(deps.bridges), devices, logger),
          services{deps.config, deps.ports, bridges, devices, logger, stats} {
        bridges.setVisionBridges(std::move(deps.visionBridges));
    }

    sdk::Result<std::unique_ptr<SimSession>> connect() {
        sdk::Result<std::unique_ptr<proto::ITransport>> transport =
            deps.client.connect(deps.config.endpoint, deps.config.connectTimeout);
        if (!transport) return transport.error();
        auto channel = std::make_unique<proto::MessageChannel>(std::move(transport).value(), deps.codec);
        sdk::Result<proto::HelloAck> ack = performHandshake(*channel, deps.robot, deps.config, logger);
        if (!ack) {
            channel->close();
            return ack.error();
        }
        if (ack.value().enableL3Hooks) optionalHooks.store(true);
        stats.sessions.fetch_add(1, std::memory_order_relaxed);
        auto link = std::make_unique<IpcLink>(std::move(channel));
        return std::make_unique<SimSession>(services, std::move(link), ack.value());
    }

    void clockMain() {
        for (;;) {
            SimSession* session = nullptr;
            {
                std::lock_guard lock(mutex);
                session = current.get();
            }
            if (session == nullptr) break;
            const SessionEnd end = session->run();
            const proto::WatchdogPolicy policy = session->watchdogPolicy();
            {
                std::lock_guard lock(mutex);
                current.reset();
            }
            if (stopping || end != SessionEnd::Disconnected) break;
            if (!deps.config.reconnect || policy == proto::WatchdogPolicy::Terminate) break;

            logger.info(kLog, "waiting for the emulator to come back (reconnect every " +
                                  std::to_string(deps.config.reconnectInterval.count()) + " ms)");
            std::unique_ptr<SimSession> next;
            while (!stopping) {
                {
                    std::unique_lock lock(mutex);
                    cv.wait_for(lock, deps.config.reconnectInterval, [&] { return stopping.load(); });
                }
                if (stopping) break;
                sdk::Result<std::unique_ptr<SimSession>> attempt = connect();
                if (attempt) {
                    next = std::move(attempt).value();
                    break;
                }
                logger.debug(kLog, "reconnect failed: " + attempt.error().message);
            }
            if (!next) break;
            next->begin();
            if (next->lockstep()) deps.ports.clock.pauseTiming();
            logger.info(kLog, "reconnected to the emulator");
            {
                std::lock_guard lock(mutex);
                current = std::move(next);
                if (stopping) current->requestShutdown();
            }
        }
        {
            std::lock_guard lock(mutex);
            finished = true;
        }
        cv.notify_all();
    }

    RuntimeDeps deps;
    ForwardingLogger logger;
    DeviceTable devices;
    BridgeSet bridges;
    AtomicRuntimeStats stats;
    SessionServices services;
    std::atomic<bool> optionalHooks{false};

    mutable std::mutex mutex;
    mutable std::condition_variable cv;
    std::unique_ptr<SimSession> current;
    std::thread clockThread;
    bool started = false;
    bool finished = false;
    std::atomic<bool> stopping{false};
};

ExtensionRuntime::ExtensionRuntime(RuntimeDeps deps) : impl_(std::make_shared<Impl>(std::move(deps))) {}

ExtensionRuntime::~ExtensionRuntime() { stop(); }

sdk::Status ExtensionRuntime::start() {
    Impl& s = *impl_;
    {
        std::lock_guard lock(s.mutex);
        if (s.started) return sdk::makeError(ErrorCode::InvalidState, "extension runtime already started");
        s.started = true;
    }
    auto fail = [&](sdk::Error error) {
        std::lock_guard lock(s.mutex);
        s.started = false;
        return sdk::Status(std::move(error));
    };
    if (s.deps.config.endpoint.empty()) return fail(sdk::Error(ErrorCode::InvalidArgument, "no endpoint configured"));

    sdk::Result<std::unique_ptr<SimSession>> session = s.connect();
    if (!session) return fail(session.error());
    std::unique_ptr<SimSession> active = std::move(session).value();
    active->begin();
    // HAL time is paused before the robot code exists (ADR 0001); the bridges' initialNotify flood
    // then lands in the outbound queue in order.
    if (active->lockstep()) s.deps.ports.clock.pauseTiming();
    s.bridges.start(s.optionalHooks);
    s.logger.info(kLog, std::string("connected to ") + s.deps.config.endpoint + (active->lockstep() ? " (lock-step)" : " (free-running)"));
    {
        std::lock_guard lock(s.mutex);
        s.current = std::move(active);
    }
    std::shared_ptr<Impl> keepAlive = impl_;
    s.clockThread = std::thread([keepAlive] { keepAlive->clockMain(); });
    return sdk::okStatus();
}

void ExtensionRuntime::stop() noexcept {
    if (!impl_) return;
    Impl& s = *impl_;
    {
        std::lock_guard lock(s.mutex);
        if (!s.started || s.stopping.exchange(true)) return;
        if (s.current) s.current->requestShutdown();
    }
    s.cv.notify_all();
    s.deps.ports.clock.resumeTiming();  // never leave the robot frozen, whatever state we are in

    std::unique_lock lock(s.mutex);
    if (s.cv.wait_for(lock, kJoinTimeout, [&] { return s.finished; })) {
        lock.unlock();
        if (s.clockThread.joinable() && s.clockThread.get_id() != std::this_thread::get_id()) s.clockThread.join();
        s.bridges.stop();
    } else {
        lock.unlock();
        // Recovery and bounded stop must not wait for a blocked console/logger.
        if (s.clockThread.joinable()) s.clockThread.detach();
    }
}

bool ExtensionRuntime::finished() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->finished;
}

bool ExtensionRuntime::waitFinished(std::chrono::milliseconds timeout) const {
    std::unique_lock lock(impl_->mutex);
    return impl_->cv.wait_for(lock, timeout, [&] { return impl_->finished; });
}

RuntimeStats ExtensionRuntime::stats() const { return impl_->stats.snapshot(); }

sdk::ILogger& ExtensionRuntime::logger() { return impl_->logger; }

}  // namespace wpiemu::halsim
