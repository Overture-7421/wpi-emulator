#pragma once

// Shared helpers for the ipc tests: transport "links" over every transport kind, the fake extension (HALSim side) on a driver
// thread and an emulator-side driver (LockstepWindow).

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <doctest/doctest.h>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/handshake.h"
#include "wpiemu/ipc/in_process.h"
#include "wpiemu/ipc/local_ipc.h"
#include "wpiemu/sdk/ipc/lockstep.h"
#include "wpiemu/sdk/testing/fake_extension.h"
#include "wpiemu/sdk/testing/wait.h"

namespace wpiemu::ipc::testing {

using namespace std::chrono_literals;
namespace proto = sdk::proto;

enum class TransportKind { InProcess, Native };

inline const char* toString(TransportKind k) { return k == TransportKind::InProcess ? "in-process" : "native"; }

template <class Fn>
void forEachTransportKind(Fn&& fn) {
    for (TransportKind kind : {TransportKind::InProcess, TransportKind::Native}) {
        INFO("transport: " << toString(kind));
        fn(kind);
    }
}

// A listening server plus a client factory for one transport kind.
struct Link {
    explicit Link(TransportKind kind, ipc::FramedTransportOptions options = {}) {
        if (kind == TransportKind::InProcess) {
            auto network = InProcessNetwork::create({}, options);
            const ConnectionEndpoint ep = ConnectionEndpoint::generateInProcess();
            address = ep.address();
            server = network->createServer(address);
            client = network->createClient();
        } else {
            const ConnectionEndpoint ep = ConnectionEndpoint::generate();
            address = ep.address();
            auto made = makeLocalServer(ep, options);
            REQUIRE(made.ok());
            server = std::move(made).value();
            client = makeLocalClient(options);
        }
        const sdk::Status listening = server->listen();
        REQUIRE_MESSAGE(listening.ok(), (listening.ok() ? std::string() : listening.error().message));
    }

    struct Pair {
        std::unique_ptr<proto::ITransport> emulator;  // server side
        std::unique_ptr<proto::ITransport> extension;  // client side
    };

    // Connects one client and accepts it (works sequentially: the OS queues the connection).
    Pair connect() {
        auto c = client->connect(address, 5000ms);
        REQUIRE_MESSAGE(c.ok(), (c.ok() ? std::string() : c.error().message));
        auto a = server->accept(5000ms);
        REQUIRE_MESSAGE(a.ok(), (a.ok() ? std::string() : a.error().message));
        REQUIRE(a.value() != nullptr);
        return {std::move(a).value(), std::move(c).value()};
    }

    std::string address;
    std::unique_ptr<proto::ITransportServer> server;
    std::unique_ptr<proto::ITransportClient> client;
};

inline std::shared_ptr<const proto::IMessageCodec> codec() { return std::make_shared<BinaryMessageCodec>(); }

// ---- extension side -------------------------------------------------------------------------------------

// The shared protocol double on a driver thread: it says Hello at once and answers every sample doubled until it leaves
// lock-step (Stop, Goodbye, a dropped connection) or is stopped. Counters and the identity it adopted are readable
// from the test thread.
class FakeExtension : public sdk::testing::FakeExtension {
public:
    using SensorHook = std::function<Reply(const proto::SensorFrame&, FakeExtension&)>;

    FakeExtension(std::unique_ptr<proto::ITransport> transport, SensorHook hook = {}, std::uint32_t proposedWatchdogMs = 0)
        : sdk::testing::FakeExtension(std::move(transport), codec()), hook_(std::move(hook)) {
        leaveOnStop = true;
        onFrame = [this](const proto::SensorFrame& sf, sdk::testing::FakeExtension&) {
            const Reply decided = hook_ ? hook_(sf, *this) : Reply::Answer;
            outputs.clear();
            for (const sdk::SignalSample& s : sf.samples) outputs.push_back({s.device, s.signal, s.value * 2.0});
            return decided;
        };
        thread_ = std::thread([this, proposedWatchdogMs] { run(proposedWatchdogMs); });
    }
    ~FakeExtension() { stop(); }

    void stop() {
        stop_ = true;
        ch.close();
        if (thread_.joinable()) thread_.join();
    }

    // Extension-thread API for hooks.
    proto::MessageChannel& channel() { return ch; }

    bool waitHandshaken(std::chrono::milliseconds timeout) const {
        sdk::testing::waitUntil([&] { return handshaken_ || finished_; }, timeout, 1ms);
        return handshaken_;
    }

    std::atomic<std::uint64_t> epoch{0};
    std::atomic<std::uint32_t> watchdogMs{0};

private:
    void run(std::uint32_t proposedWatchdogMs) {
        hello(proposedWatchdogMs);
        while (!stop_ && !disconnected) {
            pump(20ms);
            epoch = follower.runEpoch();
            if (!helloAck) continue;
            watchdogMs = helloAck->watchdogTimeoutMs;
            if (!helloAck->accepted) break;
            handshaken_ = true;
        }
        ch.close();  // the extension leaving lock-step drops the connection
        finished_ = true;
    }

    SensorHook hook_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> handshaken_{false};
    std::atomic<bool> finished_{false};
    std::thread thread_;
};

// ---- emulator side -----------------------------------------------------------------------------------------

enum class TickOutcome { Completed, Stalled, Stopped, ProtocolError, Timeout, Disconnected, SendFailed };

class EmulatorSide {
public:
    explicit EmulatorSide(std::unique_ptr<proto::ITransport> transport) : channel_(std::move(transport), codec()) {}

    sdk::Status handshake(std::uint64_t sessionId, std::uint64_t epoch, std::uint32_t preferredWatchdogMs = 0) {
        ServerHandshakeConfig cfg;
        cfg.sessionId = sessionId;
        cfg.runEpoch = epoch;
        cfg.preferredWatchdogMs = preferredWatchdogMs;
        auto r = performServerHandshake(channel_, cfg, 5000ms);
        if (!r) return r.error();
        session_ = r.value().ack.sessionId;
        epoch_ = r.value().ack.runEpoch;
        watchdogMs_ = r.value().ack.watchdogTimeoutMs;
        window_.begin(session_, epoch_);
        return sdk::okStatus();
    }

    proto::SensorFrame makeFrame(std::uint64_t tick) const {
        proto::SensorFrame f;
        f.tick = tick;
        f.simTimeUs = static_cast<sdk::SimTimeUs>(tick) * 5000;
        f.stepUs = 5000;
        f.sessionId = session_;
        f.runEpoch = epoch_;
        f.samples.push_back({sdk::DeviceId{1}, 0, static_cast<double>(tick)});
        return f;
    }

    // One lock-step tick: send SensorFrame, wait (at most `watchdog`) for the acknowledging ActuatorFrame.
    TickOutcome tick(std::uint64_t n, std::chrono::milliseconds watchdog, proto::ActuatorFrame* answer = nullptr) {
        const proto::SensorFrame frame = makeFrame(n);
        if (!window_.onSent(frame).ok()) return TickOutcome::SendFailed;
        if (!channel_.send(frame).ok()) {
            window_.onWatchdogExpired();
            return TickOutcome::Disconnected;
        }
        const auto deadline = std::chrono::steady_clock::now() + watchdog;
        for (;;) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            proto::Message m;
            auto got = receiveSkippingUnknown(channel_, m, left.count() > 0 ? left : 0ms);
            if (!got) {
                window_.onWatchdogExpired();
                return TickOutcome::Disconnected;
            }
            if (!got.value()) {
                window_.onWatchdogExpired();
                return TickOutcome::Timeout;
            }
            if (const auto* ack = std::get_if<proto::SimControlAck>(&m)) {
                acks_.push_back(*ack);
                continue;
            }
            const auto* af = std::get_if<proto::ActuatorFrame>(&m);
            if (af == nullptr) continue;
            switch (window_.onAck(*af)) {
                case proto::AckVerdict::Dropped: continue;
                case proto::AckVerdict::Completed:
                    if (answer != nullptr) *answer = *af;
                    return TickOutcome::Completed;
                case proto::AckVerdict::Stalled: return TickOutcome::Stalled;
                case proto::AckVerdict::Stopped: return TickOutcome::Stopped;
                case proto::AckVerdict::ProtocolError: return TickOutcome::ProtocolError;
            }
        }
    }

    // Reset: bump the epoch, re-open the window, send the command (the first message of the new epoch).
    sdk::Status reset(std::uint64_t commandId) {
        ++epoch_;
        window_.begin(session_, epoch_);
        return sendControl(proto::SimCommand::Reset, commandId, epoch_);
    }

    sdk::Status sendControl(proto::SimCommand command, std::uint64_t commandId, std::uint64_t epoch) {
        proto::SimControl c;
        c.command = command;
        c.commandId = commandId;
        c.sessionId = session_;
        c.runEpoch = epoch;
        return channel_.send(c);
    }

    // Reads until the SimControlAck for `commandId` arrives (late ActuatorFrames are fed to the window).
    bool awaitAck(std::uint64_t commandId, std::chrono::milliseconds timeout, proto::SimControlAck* out = nullptr) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            for (const auto& a : acks_)
                if (a.commandId == commandId) {
                    if (out != nullptr) *out = a;
                    return true;
                }
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (left.count() <= 0) return false;
            proto::Message m;
            auto got = receiveSkippingUnknown(channel_, m, left);
            if (!got || !got.value()) return false;
            if (const auto* ack = std::get_if<proto::SimControlAck>(&m)) acks_.push_back(*ack);
            if (const auto* af = std::get_if<proto::ActuatorFrame>(&m)) (void)window_.onAck(*af);
        }
    }

    proto::MessageChannel& channel() { return channel_; }
    proto::LockstepWindow& window() { return window_; }
    std::uint64_t epoch() const { return epoch_; }
    std::uint64_t session() const { return session_; }
    std::uint32_t watchdogMs() const { return watchdogMs_; }

private:
    proto::MessageChannel channel_;
    proto::LockstepWindow window_;
    std::uint64_t session_ = 0;
    std::uint64_t epoch_ = 0;
    std::uint32_t watchdogMs_ = 0;
    std::vector<proto::SimControlAck> acks_;
};

}  // namespace wpiemu::ipc::testing
