#pragma once

// The emulator end of a protocol connection, for tests of the extension side. A reader thread collects every inbound message
// into an inbox that tests wait on and take from by type; helpers answer Hello, send frames, controls and DsState with the
// identity the test chooses, and step one tick at a time. The extension end is FakeExtension (fake_extension.h).

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/sdk/ipc/lifecycle.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::sdk::testing {

class FakeEmulator {
public:
    FakeEmulator(std::unique_ptr<proto::ITransport> transport, std::shared_ptr<const proto::IMessageCodec> codec)
        : channel_(std::make_unique<proto::MessageChannel>(std::move(transport), std::move(codec))) {
        reader_ = std::thread([this] { readLoop(); });
    }
    ~FakeEmulator() {
        stop_ = true;
        channel_->close();
        if (reader_.joinable()) reader_.join();
    }

    // Answer the next Hello with this ack (sessionId/epoch are what the tests then stamp on frames).
    void autoAck(proto::HelloAck ack) {
        std::lock_guard lock(mutex_);
        autoAck_ = std::move(ack);
    }
    bool send(proto::Message m) { return static_cast<bool>(channel_->send(m)); }
    void disconnect() { channel_->close(); }

    template <class T, class Pred = std::function<bool(const T&)>>
    std::optional<T> take(std::chrono::milliseconds timeout, Pred pred = [](const T&) { return true; }) {
        std::unique_lock lock(mutex_);
        std::optional<T> found;
        cv_.wait_for(lock, timeout, [&] {
            for (auto it = inbox_.begin(); it != inbox_.end(); ++it) {
                if (const T* v = std::get_if<T>(&*it); v != nullptr && pred(*v)) {
                    found = *v;
                    inbox_.erase(it);
                    return true;
                }
            }
            return false;
        });
        return found;
    }
    template <class T>
    std::size_t count() const {
        std::lock_guard lock(mutex_);
        std::size_t n = 0;
        for (const auto& m : inbox_)
            if (std::holds_alternative<T>(m)) ++n;
        return n;
    }
    bool waitForLifecycle(std::chrono::milliseconds timeout) {
        return take<proto::Log>(timeout, [](const proto::Log& l) {
                   return proto::isLifecycleLog(l, proto::kLifecycleProgramStarted);
               }).has_value();
    }
    bool connectionClosed() const { return closed_; }
    // Message types in arrival order (never consumed by take()).
    static constexpr int kHistoryLifecycle = 1000;  // the program_started Log (other Logs record as MessageType::Log)
    std::vector<int> history() const {
        std::lock_guard lock(mutex_);
        return history_;
    }

    std::uint64_t sessionId = 7;
    std::uint64_t epoch = proto::kFirstRunEpoch;
    std::uint64_t nextTick = 1;
    SimTimeUs simTimeUs = 0;
    std::vector<CameraFrame> pendingVision;  // attached to the next tick()

    // Sends one SensorFrame and returns the extension's ack (nullopt on timeout).
    std::optional<proto::ActuatorFrame> tick(std::vector<SignalSample> samples = {}, std::uint32_t stepUs = 5000,
                                             double batteryV = 12.0,
                                             std::chrono::milliseconds timeout = std::chrono::milliseconds{2000}) {
        proto::SensorFrame f;
        f.tick = nextTick++;
        simTimeUs += stepUs;
        f.simTimeUs = simTimeUs;
        f.stepUs = stepUs;
        f.batteryVoltageV = batteryV;
        f.samples = std::move(samples);
        f.vision = std::move(pendingVision);
        pendingVision.clear();
        f.runEpoch = epoch;
        const std::uint64_t tickNo = f.tick;
        if (!send(proto::Message(std::move(f)))) return std::nullopt;
        return take<proto::ActuatorFrame>(timeout, [tickNo](const proto::ActuatorFrame& a) {
            return a.ackedTick() == tickNo;
        });
    }
    bool sendControl(proto::SimCommand cmd, std::uint64_t commandId = proto::kUnspecified) {
        proto::SimControl c;
        c.command = cmd;
        c.commandId = commandId;
        c.sessionId = sessionId;
        c.runEpoch = epoch;
        return send(proto::Message(c));
    }
    bool sendDs(proto::DsState ds) {
        ds.sessionId = sessionId;
        ds.runEpoch = epoch;
        return send(proto::Message(std::move(ds)));
    }

private:
    static bool isLifecycle(const proto::Message& m) {
        const auto* log = std::get_if<proto::Log>(&m);
        return log != nullptr && proto::isLifecycleLog(*log, proto::kLifecycleProgramStarted);
    }

    void readLoop() {
        while (!stop_) {
            proto::Message m;
            const Result<bool> got = channel_->receive(m, std::chrono::milliseconds{20});
            if (!got) {
                closed_ = true;
                cv_.notify_all();
                return;
            }
            if (!got.value()) continue;
            std::optional<proto::HelloAck> reply;
            {
                std::lock_guard lock(mutex_);
                if (std::holds_alternative<proto::Hello>(m) && autoAck_) reply = autoAck_;
                history_.push_back(isLifecycle(m) ? kHistoryLifecycle : static_cast<int>(proto::typeOf(m)));
                inbox_.push_back(std::move(m));
            }
            if (reply) (void)channel_->send(proto::Message(*reply));
            cv_.notify_all();
        }
    }

    std::unique_ptr<proto::MessageChannel> channel_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<proto::Message> inbox_;
    std::vector<int> history_;
    std::optional<proto::HelloAck> autoAck_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> closed_{false};
    std::thread reader_;
};

}  // namespace wpiemu::sdk::testing
