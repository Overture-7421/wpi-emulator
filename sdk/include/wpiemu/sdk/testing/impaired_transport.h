#pragma once

// ITransport decorator that makes a perfect in-process connection behave like a bad network, deterministically, for
// tests of the match protocol, prediction, encryption and the relay.
//
// Every frame SENT through the decorator is scheduled by a seeded profile and handed to the wrapped transport when the
// injected IWallClock reaches its due time; nothing sleeps, so a test with a ManualWallClock (fake_clock_logger.h) runs
// instantly and reproducibly:
//   * latency + jitter   every frame is delayed by latency plus a uniform draw in [0, jitter];
//   * reliable frames    never dropped, never duplicated, delivered in send order (a later frame waits for an earlier one);
//   * unreliable frames  (sendUnreliable) may be lost, duplicated, or reordered: a reordered frame is held back by
//                        reorderDelay so later frames overtake it, and jitter alone can also swap neighbours.
// Frames are released when something touches the connection (send, receive, pump) or the clock moves and the test
// calls pump(). Closing drops the frames still in flight.
//
//     ManualWallClock clock;
//     ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 40ms, .lossProbability = 0.1}, seed);
//     link.a->sendUnreliable(frame);
//     clock.advance(40ms);
//     link.pump();                         // or link.b->receive(...), which also releases due frames

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

namespace wpiemu::sdk::testing {

struct ImpairmentProfile {
    std::chrono::microseconds latency{0};  // fixed one-way delay
    std::chrono::microseconds jitter{0};   // extra delay, uniform in [0, jitter]
    double lossProbability = 0.0;          // unreliable frames only
    double reorderProbability = 0.0;       // unreliable frames only
    std::chrono::microseconds reorderDelay{30000};  // how long a reordered frame is held back
    double duplicateProbability = 0.0;     // unreliable frames only (a reliable channel never repeats a frame)
};

struct ImpairmentStats {
    std::uint64_t sent = 0;        // frames handed to the decorator
    std::uint64_t delivered = 0;   // frames handed on to the wrapped transport (duplicates count once each)
    std::uint64_t dropped = 0;
    std::uint64_t duplicated = 0;
    std::uint64_t reordered = 0;
};

namespace detail {

// One or two directions that share a clock and are released together, so either end can release the other's frames.
class ImpairedLink {
public:
    explicit ImpairedLink(const IWallClock& clock) : clock_(clock) {}

    std::size_t addLane(proto::ITransport* sink, const ImpairmentProfile& profile, std::uint64_t seed) {
        std::lock_guard lock(mutex_);
        Lane lane(profile, seed);
        lane.sink = sink;
        lanes_.push_back(std::move(lane));
        return lanes_.size() - 1;
    }

    // The lane's sink is going away: forget it and whatever was still in flight.
    void detach(std::size_t lane) {
        std::lock_guard lock(mutex_);
        lanes_[lane].sink = nullptr;
        lanes_[lane].pending.clear();
    }

    void submit(std::size_t index, std::span<const std::byte> frame, proto::Delivery delivery) {
        std::lock_guard lock(mutex_);
        Lane& lane = lanes_[index];
        ++lane.stats.sent;
        const ImpairmentProfile& p = lane.profile;
        // Always the same five draws, so a seed gives one outcome per frame whatever the profile.
        const double lossDraw = lane.rng.nextDouble();
        const double duplicateDraw = lane.rng.nextDouble();
        const double reorderDraw = lane.rng.nextDouble();
        const std::chrono::microseconds firstJitter = jitterDraw(lane);
        const std::chrono::microseconds secondJitter = jitterDraw(lane);
        const auto now = clock_.now();

        if (delivery == proto::Delivery::Reliable) {
            auto due = now + p.latency + firstJitter;
            due = std::max(due, lane.lastReliableDue);
            lane.lastReliableDue = due;
            lane.pending.push_back({due, lane.nextSeq++, {frame.begin(), frame.end()}, delivery});
            return;
        }
        if (lossDraw < p.lossProbability) {
            ++lane.stats.dropped;
            return;
        }
        auto due = now + p.latency + firstJitter;
        if (reorderDraw < p.reorderProbability) {
            due += p.reorderDelay;
            ++lane.stats.reordered;
        }
        lane.pending.push_back({due, lane.nextSeq++, {frame.begin(), frame.end()}, delivery});
        if (duplicateDraw < p.duplicateProbability) {
            ++lane.stats.duplicated;
            lane.pending.push_back({now + p.latency + secondJitter, lane.nextSeq++, {frame.begin(), frame.end()}, delivery});
        }
    }

    void flush() {
        std::lock_guard lock(mutex_);
        const auto now = clock_.now();
        for (Lane& lane : lanes_) {
            if (lane.sink == nullptr) continue;
            std::vector<Pending> due;
            std::vector<Pending> later;
            for (Pending& p : lane.pending) (p.due > now ? later : due).push_back(std::move(p));
            lane.pending = std::move(later);
            std::sort(due.begin(), due.end(), [](const Pending& a, const Pending& b) {
                return a.due != b.due ? a.due < b.due : a.seq < b.seq;
            });
            for (const Pending& p : due) {
                const Status s = p.delivery == proto::Delivery::Unreliable ? lane.sink->sendUnreliable(p.frame)
                                                                           : lane.sink->send(p.frame);
                if (s.ok()) ++lane.stats.delivered;
            }
        }
    }

    std::size_t inFlight(std::size_t index) const {
        std::lock_guard lock(mutex_);
        return lanes_[index].pending.size();
    }

    ImpairmentStats stats(std::size_t index) const {
        std::lock_guard lock(mutex_);
        return lanes_[index].stats;
    }

private:
    struct Pending {
        std::chrono::steady_clock::time_point due;
        std::uint64_t seq;
        std::vector<std::byte> frame;
        proto::Delivery delivery;
    };

    struct Lane {
        Lane(const ImpairmentProfile& p, std::uint64_t seed) : profile(p), rng(seed) {
            profile.latency = std::max(profile.latency, std::chrono::microseconds{0});
            profile.jitter = std::max(profile.jitter, std::chrono::microseconds{0});
            profile.reorderDelay = std::max(profile.reorderDelay, std::chrono::microseconds{0});
        }
        ImpairmentProfile profile;
        Rng rng;
        proto::ITransport* sink = nullptr;
        std::vector<Pending> pending;
        std::uint64_t nextSeq = 0;
        std::chrono::steady_clock::time_point lastReliableDue{};
        ImpairmentStats stats;
    };

    static std::chrono::microseconds jitterDraw(Lane& lane) {
        const double unit = lane.rng.nextDouble();
        const auto span = static_cast<double>(lane.profile.jitter.count());
        return std::chrono::microseconds{static_cast<std::int64_t>(unit * span)};
    }

    const IWallClock& clock_;
    mutable std::mutex mutex_;
    std::vector<Lane> lanes_;
};

}  // namespace detail

class ImpairedTransport final : public proto::ITransport {
public:
    // Impairs what this transport sends; what it receives was impaired by the peer's end (see makeImpairedPair).
    ImpairedTransport(std::unique_ptr<proto::ITransport> inner, const IWallClock& clock,
                      const ImpairmentProfile& profile, std::uint64_t seed)
        : link_(std::make_shared<detail::ImpairedLink>(clock)), inner_(std::move(inner)) {
        lane_ = link_->addLane(inner_.get(), profile, seed);
    }

    // One end of a pair that shares `link`, with the lane already created by the factory.
    ImpairedTransport(std::shared_ptr<detail::ImpairedLink> link, std::size_t lane,
                      std::unique_ptr<proto::ITransport> inner)
        : link_(std::move(link)), lane_(lane), inner_(std::move(inner)) {}

    ~ImpairedTransport() override { link_->detach(lane_); }

    Status send(std::span<const std::byte> frame) override { return submit(frame, proto::Delivery::Reliable); }
    Status sendUnreliable(std::span<const std::byte> frame) override {
        return submit(frame, proto::Delivery::Unreliable);
    }

    Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            link_->flush();
            // Short slices keep frames that become due while we wait from being held until the call ends.
            const Result<bool> got = inner_->receive(frame, std::min(proto::timeLeft(deadline), std::chrono::milliseconds{1}));
            if (!got || got.value()) return got;
            if (proto::timeLeft(deadline).count() == 0) return false;
        }
    }

    void close() override {
        link_->detach(lane_);
        inner_->close();
    }

    bool connected() const override { return inner_->connected(); }
    std::string describe() const override { return "impaired(" + inner_->describe() + ")"; }

    // Hands on every frame that is due on this connection (both directions of a pair).
    void pump() { link_->flush(); }
    std::size_t inFlight() const { return link_->inFlight(lane_); }
    ImpairmentStats stats() const { return link_->stats(lane_); }

private:
    Status submit(std::span<const std::byte> frame, proto::Delivery delivery) {
        if (!inner_->connected()) return makeError(ErrorCode::Disconnected, "impaired transport is closed");
        link_->submit(lane_, frame, delivery);
        link_->flush();
        return okStatus();
    }

    std::shared_ptr<detail::ImpairedLink> link_;
    std::size_t lane_ = 0;
    std::unique_ptr<proto::ITransport> inner_;
};

struct ImpairedTransportPair {
    std::unique_ptr<ImpairedTransport> a;
    std::unique_ptr<ImpairedTransport> b;

    void pump() { a->pump(); }
};

// `aToB` impairs what `a` sends, `bToA` what `b` sends. Each direction has its own random stream derived from `seed`;
// `clock` must outlive the pair.
inline ImpairedTransportPair makeImpairedPair(const IWallClock& clock, const ImpairmentProfile& aToB,
                                              const ImpairmentProfile& bToA, std::uint64_t seed) {
    InMemoryTransportPair raw = makeInMemoryTransportPair();
    auto link = std::make_shared<detail::ImpairedLink>(clock);
    const std::size_t laneA = link->addLane(raw.a.get(), aToB, seed ^ 0xA5A5A5A5A5A5A5A5ull);
    const std::size_t laneB = link->addLane(raw.b.get(), bToA, seed ^ 0x5A5A5A5A5A5A5A5Aull);
    ImpairedTransportPair pair;
    pair.a = std::make_unique<ImpairedTransport>(link, laneA, std::move(raw.a));
    pair.b = std::make_unique<ImpairedTransport>(std::move(link), laneB, std::move(raw.b));
    return pair;
}

// Both directions of a fresh in-memory connection impaired by the same profile.
inline ImpairedTransportPair makeImpairedPair(const IWallClock& clock, const ImpairmentProfile& profile,
                                              std::uint64_t seed) {
    return makeImpairedPair(clock, profile, profile, seed);
}

}  // namespace wpiemu::sdk::testing
