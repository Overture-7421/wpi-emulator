#pragma once

// A game client of a hosted practice match for tests: a real transport (ENet direct or the relay), the SecureTransport
// handshake with the invite, and the match protocol (join, drive, snapshots) on top of it. Header-only, no test framework.
// Used by the match tests (tests/match) and the app tests (the real runtime host hosting a match).

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::app::host::testing {

// True when an ENet server can listen on `host`:`port` right now.
inline bool udpPortIsFree(const std::string& host, std::uint16_t port) {
    net::EnetServerOptions options;
    options.bindHost = host;
    options.port = port;
    net::EnetServerTransport server(options);
    return server.listen().ok();
}

class HostingTestClient {
public:
    // enet://host:port or relay://host:port/ROOM, then the handshake with `secret`. nullptr with `error` set on failure.
    static std::unique_ptr<HostingTestClient> connect(const std::string& endpoint, const net::InviteSecret& secret,
                                                      std::string* error = nullptr) {
        auto raw = openRaw(endpoint, error);
        if (!raw) return nullptr;
        auto secured = net::SecureTransport::connectAsClient(std::move(raw), secret, std::chrono::seconds(3));
        if (!secured) return fail(error, secured.error().message);
        return std::unique_ptr<HostingTestClient>(new HostingTestClient(std::move(secured).value()));
    }

    // The same with the invite as the host displays it.
    static std::unique_ptr<HostingTestClient> connect(const std::string& endpoint, const std::string& inviteText,
                                                      std::string* error = nullptr) {
        auto secret = net::parseInvite(inviteText);
        if (!secret) return fail(error, secret.error().message);
        return connect(endpoint, secret.value(), error);
    }

    // A connection that skips the handshake and speaks the game protocol in the clear.
    static std::unique_ptr<HostingTestClient> connectPlain(const std::string& endpoint, std::string* error = nullptr) {
        auto raw = openRaw(endpoint, error);
        if (!raw) return nullptr;
        return std::unique_ptr<HostingTestClient>(new HostingTestClient(std::move(raw)));
    }

    bool send(const sdk::proto::Message& m) { return channel_->send(m, sdk::proto::deliveryOf(m)).ok(); }

    // Reads whatever arrives within `timeout`.
    void pump(std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            sdk::proto::Message m;
            const auto left = std::max(std::chrono::milliseconds(0),
                                       std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()));
            auto got = channel_->receive(m, left);
            if (!got) {
                closed = true;
                return;
            }
            if (!got.value()) return;
            if (const auto* a = std::get_if<sdk::match::JoinAccept>(&m)) accept = *a;
            if (const auto* s = std::get_if<sdk::match::MatchSnapshot>(&m)) lastSnapshot = *s;
            log.push_back(std::move(m));
        }
    }

    template <class Pred>
    bool waitFor(Pred pred, std::chrono::milliseconds limit = std::chrono::seconds(5)) {
        const auto deadline = std::chrono::steady_clock::now() + limit;
        while (!pred()) {
            if (std::chrono::steady_clock::now() >= deadline || closed) return pred();
            pump(std::chrono::milliseconds(20));
        }
        return true;
    }

    template <class T>
    std::size_t count() const {
        std::size_t n = 0;
        for (const sdk::proto::Message& m : log)
            if (std::holds_alternative<T>(m)) ++n;
        return n;
    }
    bool gotControl(sdk::match::MatchControlKind kind) const {
        for (const sdk::proto::Message& m : log)
            if (const auto* c = std::get_if<sdk::match::MatchControl>(&m); c != nullptr && c->kind == kind) return true;
        return false;
    }
    std::optional<sdk::match::JoinReject> rejection() const {
        for (const sdk::proto::Message& m : log)
            if (const auto* r = std::get_if<sdk::match::JoinReject>(&m)) return *r;
        return std::nullopt;
    }

    // Sends a Join (the plugin and season of the host) and waits for the JoinAccept or a JoinReject.
    bool join(const std::string& name, const std::string& robotSpecJson, const std::string& gamePluginId = {}, int wpilibYear = 0) {
        sdk::match::Join j;
        j.displayName = name;
        j.gamePluginId = gamePluginId;
        j.wpilibYear = wpilibYear;
        j.robotSpecJson = robotSpecJson;
        if (!send(j)) return false;
        return waitFor([&] { return accept.has_value() || count<sdk::match::JoinReject>() > 0; }) && accept.has_value();
    }

    // Sends the next input (with the two before it): every wheel at `wheelSpeed` rad/s, steering straight.
    void drive(float wheelSpeed, std::size_t wheels = 4) {
        sdk::match::MatchInput in;
        in.inputSeq = ++seq_;
        in.targetTick = lastSnapshot ? lastSnapshot->tick : 0;
        in.wheels.assign(wheels, sdk::match::WheelInput{wheelSpeed, 0});
        recent_.push_back(std::move(in));
        while (recent_.size() > sdk::match::kInputRedundancy) recent_.pop_front();
        sdk::match::MatchInputPacket p;
        p.sessionId = accept->sessionId;
        p.runEpoch = accept->runEpoch;
        p.inputs.assign(recent_.begin(), recent_.end());
        send(p);
    }

    std::optional<sdk::match::RobotWireState> myRobot() const {
        if (!lastSnapshot || !accept) return std::nullopt;
        for (const sdk::match::RobotWireState& r : lastSnapshot->robots)
            if (r.participantId == accept->participantId) return r;
        return std::nullopt;
    }

    std::vector<sdk::proto::Message> log;
    std::optional<sdk::match::JoinAccept> accept;
    std::optional<sdk::match::MatchSnapshot> lastSnapshot;
    bool closed = false;

private:
    static std::unique_ptr<HostingTestClient> fail(std::string* error, const std::string& message) {
        if (error != nullptr) *error = message;
        return nullptr;
    }

    static std::unique_ptr<sdk::proto::ITransport> openRaw(const std::string& endpoint, std::string* error) {
        const auto timeout = std::chrono::seconds(5);
        if (endpoint.rfind("relay://", 0) == 0) {
            net::RelayClientTransport relay;
            auto connected = relay.connect(endpoint, timeout);
            if (!connected) {
                if (error != nullptr) *error = connected.error().message;
                return nullptr;
            }
            return std::move(connected).value();
        }
        net::EnetClientTransport enet;
        auto connected = enet.connect(endpoint, timeout);
        if (!connected) {
            if (error != nullptr) *error = connected.error().message;
            return nullptr;
        }
        return std::move(connected).value();
    }

    explicit HostingTestClient(std::unique_ptr<sdk::proto::ITransport> transport)
        : channel_(std::make_unique<sdk::proto::MessageChannel>(std::move(transport), std::make_shared<ipc::BinaryMessageCodec>())) {}

    std::unique_ptr<sdk::proto::MessageChannel> channel_;
    std::deque<sdk::match::MatchInput> recent_;
    std::uint32_t seq_ = 0;
};

}  // namespace wpiemu::app::host::testing
