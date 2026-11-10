#pragma once

// Support for the cross-module tests that run the REAL stack on loopback sockets: a RelayServer configured as production
// (wr::parseRelayOptions + wr::transportOptionsFor, so the tests cannot diverge from wpiemu_relay), net::RelayHostTransport
// and net::RelayClientTransport peers, net::SecureTransport and real invites. A wire sniffer decorates the relay's ENet
// peers (it records every frame the relay reads or writes) and a log capture keeps everything the relay logs.
// Only public headers of sdk, ipc, net and relay are used.

#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/net/control_link.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"
#include "wpiemu/relay/relay_log.h"
#include "wpiemu/relay/relay_options.h"
#include "wpiemu/relay/relay_server.h"
#include "wpiemu/sdk/core/clock.h"

namespace relaystack {

using namespace std::chrono_literals;
namespace sdk = wpiemu::sdk;
namespace net = wpiemu::net;
namespace wr = wpiemu::relay;
namespace proto = wpiemu::sdk::proto;
using Bytes = std::vector<std::byte>;
using proto::Delivery;

inline Bytes bytesOf(const std::string& text) {
    Bytes out(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) out[i] = static_cast<std::byte>(text[i]);
    return out;
}

inline Bytes patterned(std::size_t size, std::uint32_t seed) {
    Bytes out(size);
    std::mt19937 rng(seed);
    for (auto& b : out) b = static_cast<std::byte>(rng() & 0xFF);
    return out;
}

inline bool containsBytes(const Bytes& haystack, std::span<const std::byte> needle) {
    return !needle.empty() && std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end()) != haystack.end();
}

inline std::string hexOf(std::span<const std::uint8_t> bytes, bool upper = false) {
    const char* digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    std::string out;
    for (const std::uint8_t b : bytes) {
        out.push_back(digits[b >> 4]);
        out.push_back(digits[b & 0xF]);
    }
    return out;
}

template <class Pred>
bool waitFor(Pred pred, std::chrono::milliseconds limit = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (!pred()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(5ms);
    }
    return true;
}

// ---- what the relay saw on the wire ------------------------------------------------------------------------------------

class WireCapture {
public:
    struct Frame {
        bool towardRelay = false;  // read by the relay (true) or written by it (false)
        Delivery delivery = Delivery::Reliable;
        Bytes bytes;
    };

    void game(bool towardRelay, Delivery delivery, std::span<const std::byte> bytes) {
        std::lock_guard lock(mutex_);
        game_.push_back({towardRelay, delivery, Bytes(bytes.begin(), bytes.end())});
    }
    void control(bool towardRelay, std::span<const std::byte> bytes) {
        std::lock_guard lock(mutex_);
        control_.push_back({towardRelay, Delivery::Reliable, Bytes(bytes.begin(), bytes.end())});
    }

    bool gameContains(std::span<const std::byte> needle) const {
        std::lock_guard lock(mutex_);
        return std::any_of(game_.begin(), game_.end(), [&](const Frame& f) { return containsBytes(f.bytes, needle); });
    }
    bool anyContains(std::span<const std::byte> needle) const {
        std::lock_guard lock(mutex_);
        const auto has = [&](const Frame& f) { return containsBytes(f.bytes, needle); };
        return std::any_of(game_.begin(), game_.end(), has) || std::any_of(control_.begin(), control_.end(), has);
    }
    // A game frame of exactly `size` bytes read by the relay (towardRelay) or written by it.
    bool hasGameFrameOfSize(std::size_t size, bool towardRelay) const {
        std::lock_guard lock(mutex_);
        return std::any_of(game_.begin(), game_.end(),
                           [&](const Frame& f) { return f.towardRelay == towardRelay && f.bytes.size() == size; });
    }
    std::size_t gameFrames() const {
        std::lock_guard lock(mutex_);
        return game_.size();
    }

private:
    mutable std::mutex mutex_;
    std::vector<Frame> game_;
    std::vector<Frame> control_;
};

// An ENet peer of the relay that records the frames passing through it. It implements IControlLink like the real one, since
// the relay looks for it on every accepted transport.
class SniffedPeer final : public proto::ITransport, public net::IControlLink {
public:
    SniffedPeer(std::unique_ptr<proto::ITransport> inner, std::shared_ptr<WireCapture> capture)
        : inner_(std::move(inner)), control_(net::controlLinkOf(*inner_)), capture_(std::move(capture)) {}

    sdk::Status send(std::span<const std::byte> frame) override {
        capture_->game(false, Delivery::Reliable, frame);
        return inner_->send(frame);
    }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override {
        capture_->game(false, Delivery::Unreliable, frame);
        return inner_->sendUnreliable(frame);
    }
    sdk::Result<bool> receive(Bytes& frame, std::chrono::milliseconds timeout) override { return inner_->receive(frame, timeout); }
    void close() override { inner_->close(); }
    bool connected() const override { return inner_->connected(); }
    std::string describe() const override { return inner_->describe(); }
    std::optional<proto::LinkStats> linkStats() const override { return inner_->linkStats(); }

    sdk::Status sendControl(std::span<const std::byte> frame) override {
        capture_->control(false, frame);
        return control_->sendControl(frame);
    }
    sdk::Result<bool> receiveControl(Bytes& frame, std::chrono::milliseconds timeout) override {
        auto got = control_->receiveControl(frame, timeout);
        if (got && got.value()) capture_->control(true, frame);
        return got;
    }
    sdk::Result<bool> receiveGame(Bytes& frame, Delivery& delivery, std::chrono::milliseconds timeout) override {
        auto got = control_->receiveGame(frame, delivery, timeout);
        if (got && got.value()) capture_->game(true, delivery, frame);
        return got;
    }

private:
    std::unique_ptr<proto::ITransport> inner_;
    net::IControlLink* control_;
    std::shared_ptr<WireCapture> capture_;
};

class SniffServer final : public proto::ITransportServer {
public:
    SniffServer(std::unique_ptr<proto::ITransportServer> inner, std::shared_ptr<WireCapture> capture)
        : inner_(std::move(inner)), capture_(std::move(capture)) {}

    sdk::Status listen() override { return inner_->listen(); }
    std::string endpoint() const override { return inner_->endpoint(); }
    sdk::Result<std::unique_ptr<proto::ITransport>> accept(std::chrono::milliseconds timeout) override {
        auto accepted = inner_->accept(timeout);
        if (!accepted) return accepted.error();
        if (accepted.value() == nullptr) return std::unique_ptr<proto::ITransport>();
        return std::unique_ptr<proto::ITransport>(std::make_unique<SniffedPeer>(std::move(accepted).value(), capture_));
    }
    void close() override { inner_->close(); }

private:
    std::unique_ptr<proto::ITransportServer> inner_;
    std::shared_ptr<WireCapture> capture_;
};

class LogCapture {
public:
    void add(const std::string& line) {
        std::lock_guard lock(mutex_);
        lines_.push_back(line);
    }
    bool contains(const std::string& text) const {
        std::lock_guard lock(mutex_);
        return std::any_of(lines_.begin(), lines_.end(), [&](const std::string& l) { return l.find(text) != std::string::npos; });
    }
    std::size_t count() const {
        std::lock_guard lock(mutex_);
        return lines_.size();
    }

private:
    mutable std::mutex mutex_;
    std::vector<std::string> lines_;
};

// ---- the relay ---------------------------------------------------------------------------------------------------------

class StackRelay {
public:
    using Tweak = std::function<void(wr::RelayOptions&)>;

    // What `wpiemu_relay --port 1` parses to (production defaults, the default dual-stack bind), with an ephemeral port.
    static wr::RelayOptions productionOptions() {
        const auto none = [](const std::string&) -> std::optional<std::string> { return std::nullopt; };
        auto parsed = wr::parseRelayOptions({"--port", "1"}, none);
        REQUIRE(parsed.ok());
        wr::RelayOptions options = std::move(parsed).value();
        options.port = 0;
        options.logFormat = wr::LogFormat::Json;
        return options;
    }

    // `port` != 0 rebinds a port an earlier relay used (a restart).
    explicit StackRelay(const Tweak& tweak = {}, std::uint16_t port = 0) {
        options_ = productionOptions();
        if (port != 0) options_.port = port;
        if (tweak) tweak(options_);
        auto sink = logs;
        auto log = std::make_shared<wr::RelayLog>(
            wr::LogFormat::Json, [sink](const std::string& line) { sink->add(line); }, nullptr, wr::LogLevel::Debug);
        auto transport = std::make_unique<SniffServer>(std::make_unique<net::EnetServerTransport>(wr::transportOptionsFor(options_)), capture);
        server_ = std::make_unique<wr::RelayServer>(std::move(transport), options_.server, std::make_shared<sdk::SteadyWallClock>(), log);
        const sdk::Status started = server_->start();
        startError_ = started.ok() ? std::string() : started.error().message;
        if (!started.ok()) return;
        port_ = net::NetEndpoint::parse(server_->endpoint()).value().port;
        std::promise<sdk::Status> promise;
        result_ = promise.get_future();
        thread_ = std::thread([this, p = std::move(promise)]() mutable { p.set_value(server_->run()); });
    }
    ~StackRelay() { stop(); }
    StackRelay(const StackRelay&) = delete;
    StackRelay& operator=(const StackRelay&) = delete;

    bool started() const { return startError_.empty(); }
    const std::string& startError() const { return startError_; }
    std::uint16_t port() const { return port_; }
    std::string host() const { return "127.0.0.1"; }
    std::string endpoint(const std::string& room) const { return "relay://127.0.0.1:" + std::to_string(port_) + "/" + room; }
    wr::RelayStats stats() const { return server_->stats(); }
    const wr::RelayOptions& options() const { return options_; }

    bool stop() {
        if (!thread_.joinable()) return true;
        server_->stop();
        const bool finished = result_.wait_for(5s) == std::future_status::ready;
        thread_.join();
        return finished;
    }

    std::shared_ptr<WireCapture> capture = std::make_shared<WireCapture>();
    std::shared_ptr<LogCapture> logs = std::make_shared<LogCapture>();

private:
    wr::RelayOptions options_;
    std::unique_ptr<wr::RelayServer> server_;
    std::thread thread_;
    std::future<sdk::Status> result_;
    std::uint16_t port_ = 0;
    std::string startError_;
};

// ---- invites, hosts and clients ----------------------------------------------------------------------------------------

struct Invite {
    net::InviteSecret secret;
    std::string room;
};

inline Invite newInvite() {
    auto generated = net::generateInvite();
    REQUIRE(generated.ok());
    net::InviteSecret secret = std::move(generated).value();
    std::string room = net::roomId(secret);
    return Invite{std::move(secret), std::move(room)};
}

inline net::RelayHostOptions hostOptions(const StackRelay& relay, const net::InviteSecret& secret) {
    net::RelayHostOptions options = net::relayHostOptionsFor(relay.host(), relay.port(), secret);
    options.connectTimeout = 3s;
    options.registerTimeout = 3s;
    options.reconnectBackoffInitial = 50ms;
    options.reconnectBackoffMax = 200ms;
    options.reconnectGiveUp = 20s;
    return options;
}

inline std::unique_ptr<net::RelayHostTransport> startHost(const StackRelay& relay, const net::InviteSecret& secret) {
    auto host = std::make_unique<net::RelayHostTransport>(hostOptions(relay, secret));
    const sdk::Status listening = host->listen();
    REQUIRE_MESSAGE(listening.ok(), (listening.ok() ? "" : listening.error().message));
    return host;
}

inline sdk::Result<std::unique_ptr<proto::ITransport>> joinRaw(const StackRelay& relay, const std::string& room,
                                                              net::RelayClientOptions options = {}) {
    net::RelayClientTransport client(std::move(options));
    return client.connect(relay.endpoint(room), 5s);
}

inline std::unique_ptr<proto::ITransport> acceptPeer(net::RelayHostTransport& host, std::chrono::milliseconds timeout = 5s) {
    auto accepted = host.accept(timeout);
    REQUIRE_MESSAGE(accepted.ok(), (accepted.ok() ? "" : accepted.error().message));
    return std::move(accepted).value();
}

// A client and the host's virtual peer for it, both through the encryption handshake (or the failure of it).
struct SecureSession {
    std::unique_ptr<proto::ITransport> client;
    std::unique_ptr<proto::ITransport> peer;
    std::string clientError;
    std::string hostError;
    bool ok() const { return client != nullptr && peer != nullptr; }
};

// Joins `room` as a client holding `clientSecret` while the host side accepts with `hostSecret`; the two handshake halves
// run concurrently.
inline SecureSession joinSecure(const StackRelay& relay, net::RelayHostTransport& host, const std::string& room,
                                const net::InviteSecret& clientSecret, const net::InviteSecret& hostSecret,
                                std::chrono::milliseconds handshake = 5s) {
    SecureSession session;
    auto clientSide = std::async(std::launch::async, [&]() -> std::pair<std::unique_ptr<proto::ITransport>, std::string> {
        net::RelayClientTransport client;
        auto connected = client.connect(relay.endpoint(room), 5s);
        if (!connected) return {nullptr, connected.error().message};
        auto secured = net::SecureTransport::connectAsClient(std::move(connected).value(), clientSecret, handshake);
        if (!secured) return {nullptr, secured.error().message};
        return std::pair<std::unique_ptr<proto::ITransport>, std::string>(std::move(secured).value(), std::string());
    });
    auto accepted = host.accept(5s);
    if (!accepted) session.hostError = accepted.error().message;
    else if (accepted.value() == nullptr) session.hostError = "no client reached the host";
    else {
        auto secured = net::SecureTransport::acceptAsHost(std::move(accepted).value(), hostSecret, handshake);
        if (!secured) session.hostError = secured.error().message;
        else session.peer = std::move(secured).value();
    }
    auto clientResult = clientSide.get();
    session.client = std::move(clientResult.first);
    session.clientError = std::move(clientResult.second);
    return session;
}

// ---- frames ------------------------------------------------------------------------------------------------------------

// True when `expected` arrives before the deadline; other frames (late unreliable duplicates) are skipped.
inline bool receiveExact(proto::ITransport& transport, const Bytes& expected, std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Bytes frame;
    while (std::chrono::steady_clock::now() < deadline) {
        auto got = transport.receive(frame, std::min<std::chrono::milliseconds>(50ms, proto::timeLeft(deadline)));
        if (!got) return false;
        if (got.value() && frame == expected) return true;
    }
    return false;
}

inline sdk::ErrorCode receiveError(proto::ITransport& transport, std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Bytes frame;
    while (std::chrono::steady_clock::now() < deadline) {
        auto got = transport.receive(frame, std::min<std::chrono::milliseconds>(50ms, proto::timeLeft(deadline)));
        if (!got) return got.error().code;
    }
    return sdk::ErrorCode::Internal;
}

inline bool reliableRoundTrip(proto::ITransport& from, proto::ITransport& to, const Bytes& payload, std::chrono::milliseconds timeout = 10s) {
    if (!from.send(payload)) return false;
    return receiveExact(to, payload, timeout);
}

// An unreliable frame may in principle be lost: resend until it arrives (a loopback delivers the first one).
inline bool unreliableRoundTrip(proto::ITransport& from, proto::ITransport& to, const Bytes& payload, int attempts = 40) {
    for (int i = 0; i < attempts; ++i) {
        if (!from.sendUnreliable(payload)) return false;
        if (receiveExact(to, payload, 150ms)) return true;
    }
    return false;
}

}  // namespace relaystack
