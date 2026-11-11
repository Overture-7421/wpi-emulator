#pragma once

// Test doubles for the relay: an in-memory transport server whose peers implement ITransport AND net::IControlLink (what the
// ENet peers offer), a harness that runs a RelayServer on them with a manual clock, and helpers to read effects.
// Nothing here sleeps or touches a socket.

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/control_link.h"
#include "wpiemu/relay/relay_server.h"
#include "wpiemu/relay/room_table.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

namespace relaytest {

using Bytes = std::vector<std::byte>;
namespace rl = wpiemu::sdk::match::relay;
using wpiemu::sdk::proto::Delivery;
using wpiemu::sdk::proto::Message;

inline Bytes bytesOf(const std::string& text) {
    Bytes out(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) out[i] = static_cast<std::byte>(text[i]);
    return out;
}

inline const wpiemu::ipc::BinaryMessageCodec& codec() {
    static const wpiemu::ipc::BinaryMessageCodec instance(wpiemu::net::kMaxControlFrameBytes);
    return instance;
}

inline Bytes encoded(const Message& message) {
    Bytes out;
    (void)codec().encode(message, out);
    return out;
}

// A valid room id derived from a number (distinct numbers give distinct ids, none is the reserved health-ping room).
inline std::string roomIdFor(unsigned n) {
    std::string id(rl::kRelayRoomIdChars, '0');
    const std::string alphabet = rl::kRoomIdAlphabet;
    ++n;
    for (std::size_t i = 0; i < id.size(); ++i) {
        id[id.size() - 1 - i] = alphabet[n % alphabet.size()];
        n /= static_cast<unsigned>(alphabet.size());
    }
    return id;
}

inline rl::HostAuth authOf(std::uint8_t seed) {
    rl::HostAuth auth{};
    for (std::size_t i = 0; i < auth.size(); ++i) auth[i] = static_cast<std::uint8_t>(seed + i);
    return auth;
}

inline Message registerMessage(const std::string& room, std::uint8_t authSeed = 1,
                               std::uint32_t version = rl::kRelayProtocolVersion) {
    rl::RegisterRoom m;
    m.protocolVersion = version;
    m.roomId = room;
    m.hostAuth = authOf(authSeed);
    return m;
}

inline Message joinMessage(const std::string& room, std::uint32_t version = rl::kRelayProtocolVersion) {
    rl::JoinRoom m;
    m.protocolVersion = version;
    m.roomId = room;
    return m;
}

// ---- effects ----
inline std::vector<Message> controlsTo(const wpiemu::relay::Effects& effects, wpiemu::relay::ConnId to) {
    std::vector<Message> out;
    for (const auto& e : effects)
        if (const auto* c = std::get_if<wpiemu::relay::SendControl>(&e); c != nullptr && c->to == to) out.push_back(c->message);
    return out;
}

inline std::vector<wpiemu::relay::SendGame> gamesIn(const wpiemu::relay::Effects& effects) {
    std::vector<wpiemu::relay::SendGame> out;
    for (const auto& e : effects)
        if (const auto* g = std::get_if<wpiemu::relay::SendGame>(&e)) out.push_back(*g);
    return out;
}

inline std::vector<wpiemu::relay::CloseConnection> closesIn(const wpiemu::relay::Effects& effects) {
    std::vector<wpiemu::relay::CloseConnection> out;
    for (const auto& e : effects)
        if (const auto* c = std::get_if<wpiemu::relay::CloseConnection>(&e)) out.push_back(*c);
    return out;
}

template <class T>
std::optional<T> firstOf(const std::vector<Message>& messages) {
    for (const Message& m : messages)
        if (const auto* v = std::get_if<T>(&m)) return *v;
    return std::nullopt;
}

// ---- fake network ----
struct GameFrame {
    Delivery delivery = Delivery::Reliable;
    Bytes bytes;
};

// The remote end of one connection, as the test sees it.
struct FakePeer {
    std::string address = "enet://10.0.0.1:5000";
    std::deque<Bytes> controlIn;      // the remote sent these on channel 0
    std::deque<GameFrame> gameIn;     // ... and these on channels 1 and 2
    std::vector<Bytes> controlOut;    // the relay sent these on channel 0
    std::vector<GameFrame> gameOut;   // ... and these on the game channels
    bool remoteClosed = false;
    bool closedByRelay = false;
    bool failSends = false;

    void sendControl(const Message& message) { controlIn.push_back(encoded(message)); }
    void sendControlRaw(Bytes frame) { controlIn.push_back(std::move(frame)); }
    void sendGame(Delivery delivery, Bytes frame) { gameIn.push_back({delivery, std::move(frame)}); }
    void disconnect() { remoteClosed = true; }

    std::vector<Message> takeControl() {
        std::vector<Message> out;
        for (const Bytes& frame : controlOut) {
            auto decoded = codec().decode(frame);
            if (decoded) out.push_back(std::move(decoded).value());
        }
        controlOut.clear();
        return out;
    }
    std::vector<GameFrame> takeGame() { return std::exchange(gameOut, {}); }
};

class FakeTransport final : public wpiemu::sdk::proto::ITransport, public wpiemu::net::IControlLink {
public:
    explicit FakeTransport(std::shared_ptr<FakePeer> peer) : peer_(std::move(peer)) {}

    wpiemu::sdk::Status send(std::span<const std::byte> frame) override { return record(Delivery::Reliable, frame); }
    wpiemu::sdk::Status sendUnreliable(std::span<const std::byte> frame) override { return record(Delivery::Unreliable, frame); }
    wpiemu::sdk::Result<bool> receive(Bytes& frame, std::chrono::milliseconds) override {
        Delivery ignored;
        return receiveGame(frame, ignored, {});
    }
    void close() override { peer_->closedByRelay = true; }
    bool connected() const override { return !peer_->closedByRelay && !peer_->remoteClosed; }
    std::string describe() const override { return peer_->address; }

    wpiemu::sdk::Status sendControl(std::span<const std::byte> frame) override {
        if (peer_->closedByRelay || peer_->remoteClosed || peer_->failSends) return gone();
        peer_->controlOut.emplace_back(frame.begin(), frame.end());
        return wpiemu::sdk::okStatus();
    }
    wpiemu::sdk::Result<bool> receiveControl(Bytes& frame, std::chrono::milliseconds) override {
        if (!peer_->controlIn.empty() && !peer_->closedByRelay) {
            frame = std::move(peer_->controlIn.front());
            peer_->controlIn.pop_front();
            return true;
        }
        if (peer_->remoteClosed || peer_->closedByRelay) return gone();
        return false;
    }
    wpiemu::sdk::Result<bool> receiveGame(Bytes& frame, Delivery& delivery, std::chrono::milliseconds) override {
        if (!peer_->gameIn.empty() && !peer_->closedByRelay) {
            frame = std::move(peer_->gameIn.front().bytes);
            delivery = peer_->gameIn.front().delivery;
            peer_->gameIn.pop_front();
            return true;
        }
        if (peer_->remoteClosed || peer_->closedByRelay) return gone();
        return false;
    }

private:
    static wpiemu::sdk::Error gone() { return wpiemu::sdk::makeError(wpiemu::sdk::ErrorCode::Disconnected, "fake peer is gone"); }
    wpiemu::sdk::Status record(Delivery delivery, std::span<const std::byte> frame) {
        if (peer_->closedByRelay || peer_->remoteClosed || peer_->failSends) return gone();
        peer_->gameOut.push_back({delivery, Bytes(frame.begin(), frame.end())});
        return wpiemu::sdk::okStatus();
    }

    std::shared_ptr<FakePeer> peer_;
};

struct FakeNetwork {
    std::deque<std::shared_ptr<FakePeer>> pending;
    std::vector<std::shared_ptr<FakePeer>> all;
    bool closed = false;
    bool listening = false;

    std::shared_ptr<FakePeer> connect(const std::string& ip = "10.0.0.1", unsigned port = 5000) {
        auto peer = std::make_shared<FakePeer>();
        peer->address = ip.empty() ? "enet://?" : "enet://" + ip + ":" + std::to_string(port);
        pending.push_back(peer);
        all.push_back(peer);
        return peer;
    }
};

class FakeServer final : public wpiemu::sdk::proto::ITransportServer {
public:
    explicit FakeServer(std::shared_ptr<FakeNetwork> network) : network_(std::move(network)) {}
    wpiemu::sdk::Status listen() override {
        network_->listening = true;
        return wpiemu::sdk::okStatus();
    }
    std::string endpoint() const override { return "enet://127.0.0.1:1"; }
    wpiemu::sdk::Result<std::unique_ptr<wpiemu::sdk::proto::ITransport>> accept(std::chrono::milliseconds) override {
        if (network_->closed) return wpiemu::sdk::makeError(wpiemu::sdk::ErrorCode::Disconnected, "closed");
        if (network_->pending.empty()) return std::unique_ptr<wpiemu::sdk::proto::ITransport>();
        auto peer = network_->pending.front();
        network_->pending.pop_front();
        return std::unique_ptr<wpiemu::sdk::proto::ITransport>(std::make_unique<FakeTransport>(peer));
    }
    void close() override { network_->closed = true; }

private:
    std::shared_ptr<FakeNetwork> network_;
};

// A RelayServer on a fake network with a manual clock; log lines are captured.
struct Harness {
    std::shared_ptr<wpiemu::sdk::testing::ManualWallClock> clock = std::make_shared<wpiemu::sdk::testing::ManualWallClock>();
    std::shared_ptr<FakeNetwork> network = std::make_shared<FakeNetwork>();
    std::shared_ptr<std::vector<std::string>> logLines = std::make_shared<std::vector<std::string>>();
    std::unique_ptr<wpiemu::relay::RelayServer> server;

    explicit Harness(wpiemu::relay::RelayServerConfig config = {}) {
        auto lines = logLines;
        auto log = std::make_shared<wpiemu::relay::RelayLog>(
            wpiemu::relay::LogFormat::Json, [lines](const std::string& line) { lines->push_back(line); }, [] { return std::string("T"); },
            wpiemu::relay::LogLevel::Debug);
        server = std::make_unique<wpiemu::relay::RelayServer>(std::make_unique<FakeServer>(network), std::move(config), clock, log);
        (void)server->start();
    }

    // Polls until a pass finds nothing to do.
    void pump() {
        for (int i = 0; i < 50 && server->poll(); ++i) {
        }
    }
    void advance(std::chrono::milliseconds amount) {
        clock->advance(amount);
        pump();
    }
    std::shared_ptr<FakePeer> connect(const std::string& ip = "10.0.0.1", unsigned port = 5000) {
        auto peer = network->connect(ip, port);
        pump();
        return peer;
    }
    // Connects, registers as host of `room`, returns the peer after the reply was read.
    std::shared_ptr<FakePeer> host(const std::string& room, std::uint8_t authSeed = 1, const std::string& ip = "10.0.0.1",
                                   unsigned port = 5000) {
        auto peer = connect(ip, port);
        peer->sendControl(registerMessage(room, authSeed));
        pump();
        return peer;
    }
    std::shared_ptr<FakePeer> client(const std::string& room, const std::string& ip = "10.0.0.2", unsigned port = 6000) {
        auto peer = connect(ip, port);
        peer->sendControl(joinMessage(room));
        pump();
        return peer;
    }
    bool logContains(const std::string& text) const {
        for (const std::string& line : *logLines)
            if (line.find(text) != std::string::npos) return true;
        return false;
    }
};

}  // namespace relaytest
