#include <doctest/doctest.h>

#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/sdk/ipc/delivery.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;
using namespace wpiemu::sdk::testing;

namespace {

// A transport that only knows reliable sends, like every transport written before delivery classes existed.
class ReliableOnlyTransport final : public ITransport {
public:
    Status send(std::span<const std::byte> frame) override {
        sent.emplace_back(frame.begin(), frame.end());
        return okStatus();
    }
    Result<bool> receive(std::vector<std::byte>&, std::chrono::milliseconds) override { return false; }
    void close() override {}
    bool connected() const override { return true; }
    std::string describe() const override { return "reliable-only"; }

    std::vector<std::vector<std::byte>> sent;
};

// A transport with both channels that records which one was used.
class TwoChannelTransport final : public ITransport {
public:
    Status send(std::span<const std::byte>) override {
        ++reliable;
        return okStatus();
    }
    Status sendUnreliable(std::span<const std::byte>) override {
        ++unreliable;
        return okStatus();
    }
    Result<bool> receive(std::vector<std::byte>&, std::chrono::milliseconds) override { return false; }
    void close() override {}
    bool connected() const override { return true; }
    std::string describe() const override { return "two-channel"; }

    int reliable = 0;
    int unreliable = 0;
};

template <std::size_t... I>
std::vector<Message> oneOfEach(std::index_sequence<I...>) {
    return {Message{std::in_place_index<I>}...};
}

std::vector<std::byte> bytesOf(std::initializer_list<int> v) {
    std::vector<std::byte> out;
    for (int b : v) out.push_back(static_cast<std::byte>(b));
    return out;
}

}  // namespace

TEST_CASE("delivery: a transport that only implements send() carries unreliable frames reliably") {
    ReliableOnlyTransport transport;
    const auto frame = bytesOf({1, 2, 3});
    CHECK(transport.sendUnreliable(frame).ok());
    REQUIRE(transport.sent.size() == 1);
    CHECK(transport.sent[0] == frame);
}

TEST_CASE("delivery: the in-memory transport delivers unreliable frames in order") {
    InMemoryTransportPair pair = makeInMemoryTransportPair();
    CHECK(pair.a->send(bytesOf({1})).ok());
    CHECK(pair.a->sendUnreliable(bytesOf({2})).ok());
    CHECK(pair.a->send(bytesOf({3})).ok());
    std::vector<std::byte> frame;
    for (int expected = 1; expected <= 3; ++expected) {
        auto got = pair.b->receive(frame, std::chrono::milliseconds{0});
        REQUIRE(got.ok());
        REQUIRE(got.value());
        CHECK(frame == bytesOf({expected}));
    }
}

TEST_CASE("delivery: MessageChannel sends reliably unless told otherwise") {
    auto transport = std::make_unique<TwoChannelTransport>();
    TwoChannelTransport* raw = transport.get();
    MessageChannel channel(std::move(transport), std::make_shared<MemoryMessageCodec>());
    CHECK(channel.send(Goodbye{}).ok());
    CHECK(channel.send(Goodbye{}, Delivery::Reliable).ok());
    CHECK(raw->reliable == 2);
    CHECK(raw->unreliable == 0);
}

TEST_CASE("delivery: MessageChannel routes Delivery::Unreliable to sendUnreliable") {
    auto transport = std::make_unique<TwoChannelTransport>();
    TwoChannelTransport* raw = transport.get();
    MessageChannel channel(std::move(transport), std::make_shared<MemoryMessageCodec>());
    CHECK(channel.send(Goodbye{}, Delivery::Unreliable).ok());
    CHECK(channel.send(Goodbye{}, Delivery::Reliable).ok());
    CHECK(channel.send(Goodbye{}, Delivery::Unreliable).ok());
    CHECK(raw->reliable == 1);
    CHECK(raw->unreliable == 2);
}

TEST_CASE("delivery: an unreliable MessageChannel send over a reliable-only transport still arrives") {
    auto transport = std::make_unique<ReliableOnlyTransport>();
    ReliableOnlyTransport* raw = transport.get();
    MessageChannel channel(std::move(transport), std::make_shared<MemoryMessageCodec>());
    CHECK(channel.send(Goodbye{}, Delivery::Unreliable).ok());
    CHECK(raw->sent.size() == 1);
}

TEST_CASE("delivery: a failed encode sends nothing on either channel") {
    class FailingCodec final : public IMessageCodec {
    public:
        std::string_view name() const override { return "failing"; }
        Status encode(const Message&, std::vector<std::byte>&) const override {
            return makeError(ErrorCode::InvalidArgument, "no");
        }
        Result<Message> decode(std::span<const std::byte>) const override { return makeError(ErrorCode::ProtocolError, "no"); }
    };
    auto transport = std::make_unique<TwoChannelTransport>();
    TwoChannelTransport* raw = transport.get();
    MessageChannel channel(std::move(transport), std::make_shared<FailingCodec>());
    CHECK_FALSE(channel.send(Goodbye{}, Delivery::Unreliable).ok());
    CHECK_FALSE(channel.send(Goodbye{}).ok());
    CHECK(raw->reliable == 0);
    CHECK(raw->unreliable == 0);
}

TEST_CASE("delivery: every extension protocol message is reliable") {
    const std::vector<Message> all = oneOfEach(std::make_index_sequence<kExtensionMessageCount>{});
    REQUIRE(all.size() == kExtensionMessageCount);
    for (const Message& m : all) {
        CAPTURE(m.index());
        CHECK(deliveryOf(m) == Delivery::Reliable);
    }
}
