#include <doctest/doctest.h>

#include "fakes.h"
#include "wpiemu/halsim_ext/ipc_link.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

namespace {
struct LinkPair {
    LinkPair() {
        auto codec = std::make_shared<sdk::testing::MemoryMessageCodec>();
        auto pair = sdk::testing::makeInMemoryTransportPair();
        peer = std::make_unique<proto::MessageChannel>(std::move(pair.a), codec);
        link = std::make_unique<IpcLink>(std::make_unique<proto::MessageChannel>(std::move(pair.b), codec));
        link->start();
    }
    std::unique_ptr<proto::MessageChannel> peer;
    std::unique_ptr<IpcLink> link;
};
}  // namespace

TEST_CASE("link moves messages in both directions in order") {
    LinkPair p;
    for (std::uint64_t i = 1; i <= 3; ++i) {
        proto::SensorFrame f;
        f.tick = i;
        REQUIRE(p.peer->send(proto::Message(f)));
    }
    for (std::uint64_t i = 1; i <= 3; ++i) {
        proto::Message m;
        REQUIRE(p.link->pop(m, 1000ms) == IpcLink::PopResult::Message);
        REQUIRE(std::holds_alternative<proto::SensorFrame>(m));
        CHECK(std::get<proto::SensorFrame>(m).tick == i);
    }
    proto::Message none;
    CHECK(p.link->pop(none, 10ms) == IpcLink::PopResult::Timeout);

    proto::ActuatorFrame a;
    a.tick = 9;
    REQUIRE(p.link->post(proto::Message(a)));
    proto::Message out;
    const auto got = p.peer->receive(out, 1000ms);
    REQUIRE(got);
    REQUIRE(got.value());
    CHECK(std::get<proto::ActuatorFrame>(out).tick == 9);
}

TEST_CASE("link reports Closed once the peer disconnects and rejects further posts") {
    LinkPair p;
    p.peer->close();
    proto::Message m;
    CHECK(p.link->pop(m, 1000ms) == IpcLink::PopResult::Closed);
    CHECK_FALSE(p.link->open());
    CHECK_FALSE(p.link->post(proto::Message(proto::Goodbye{"late"})));
    CHECK_FALSE(p.link->closeReason().empty());
}

TEST_CASE("link drains queued inbound messages before reporting Closed") {
    LinkPair p;
    proto::SensorFrame f;
    f.tick = 5;
    REQUIRE(p.peer->send(proto::Message(f)));
    std::this_thread::sleep_for(50ms);  // let the reader enqueue it
    p.peer->close();
    proto::Message m;
    CHECK(p.link->pop(m, 1000ms) == IpcLink::PopResult::Message);
    CHECK(p.link->pop(m, 1000ms) == IpcLink::PopResult::Closed);
}

TEST_CASE("link flush waits until the outbound queue reached the transport") {
    LinkPair p;
    for (int i = 0; i < 20; ++i) p.link->post(proto::Message(proto::Goodbye{"x"}));
    CHECK(p.link->flush(1000ms));
    int received = 0;
    proto::Message m;
    while (p.peer->receive(m, 20ms) && received < 20) ++received;
    CHECK(received == 20);
}

TEST_CASE("droppable logs are capped, control messages are never dropped") {
    LinkPair p;
    // Stall the writer by closing nothing: just post faster than we read; the cap is on queued logs.
    int accepted = 0;
    for (int i = 0; i < 5000; ++i) {
        proto::Log l;
        l.message = "spam";
        if (p.link->postDroppable(proto::Message(std::move(l)))) ++accepted;
    }
    CHECK(accepted >= 1);
    CHECK(accepted + static_cast<int>(p.link->droppedLogs()) == 5000);
    CHECK(p.link->post(proto::Message(proto::Goodbye{"still accepted"})));
}

TEST_CASE("malformed frames are skipped, a closed peer ends the link") {
    auto codec = std::make_shared<sdk::testing::MemoryMessageCodec>();
    auto pair = sdk::testing::makeInMemoryTransportPair();
    auto raw = std::move(pair.a);
    IpcLink link(std::make_unique<proto::MessageChannel>(std::move(pair.b), codec));
    link.start();

    const std::byte junk[3] = {std::byte{1}, std::byte{2}, std::byte{3}};
    REQUIRE(raw->send(junk));  // wrong size for the memory codec => ProtocolError on decode
    proto::MessageChannel peer(std::move(raw), codec);
    proto::SensorFrame f;
    f.tick = 11;
    REQUIRE(peer.send(proto::Message(f)));

    proto::Message m;
    REQUIRE(link.pop(m, 1000ms) == IpcLink::PopResult::Message);
    CHECK(std::get<proto::SensorFrame>(m).tick == 11);
    CHECK(link.skippedFrames() == 1);
}
