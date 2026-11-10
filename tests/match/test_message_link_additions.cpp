#include <atomic>

#include "match_test_support.h"

using namespace match_test;
using wpiemu::core::runtime::ChannelMessageLink;
using wpiemu::core::runtime::InboundEvent;
using wpiemu::core::runtime::MessageInbox;

namespace {

struct LinkRig {
    MessageInbox inbox;
    std::shared_ptr<ipc::BinaryMessageCodec> codec = std::make_shared<ipc::BinaryMessageCodec>();
    testing::ImpairedTransportPair pair;
    ScriptedTransport* hostEnd = nullptr;
    std::shared_ptr<ChannelMessageLink> link;
    std::unique_ptr<proto::MessageChannel> client;
    testing::ManualWallClock clock;

    explicit LinkRig(std::size_t malformedLimit = 0) {
        pair = testing::makeImpairedPair(clock, {}, 1);
        auto scripted = std::make_unique<ScriptedTransport>(std::move(pair.a));
        hostEnd = scripted.get();
        link = std::make_shared<ChannelMessageLink>(1, std::move(scripted), codec, inbox);
        link->setMalformedFrameLimit(malformedLimit);
        link->start();
        client = std::make_unique<proto::MessageChannel>(std::move(pair.b), codec);
    }
    ~LinkRig() { link->shutdown(); }

    bool nextEvent(InboundEvent& ev) { return inbox.waitPop(ev, 2s); }
};

}  // namespace

TEST_CASE("message link: sends go out on the delivery class of the message") {
    LinkRig rig;
    wire::MatchSnapshot snapshot;
    REQUIRE(rig.link->send(snapshot).ok());
    wire::JoinReject reject;
    REQUIRE(rig.link->send(reject).ok());
    wire::MatchControl control;
    REQUIRE(rig.link->send(control).ok());
    CHECK(rig.hostEnd->unreliableSends.load() == 1);   // only the snapshot
    CHECK(rig.hostEnd->sends.load() == 3);

    proto::SensorFrame frame;   // every HALSim message stays reliable
    REQUIRE(rig.link->send(frame).ok());
    CHECK(rig.hostEnd->unreliableSends.load() == 1);
}

TEST_CASE("message link: link stats come from the transport") {
    LinkRig rig;
    CHECK_FALSE(rig.link->linkStats().has_value());
    rig.hostEnd->hasStats = true;
    rig.hostEnd->rttMs = 42;
    rig.hostEnd->lossPermille = 7;
    const auto stats = rig.link->linkStats();
    REQUIRE(stats.has_value());
    CHECK(stats->rttMs == 42);
    CHECK(stats->lossPermille == 7);
}

TEST_CASE("message link: by default the first malformed frame ends the link") {
    LinkRig rig;
    const std::array<std::byte, 4> garbage{std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}};
    REQUIRE(rig.client->transport().send(garbage).ok());
    InboundEvent ev;
    REQUIRE(rig.nextEvent(ev));
    CHECK(ev.kind == InboundEvent::Kind::Disconnected);
    CHECK(rig.link->malformedFrames() == 0);
}

TEST_CASE("message link: a malformed frame limit drops and counts frames, then ends the link") {
    LinkRig rig(3);
    const std::array<std::byte, 4> garbage{std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}};
    std::array<std::byte, 12> wrongVersion{};
    wrongVersion.fill(std::byte{0xFF});   // long enough for a header: refused as an unsupported wire format
    REQUIRE(rig.client->transport().send(garbage).ok());
    REQUIRE(rig.client->transport().send(wrongVersion).ok());
    wire::Leave leave;
    leave.detail = "still here";
    REQUIRE(rig.client->send(leave).ok());
    InboundEvent ev;
    REQUIRE(rig.nextEvent(ev));
    CHECK(ev.kind == InboundEvent::Kind::Message);   // the good frame came through between the bad ones
    CHECK(std::get<wire::Leave>(ev.message).detail == "still here");
    CHECK(rig.link->malformedFrames() == 2);
    CHECK(rig.link->connected());

    REQUIRE(rig.client->transport().send(garbage).ok());
    REQUIRE(rig.nextEvent(ev));
    CHECK(ev.kind == InboundEvent::Kind::Disconnected);
    CHECK(ev.detail.find("malformed") != std::string::npos);
    CHECK(rig.link->malformedFrames() == 3);
}

TEST_CASE("message link: a transport error is never tolerated as a malformed frame") {
    LinkRig rig(3);
    rig.client->close();
    InboundEvent ev;
    REQUIRE(rig.nextEvent(ev));
    CHECK(ev.kind == InboundEvent::Kind::Disconnected);
    CHECK(rig.link->malformedFrames() == 0);
}
