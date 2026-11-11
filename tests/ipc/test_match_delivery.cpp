#include <doctest/doctest.h>

#include <chrono>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "match_samples.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/sdk/ipc/delivery.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/impaired_transport.h"

using namespace wpiemu;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;
using namespace wpiemu::sdk::testing;
using namespace std::chrono_literals;
using wpiemu::test_ipc::matchSamples;
using wpiemu::test_ipc::NamedMessage;
using wpiemu::test_ipc::sameMatchMessage;

namespace {

// The contract (sdk/match/): only the latest-wins streams are unreliable.
bool expectedUnreliable(std::string_view name) {
    return name == "MatchInputPacket" || name == "MatchSnapshot" || name == "MatchSnapshotAck";
}

struct ChannelPair {
    ChannelPair(const ManualWallClock& clock, const ImpairmentProfile& profile) {
        ImpairedTransportPair link = makeImpairedPair(clock, profile, 1);
        impaired = link.a.get();
        const auto codec = std::make_shared<ipc::BinaryMessageCodec>();
        sender = std::make_unique<MessageChannel>(std::move(link.a), codec);
        receiver = std::make_unique<MessageChannel>(std::move(link.b), codec);
    }
    void pump() { impaired->pump(); }
    std::vector<Message> drain() {
        std::vector<Message> got;
        for (;;) {
            Message m;
            const Result<bool> r = receiver->receive(m, 0ms);
            REQUIRE(r.ok());
            if (!r.value()) return got;
            got.push_back(std::move(m));
        }
    }
    ImpairedTransport* impaired = nullptr;
    std::unique_ptr<MessageChannel> sender;
    std::unique_ptr<MessageChannel> receiver;
};

}  // namespace

TEST_CASE("match delivery: deliveryOf agrees with the contract table") {
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        CHECK(deliveryOf(s.message) == (expectedUnreliable(s.name) ? Delivery::Unreliable : Delivery::Reliable));
    }
}

TEST_CASE("match delivery: over a link that loses every unreliable frame only the reliable messages arrive") {
    ManualWallClock clock;
    ChannelPair pair(clock, {.lossProbability = 1.0});
    std::vector<const NamedMessage*> reliable;
    const std::vector<NamedMessage> samples = matchSamples();
    for (const NamedMessage& s : samples) {
        REQUIRE(pair.sender->send(s.message, deliveryOf(s.message)).ok());
        if (!expectedUnreliable(s.name)) reliable.push_back(&s);
    }
    pair.pump();
    const std::vector<Message> got = pair.drain();
    REQUIRE(got.size() == reliable.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
        CAPTURE(reliable[i]->name);
        CHECK(sameMatchMessage(got[i], reliable[i]->message));
    }
    CHECK(pair.impaired->stats().dropped == samples.size() - reliable.size());
}

TEST_CASE("match delivery: with latency and jitter every message round-trips through the codec") {
    ManualWallClock clock;
    ChannelPair pair(clock, {.latency = 40ms, .jitter = 10ms});
    const std::vector<NamedMessage> samples = matchSamples();
    for (const NamedMessage& s : samples) REQUIRE(pair.sender->send(s.message, deliveryOf(s.message)).ok());
    CHECK(pair.drain().empty());
    clock.advance(200ms);
    pair.pump();
    const std::vector<Message> got = pair.drain();
    REQUIRE(got.size() == samples.size());
    std::map<std::size_t, const Message*> byIndex;
    for (const Message& m : got) byIndex[m.index()] = &m;
    for (const NamedMessage& s : samples) {
        CAPTURE(s.name);
        const auto it = byIndex.find(s.message.index());
        REQUIRE(it != byIndex.end());
        CHECK(sameMatchMessage(*it->second, s.message));
    }
}
