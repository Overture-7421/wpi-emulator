#include <doctest/doctest.h>

#include <cstdint>

#include "wpiemu/sdk/physics/robot_tags.h"

using namespace wpiemu::sdk;

TEST_CASE("robot tags round-trip every participant id") {
    for (std::uint32_t id = 0; id <= 0xFFFF; ++id) {
        const std::uint64_t tag = makeRobotTag(static_cast<match::ParticipantId>(id));
        REQUIRE(isRobotTag(tag));
        REQUIRE(robotTagParticipant(tag) == id);
    }
}

TEST_CASE("robot tags are distinct per participant and never zero") {
    CHECK(makeRobotTag(0) != 0u);
    CHECK(makeRobotTag(1) != makeRobotTag(2));
    CHECK(makeRobotTag(match::kHostParticipantId) != makeRobotTag(1));
}

TEST_CASE("robot tag detection rejects untagged shapes and other tag schemes") {
    CHECK_FALSE(isRobotTag(0));
    CHECK(robotTagParticipant(0) == match::kNoParticipant);
    // The top byte alone decides a scheme: a tag of any other kind byte is not a robot tag, whatever its low bits hold.
    for (std::uint64_t kind = 0; kind <= 0xFF; ++kind) {
        if (kind == kRobotTagMarker) continue;
        CHECK_FALSE(isRobotTag((kind << 56) | 0x0001u));
        CHECK_FALSE(isRobotTag((kind << 56) | (std::uint64_t{1} << 48) | 0x1234u));
    }
    // The marker with stray middle bits is not something makeRobotTag produces.
    CHECK_FALSE(isRobotTag(makeRobotTag(3) | (std::uint64_t{1} << 20)));
    CHECK(robotTagParticipant(makeRobotTag(3) | (std::uint64_t{1} << 20)) == match::kNoParticipant);
}
