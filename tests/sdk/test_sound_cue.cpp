#include <doctest/doctest.h>

#include "wpiemu/sdk/game/sound_cue.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

using namespace wpiemu::sdk;

TEST_CASE("sound cue log numbers cues, keeps the most recent ones and publishes them oldest first") {
    SoundCueLog log;
    for (int i = 0; i < 12; ++i) log.raise("g.sound.a", 1000 * i);

    REQUIRE(log.events().size() == SoundCueLog::kCapacity);
    CHECK(log.events().front().sequence == 5);
    CHECK(log.events().back().sequence == 12);
    CHECK(log.events().back().simTimeUs == 11000);

    SnapshotBuilder builder;
    log.publish(builder);
    const auto snapshot = builder.build();
    REQUIRE(snapshot->soundCues().size() == SoundCueLog::kCapacity);
    CHECK(snapshot->soundCues().front().cueId == "g.sound.a");
    CHECK(snapshot->soundCues().front().sequence < snapshot->soundCues().back().sequence);
}

TEST_CASE("sound cue log keeps counting after clear so a reset never reuses a sequence") {
    SoundCueLog log;
    log.raise("g.sound.a", 0);
    log.raise("g.sound.b", 10);
    log.clear();
    CHECK(log.events().empty());

    log.raise("g.sound.c", 20);
    REQUIRE(log.events().size() == 1);
    CHECK(log.events().front().sequence == 3);
    CHECK(log.lastSequence() == 3);
}

TEST_CASE("snapshots without a game have no sound cues") {
    SimSnapshot snapshot{SimSnapshotData{}};
    CHECK(snapshot.soundCues().empty());
}
