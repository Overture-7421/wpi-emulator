#include <doctest/doctest.h>

#include <set>
#include <string>
#include <thread>

#include "wpiemu/sdk/game/sound_cue.h"
#include "wpiemu/sdk/game/sound_cue_queue.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

using namespace wpiemu::sdk;

TEST_CASE("sound cues: the common table has unique ids, assets and no plugin prefix") {
    std::set<std::string> ids;
    for (const SoundCueInfo& cue : commonSoundCues()) {
        CHECK_FALSE(cue.id.empty());
        CHECK_FALSE(cue.asset.empty());
        CHECK(ids.insert(cue.id).second);
        CHECK_FALSE(cue.id.starts_with("game_"));
    }
    CHECK(ids.count(std::string(sound_cue::kMatchStopped)) == 1);
}

TEST_CASE("sound cue queue: oldest first, bounded by dropping the oldest") {
    SoundCueQueue queue(3);
    for (int i = 0; i < 5; ++i) queue.post({"cue" + std::to_string(i), i});
    CHECK(queue.size() == 3);

    const std::vector<SoundCue> first = queue.take(2);
    REQUIRE(first.size() == 2);
    CHECK(first[0].id == "cue2");
    CHECK(first[1].simTimeUs == 3);

    const std::vector<SoundCue> rest = queue.take();
    REQUIRE(rest.size() == 1);
    CHECK(rest[0].id == "cue4");
    CHECK(queue.take().empty());
}

TEST_CASE("sound cue queue: a producer thread and the consumer lose nothing under capacity") {
    SoundCueQueue queue;
    std::thread producer([&queue] {
        for (int i = 0; i < 50; ++i) queue.post({"c", i});
    });
    producer.join();
    const std::vector<SoundCue> cues = queue.take();
    REQUIRE(cues.size() == 50);
    for (int i = 0; i < 50; ++i) CHECK(cues[static_cast<std::size_t>(i)].simTimeUs == i);
}

TEST_CASE("sim context: requestSound stamps the sim time, and an unbound sink discards") {
    testing::FakeSimEnvironment env;
    env.advance();
    env.advance();
    env.ctx.requestSound(sound_cue::kEndGame);
    const std::vector<SoundCue> cues = env.sounds.take();
    REQUIRE(cues.size() == 1);
    CHECK(cues[0].id == sound_cue::kEndGame);
    CHECK(cues[0].simTimeUs == env.clock.simTimeUs());
    CHECK(cues[0].simTimeUs > 0);

    SimContext unbound(SimContextServices{env.physics, env.clock, env.hub, env.logger, env.config, env.rng,
                                          env.driverStation, BodyId{}});
    unbound.requestSound(sound_cue::kMatchStart);   // no sink: must not crash
    CHECK(env.sounds.size() == 0);
}
