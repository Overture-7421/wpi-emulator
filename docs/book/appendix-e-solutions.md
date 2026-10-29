# Appendix E. Exercise answers and worked tests

## E.1 Foundations

Chapter 1: a CAN controller belongs to a bridge/device integration; a vault rule
belongs to the game; carpet geometry belongs to its field. A build does not
advance a stack. Counting every custody state after a release can expose a
phantom cube even if the scene looks plausible.

Chapter 2: a release in game step 100 first moves in physics step 101.
A persistent snapshot condition repeats every publication, so it is not itself
an edge-triggered sound event. Comparing per-tick hashes is stronger than
comparing only equal final scores.

Chapter 3: mixed friction is 0.4. Full extents mistakenly supplied as
half-extents multiply volume by eight and reduce inferred density by eight
for unchanged mass. A cube-only sensor accepts the generic piece category;
the piece mask must also accept the sensor category, with events enabled
on both shapes.

Chapter 4: under RLR, Blue's home-switch positive-Y side is Red and negative-Y
side Blue; the scale is the opposite assignment. On the far switch the
alliance-relative view reverses world-space Y. Custody proceeds field to held
to field to exchange to vault. A cube count is not an ownership duration.

## E.2 Core implementation

Chapter 5: application composition links the plugin aggregator, which links
the manifest-selected library and invokes its registration function. Mutable
score in the module would be shared between sessions. Duplicate registration
returns the SDK's already-exists error rather than replacing an instance.

Chapter 6: doubling a carpet thickness from 0.20 to 0.40 m requires its centre
at -0.20 m to keep the top at zero. The opposite-wall worked test below changes
the input direction, not production geometry. Detailed wall and aperture
geometry comes from drawings and section 3, not Table 4-1.

Chapter 7: four triangular tiers contain 10 + 6 + 3 + 1 = 20 cubes.
The double-reset worked test compares every restored field position after
disturbing custody. Reserve bodies are disabled and cannot be ordinary pickups;
feeding must transfer them explicitly.

Chapter 8: the original 0.02 m/s baseline moves at most 0.0001 m per 5 ms
tick. The final 0.04 m/s model doubles that bound. The unloaded-plate worked
test disables the delivered body and waits for level. A cube traveling above
0.4 m/s can overlap the sensor without satisfying the resting definition.

Chapter 9: enabled Test remains Test and cannot score. Disabled autonomous
is Disabled; enabled untimed autonomous remains Autonomous. Scoring and
physics have different gates, so settling need not award points.

Chapter 10: world release velocity includes the chassis's velocity at the
release point, including rotational motion, plus rotated ejection velocity.
Pickup tie-breaking uses fixed pool order. A missing attachment or mechanism
rejects activation and warns once instead of selecting an arbitrary pose.

Chapter 11: bindings decode assignment, the factory passes it to state, and
the runner overlays its message into Driver Station input. The presentation
test in Chapter 20 compares actual published keys with both scoreboard sides.
Position-only entity samples contain no orientation from which to draw rotation.

Chapter 12: omitting pool reset leaves inventory/release assertions wrong;
omitting score reset leaves the accounting assertions wrong. A retained
immutable snapshot remains an observation of its earlier run. The Chapter 20
descriptor test supplies the requested live-publication assertion.

## E.3 Robot and rule calculations

Chapter 13: the requested release covers callback counts 350 through 399,
approximately 7-8 s at 20 ms per callback. A PWM mismatch leaves the roller
without that output. Changing Blue's home assignment from L to R changes
the start lane from W/2 + 1.3716 to W/2 - 1.3716 m.

Chapter 14: a reversed mapping may be detected by an L3 drivetrain integration
test, and the complete driving assertion detects it at L4. Constant allocated
bodies do not prevent two logical locations referring to one identity.
Lengthening disable advances simulation time but not scoring-window time.

Chapter 15: 1.25 s qualifying autonomous time gives a 2-point establishment
award plus 0.25 times 2, totaling 2.5 points or 2,500,000 micro-points.
Per-tick floor would discard every sub-point increment. Any additional
subdivision must retain the original total interval and unchanged inputs.

Chapter 16: delivery changes field to exchange, and return changes exchange
to field; both retain the same total. The lower aperture is robot-to-human.
A partly completed human period resumes after disable; disabled time is not
added to its elapsed clock.

Chapter 17: levels select switch, scale, both. An accepted level is copied into
power state, so later vault counts cannot change it. With an already-owned
teleoperation target, half a second Boosted and half unboosted gives
2 times 0.5 + 1 times 0.5 = 1.5 ownership points before publication rounding.

Chapter 18: 50 kg at nominal 9.81 m/s² weighs about 490.5 N, far above 10 N.
The brick threshold is 0.09 + 0.3048 = 0.3948 m. A positive engaged upward
cable load distinguishes the modeled suspension from unsupported flight.

Chapter 19: a tie plus Auto Quest earns two ranking points. A virtual outcome
has no physical support observation. A 15 + 60 s practice history never reaches
the built-in 150 s finish events; it does not compress them.

Chapter 20: remaining time changes every active tick; active identity changes
only on activation or promotion. Another tone can use a different generated
frequency and asset name while retaining original provenance. A recording
sink tests requests; a real audio device and desktop run test playback.

## E.4 Synthesis

Chapter 21: basket overlap is an observation, uninterrupted 0.5 s residence is
rule state, and the disk still needs one custody location. Deformation changes
collision and residence observations, so rigid-body contact evidence no longer
suffices. An API rename belongs to season/toolchain integration, not the rule.

Chapter 22: a repeatable slide or drop experiment could fit friction or
restitution with independent validation trials. Five pictures provide neither
physical bodies nor robot outputs. A new out-of-field recovery policy needs
custody-conservation, phase and reset tests before its row can change.

## E.5 Compiling worked tests

These complete tests implement the opposite-wall, double-reset and unloaded-plate
exercises. Extraction through Chapter 22 includes them in the normal test target.
Other implementation exercises above identify the required oracle or refer to
an existing complete test; they do not depend on unlisted solution code.

Listing E.1. Worked component tests for the core exercises. <code>tests\game_2027\test_exercises.cpp</code>.

<!-- book-file: tests\game_2027\test_exercises.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/plates.h"

namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: exercise double reset restores every field position") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::CubePool pool(*world);
    REQUIRE(pool.build());
    pool.reset(true, 0);
    const auto first = pool.positions(pu::CubeLocation::Field);
    REQUIRE(pool.release({{1.f, 1.f, 1.f}, {}}, {2.f, 0.f, 0.f}));
    pool.reset(true, 0);
    pool.reset(true, 0);
    const auto second = pool.positions(pu::CubeLocation::Field);
    REQUIRE(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        CHECK(first[i].x == second[i].x);
        CHECK(first[i].y == second[i].y);
        CHECK(first[i].z == second[i].z);
    }
    CHECK(pool.inventory() == 1);
    CHECK(pool.released() == 0);
}

TEST_CASE("Power UP: exercise opposite wall stops a moving cube") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::Field field(*world);
    REQUIRE(field.build());
    pu::Bodies bodies(*world);
    const auto cube = bodies.box({{pu::kLengthM - 1.f, 1.f, 1.f}, {}}, pu::kCubeHalfM,
                                 s::BodyType::Dynamic, s::layer::Fuel);
    REQUIRE(cube);
    world->setLinearVelocity(cube.value(), {2.f, 0.f, 0.f});
    for (int i = 0; i < 1000; ++i) world->step(0.005f);
    const auto p = world->pose(cube.value()).position;
    CHECK(p.x < pu::kLengthM - pu::kCubeHalfM.x + 0.02f);
    CHECK(p.z == doctest::Approx(pu::kCubeHalfM.z).epsilon(0.02));
}

TEST_CASE("Power UP: exercise unloaded plate returns to level") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::Field field(*world);
    REQUIRE(field.build());
    pu::Plates plates(*world);
    REQUIRE(plates.build());
    plates.reset();
    const auto level = plates.center(0).z;
    pu::Bodies bodies(*world);
    const auto cube = bodies.box({plates.center(0) + s::Vec3{0.f, 0.f, 0.5f}, {}}, pu::kCubeHalfM,
                                 s::BodyType::Dynamic, s::layer::Fuel, pu::kCubeTag);
    REQUIRE(cube);
    for (int i = 0; i < 1000; ++i) {
        world->step(0.005f);
        plates.update(0.005);
    }
    CHECK(plates.center(0).z < level - 0.08f);
    world->setEnabled(cube.value(), false);
    for (int i = 0; i < 1000; ++i) {
        world->step(0.005f);
        plates.update(0.005);
    }
    CHECK(plates.center(0).z == doctest::Approx(level).epsilon(0.001));
    CHECK(plates.values()[0].cubes == 0);
}
```

[Appendix F](appendix-f-references.md)
