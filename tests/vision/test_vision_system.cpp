#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "vision_rig.h"
#include "wpiemu/ipc/binary_codec.h"

using namespace vision_test;
using doctest::Approx;

namespace {

// The capture instants (sim time) of every frame of one camera over `ticks` ticks.
std::vector<sdk::SimTimeUs> captureInstants(sdk::CameraConfig cam, int ticks,
                                            sdk::SimTimeUs step = 5000) {
    cam.latencyUs = 0;
    Rig rig(visionOf({cam}), wallLayout(), step);
    std::vector<sdk::SimTimeUs> instants;
    for (const Collected& c : rig.run(ticks)) instants.push_back(c.frame.captureTimeUs);
    return instants;
}

std::uint64_t capturedCount(const Rig& rig, std::size_t camera = 0) {
    return rig.cameras().at(camera).framesCaptured;
}

const sdk::FiducialObservation* observationOf(const sdk::VisionFrame& frame, int id) {
    for (const sdk::FiducialObservation& o : frame.observations)
        if (o.id == id) return &o;
    return nullptr;
}

// One N(0, sigma) pair per corner, exactly as the engine draws it: (u1, u2) -> (x, y).
sdk::PixelPoint gaussianCorner(sdk::Rng& rng, double sigma) {
    const double u1 = rng.nextDouble(), u2 = rng.nextDouble();
    const double r = std::sqrt(-2.0 * std::log(1.0 - u1));
    return {sigma * r * std::cos(2.0 * kPi * u2), sigma * r * std::sin(2.0 * kPi * u2)};
}

}  // namespace

TEST_CASE("vision system: a 30 Hz camera at a 5 ms step captures on tick boundaries at the exact rate") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 30.0;
    Rig rig(visionOf({cam}));
    rig.run(2000);  // 10 s of sim time
    CHECK(capturedCount(rig) == 300);

    // 33.33 ms periods: 35, 70, 100, 135, 170, 200 ms, ...
    const auto instants = captureInstants(cam, 60);
    REQUIRE(instants.size() >= 6);
    CHECK(instants[0] == 35000);
    CHECK(instants[1] == 70000);
    CHECK(instants[2] == 100000);
    CHECK(instants[3] == 135000);
    CHECK(instants[4] == 170000);
    CHECK(instants[5] == 200000);
}

TEST_CASE("vision system: a 90 Hz camera at a 5 ms step captures at the exact rate, at most once per tick") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 90.0;
    Rig rig(visionOf({cam}));
    rig.run(2000);
    CHECK(capturedCount(rig) == 900);

    // 11.11 ms periods: 15, 25, 35, 45, 60 ms, ...
    const auto instants = captureInstants(cam, 40);
    REQUIRE(instants.size() >= 5);
    CHECK(instants[0] == 15000);
    CHECK(instants[1] == 25000);
    CHECK(instants[2] == 35000);
    CHECK(instants[3] == 45000);
    CHECK(instants[4] == 60000);
    for (std::size_t i = 1; i < instants.size(); ++i) CHECK(instants[i] - instants[i - 1] >= 5000);
}

TEST_CASE("vision system: a camera faster than the tick rate is capped at one capture per tick") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 240.0;  // above the 200 Hz tick rate
    Rig rig(visionOf({cam}));
    rig.run(1000);
    CHECK(capturedCount(rig) == 1000);
}

TEST_CASE("vision system: a rate with decimals keeps the long-run frame count exact") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 29.97;
    Rig rig(visionOf({cam}));
    rig.run(10000);  // 50 s
    CHECK(capturedCount(rig) == 1498);  // floor(50 * 29.97)

    sdk::CameraConfig slow = frontCamera();
    slow.frameRateHz = 1.0;
    Rig slowRig(visionOf({slow}));
    slowRig.run(2000);
    CHECK(capturedCount(slowRig) == 10);
}

TEST_CASE("vision system: the frame schedule follows the step, not a fixed 5 ms") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 50.0;  // 20 ms
    for (const sdk::SimTimeUs step : {1000, 2000, 4000, 10000}) {
        Rig rig(visionOf({cam}), wallLayout(), step);
        rig.run(static_cast<int>(1'000'000 / step));  // 1 s
        CHECK(capturedCount(rig) == 50);
    }
}

TEST_CASE("vision system: a frame rides the SensorFrame N0 + 1 + ceil(latency / step) and carries k * step") {
    struct Case {
        std::int64_t latencyUs;
        sdk::SimTimeUs step;
        double frameRateHz;
        int expectedK;
    };
    // a capture on every tick, except at 4 ms where 240 Hz (the maximum) misses one tick in 25
    const Case cases[] = {{25000, 5000, 200.0, 5}, {22000, 5000, 200.0, 5}, {0, 5000, 200.0, 0},
                          {5000, 5000, 200.0, 1},  {1, 5000, 200.0, 1},     {25000, 4000, 240.0, 7},
                          {40000, 10000, 100.0, 4}};
    for (const Case& c : cases) {
        sdk::CameraConfig cam = frontCamera();
        cam.latencyUs = c.latencyUs;
        cam.frameRateHz = c.frameRateHz;
        Rig rig(visionOf({cam}), wallLayout(), c.step);
        const auto frames = rig.run(30);
        REQUIRE(!frames.empty());
        const Collected& first = frames.front();
        CHECK(first.frame.sequence == 1);
        const int captureTick = static_cast<int>(first.frame.captureTimeUs / c.step);  // N0
        CHECK(first.frame.captureTimeUs == captureTick * c.step);
        CHECK(first.tick == captureTick + 1 + c.expectedK);
        CHECK(first.frame.latencyUs == c.expectedK * c.step);
        // the extension's clock reads T(N-1) when it applies this frame
        CHECK(static_cast<sdk::SimTimeUs>(first.tick - 1) * c.step ==
              first.frame.captureTimeUs + first.frame.latencyUs);
    }
}

TEST_CASE("vision system: frames arrive in capture order, one per capture, with consecutive sequences") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 60.0;
    cam.latencyUs = 30000;
    Rig rig(visionOf({cam}));
    const auto frames = rig.run(600);
    REQUIRE(frames.size() > 50);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        CHECK(frames[i].frame.sequence == i + 1);
        CHECK(static_cast<sdk::SimTimeUs>(frames[i].tick - 1) * 5000 ==
              frames[i].frame.captureTimeUs + frames[i].frame.latencyUs);
        if (i > 0) CHECK(frames[i].frame.captureTimeUs > frames[i - 1].frame.captureTimeUs);
    }
}

TEST_CASE("vision system: the Limelight capture share is clamped to the latency") {
    struct Case {
        std::int64_t latencyUs;
        std::int64_t expectedCl;
    };
    for (const Case& c : {Case{25000, 10000}, Case{5000, 5000}, Case{0, 0}}) {
        sdk::CameraConfig cam = frontCamera();
        cam.vendor = sdk::CameraVendor::Limelight;
        cam.frameRateHz = 200.0;
        cam.captureLatencyUs = 10000;
        cam.latencyUs = c.latencyUs;
        Rig rig(visionOf({cam}));
        const auto frames = rig.run(20);
        REQUIRE(!frames.empty());
        CHECK(frames.front().frame.captureLatencyUs == c.expectedCl);
        CHECK(frames.front().frame.captureLatencyUs <= frames.front().frame.latencyUs);
    }
}

TEST_CASE("vision system: latency jitter never reorders frames and keeps the latency exact") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 100.0;
    cam.latencyUs = 25000;
    cam.realism.latencyJitterUs = 20000;
    Rig rig(visionOf({cam}));
    const auto frames = rig.run(1500);
    REQUIRE(frames.size() > 400);
    std::int64_t shortest = std::numeric_limits<std::int64_t>::max(), longest = 0;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const sdk::VisionFrame& f = frames[i].frame;
        CHECK(f.sequence == i + 1);  // never overtaken
        CHECK(f.latencyUs % 5000 == 0);
        CHECK(static_cast<sdk::SimTimeUs>(frames[i].tick - 1) * 5000 == f.captureTimeUs + f.latencyUs);
        shortest = std::min(shortest, f.latencyUs);
        longest = std::max(longest, f.latencyUs);
    }
    CHECK(shortest <= 10000);  // the 5..45 ms range is actually used
    CHECK(longest >= 40000);
}

TEST_CASE("vision system: an ideal camera never draws from the random generator") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 90.0;
    Rig rig(visionOf({cam}));
    const auto frames = rig.run(1000);
    REQUIRE(!frames.empty());
    REQUIRE(!frames.front().frame.observations.empty());  // tags are in view, still no draws
    CHECK(rig.rng.state() == sdk::Rng(2024).state());
}

TEST_CASE("vision system: jitter, dropout and noise draw only when enabled and in a fixed amount") {
    sdk::CameraConfig base = frontCamera();
    base.frameRateHz = 200.0;  // a capture per tick
    const int ticks = 100;
    const auto count = static_cast<std::uint64_t>(ticks);

    {  // jitter only: one draw per capture
        sdk::CameraConfig cam = base;
        cam.realism.latencyJitterUs = 1000;
        Rig rig(visionOf({cam}));
        rig.run(ticks);
        CHECK(rig.draws() == count);
    }
    {  // dropout only: one draw per capture
        sdk::CameraConfig cam = base;
        cam.realism.dropoutProbability = 0.25;
        Rig rig(visionOf({cam}));
        rig.run(ticks);
        CHECK(rig.draws() == count);
    }
    {  // noise only: 2 draws per corner, 8 per visible tag (three tags in view)
        sdk::CameraConfig cam = base;
        cam.realism.cornerNoisePx = 0.5;
        Rig rig(visionOf({cam}));
        rig.run(ticks);
        CHECK(rig.draws() == count * 3 * 8);
    }
    {  // jitter + dropout; a dropped frame has nothing to add noise to
        sdk::CameraConfig cam = base;
        cam.realism.latencyJitterUs = 1000;
        cam.realism.dropoutProbability = 1.0;
        cam.realism.cornerNoisePx = 0.5;
        Rig rig(visionOf({cam}));
        rig.run(ticks);
        CHECK(rig.draws() == count * 2);
    }
}

TEST_CASE("vision system: a dropped frame is still published, without observations") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 200.0;
    cam.realism.dropoutProbability = 1.0;
    Rig rig(visionOf({cam}));
    const auto frames = rig.run(50);
    REQUIRE(frames.size() > 30);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        CHECK(frames[i].frame.observations.empty());
        CHECK_FALSE(frames[i].frame.multiTag.valid);
        CHECK(frames[i].frame.sequence == i + 1);
    }
    CHECK(rig.cameras()[0].visibleTagIds.empty());

    cam.realism.dropoutProbability = 0.5;
    Rig half(visionOf({cam}));
    int empty = 0, total = 0;
    for (const Collected& c : half.run(1000)) {
        ++total;
        empty += c.frame.observations.empty() ? 1 : 0;
    }
    CHECK(static_cast<double>(empty) / total == Approx(0.5).epsilon(0.15));
}

TEST_CASE("vision system: corner noise has the configured sigma and perturbs nothing else") {
    sdk::CameraConfig exact = frontCamera();
    exact.frameRateHz = 200.0;
    exact.latencyUs = 0;
    sdk::CameraConfig noisy = exact;
    noisy.realism.cornerNoisePx = 0.7;
    Rig reference(visionOf({exact}));
    Rig rig(visionOf({noisy}));
    const auto clean = reference.run(300);
    const auto frames = rig.run(300);
    REQUIRE(clean.size() == frames.size());
    double sum = 0.0, sumSquares = 0.0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        REQUIRE(frames[i].frame.observations.size() == 3);
        for (const sdk::FiducialObservation& want : clean[i].frame.observations) {
            const sdk::FiducialObservation* got = observationOf(frames[i].frame, want.id);
            REQUIRE(got != nullptr);
            for (std::size_t c = 0; c < 4; ++c) {
                for (const double d : {got->cornersPx[c].x - want.cornersPx[c].x,
                                       got->cornersPx[c].y - want.cornersPx[c].y}) {
                    sum += d;
                    sumSquares += d * d;
                    ++n;
                }
            }
            CHECK(got->fieldToTag == want.fieldToTag);  // layout data is not noised
        }
    }
    const double mean = sum / static_cast<double>(n);
    const double sigma = std::sqrt(sumSquares / static_cast<double>(n) - mean * mean);
    CHECK(n > 5000);
    CHECK(std::abs(mean) < 0.05);
    CHECK(sigma == Approx(0.7).epsilon(0.03));
    CHECK(frames[0].frame.robotToCamera == clean[0].frame.robotToCamera);
}

TEST_CASE("vision system: noise draws follow the camera config order, tag order and corner order") {
    sdk::CameraConfig a = frontCamera("a");
    a.frameRateHz = 200.0;
    a.latencyUs = 0;
    a.realism.cornerNoisePx = 0.5;
    sdk::CameraConfig b = a;
    b.id = "b";
    b.ntName = "other";
    b.realism.cornerNoisePx = 1.5;
    sdk::CameraConfig exact = a;
    exact.realism = {};

    Rig clean(visionOf({exact}));
    const auto cleanFrames = clean.run(2);  // tick 2 publishes the capture of tick 1
    Rig rig(visionOf({a, b}));
    const auto frames = rig.run(2);
    REQUIRE(cleanFrames.size() == 1);
    REQUIRE(frames.size() == 2);
    REQUIRE(frames[0].frame.cameraId == "a");  // collected oldest first, then config order
    REQUIRE(frames[1].frame.cameraId == "b");

    // Camera a draws first, then camera b; each over the tags in layout order (1, 2, 3), corners
    // BL, BR, TR, TL.
    sdk::Rng replay(2024);
    for (const Collected& c : frames) {
        const double sigma = c.frame.cameraId == "a" ? 0.5 : 1.5;
        for (const sdk::FiducialTag& tag : wallLayout().tags) {
            const auto* want = observationOf(cleanFrames[0].frame, tag.id);
            const auto* got = observationOf(c.frame, tag.id);
            REQUIRE((want != nullptr && got != nullptr));
            for (std::size_t k = 0; k < 4; ++k) {
                const sdk::PixelPoint offset = gaussianCorner(replay, sigma);
                CHECK(got->cornersPx[k].x == Approx(want->cornersPx[k].x + offset.x).epsilon(1e-12));
                CHECK(got->cornersPx[k].y == Approx(want->cornersPx[k].y + offset.y).epsilon(1e-12));
            }
        }
    }
}

TEST_CASE("vision system: the same seed and inputs give identical frames, another seed does not") {
    sdk::CameraConfig a = frontCamera("a");
    a.frameRateHz = 90.0;
    a.realism = {0.6, 15000, 0.1};
    sdk::CameraConfig b = frontCamera("b");
    b.ntName = "second";
    b.vendor = sdk::CameraVendor::Limelight;
    b.frameRateHz = 60.0;
    b.local = sdkPose(0.1f, 0.2f, 0.4f, 0.2f);
    b.realism.cornerNoisePx = 0.3;
    const sdk::VisionConfig config = visionOf({a, b});

    Rig one(config), two(config);
    const auto framesOne = one.run(800);
    const auto framesTwo = two.run(800);
    REQUIRE(framesOne.size() == framesTwo.size());
    REQUIRE(framesOne.size() > 100);
    for (std::size_t i = 0; i < framesOne.size(); ++i) {
        CHECK(framesOne[i].tick == framesTwo[i].tick);
        CHECK(framesOne[i].frame == framesTwo[i].frame);
    }

    Rig other(config);
    other.rng.seed(99);
    const auto framesOther = other.run(800);
    bool anyDifferent = framesOther.size() != framesOne.size();
    for (std::size_t i = 0; !anyDifferent && i < framesOne.size(); ++i)
        anyDifferent = !(framesOne[i].frame == framesOther[i].frame);
    CHECK(anyDifferent);
}

TEST_CASE("vision system: reset drops queued frames and the schedule phase but the sequence continues") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 30.0;
    cam.latencyUs = 25000;
    Rig rig(visionOf({cam}));
    rig.run(60);  // captures at 35, 70, 100, ... ms; the earliest are already published
    const std::uint64_t captured = capturedCount(rig);
    REQUIRE(captured >= 4);
    REQUIRE(!rig.cameras()[0].visibleTagIds.empty());
    const std::uint64_t published = rig.cameras()[0].framesPublished;
    REQUIRE(published >= 1);
    REQUIRE(published < captured);  // frames still in flight

    rig.system->reset();
    rig.timeUs = 0;  // the session clock restarts
    rig.tickCount = 0;
    CHECK(rig.cameras()[0].visibleTagIds.empty());
    CHECK(rig.cameras()[0].framesCaptured == captured);  // counters continue
    CHECK(rig.cameras()[0].framesPublished == published);

    const auto after = rig.run(40);
    REQUIRE(!after.empty());
    // nothing from before the reset is published; the first new capture is a full period away again
    CHECK(after.front().frame.sequence == captured + 1);
    CHECK(after.front().frame.captureTimeUs == 35000);
    for (std::size_t i = 0; i < after.size(); ++i) CHECK(after[i].frame.sequence == captured + 1 + i);
}

TEST_CASE("vision system: a teleported chassis changes the next capture without disturbing the schedule") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 30.0;
    cam.latencyUs = 25000;
    Rig base(visionOf({cam}));
    Rig moved(visionOf({cam}));
    const auto undisturbed = base.run(100);

    std::vector<Collected> frames = moved.run(40);  // the capture at 200 ms (tick 40) is in flight
    moved.world.place(sdkPose(1.f, 0.f, 0.f));      // 1 m toward the wall
    for (Collected& c : moved.run(60)) frames.push_back(std::move(c));

    // the capture instants are those of the undisturbed run
    REQUIRE(frames.size() == undisturbed.size());
    for (std::size_t i = 0; i < frames.size(); ++i)
        CHECK(frames[i].frame.captureTimeUs == undisturbed[i].frame.captureTimeUs);

    int before = 0, after = 0;
    for (const Collected& c : frames) {
        REQUIRE(c.frame.observations.size() == 3);
        for (const sdk::FiducialObservation& o : c.frame.observations) {
            if (c.frame.captureTimeUs <= 200000) {
                CHECK(o.distanceM > 3.9);  // captured before the teleport, published after it
            } else {
                CHECK(o.distanceM < 3.2);
            }
        }
        if (c.frame.captureTimeUs <= 200000)
            ++before;
        else
            ++after;
        if (c.frame.captureTimeUs == 200000) CHECK(c.tick > 40);
    }
    CHECK(before >= 5);
    CHECK(after >= 3);
}

TEST_CASE("vision system: with more due frames than fit in one SensorFrame, the oldest go first and the rest wait") {
    sdk::CameraConfig base = frontCamera();
    base.frameRateHz = 200.0;
    base.latencyUs = 0;
    std::vector<sdk::CameraConfig> cameras;
    for (std::size_t i = 0; i < 256; ++i) {
        sdk::CameraConfig c = base;
        c.id = "cam" + std::to_string(i);
        c.ntName = c.id;
        cameras.push_back(c);
    }
    auto created = vis::VisionSystem::create(visionOf(cameras), wallLayout(), 5000);
    REQUIRE(created.ok());
    auto system = std::move(created).value();
    FakeWorld world;
    sdk::Rng rng(1);
    for (int tick = 1; tick <= 5; ++tick) system->step(viewOf(world), rng, tick * 5000);  // 1280 frames

    std::vector<sdk::VisionFrame> out;
    system->collectDue(6 * 5000, out);
    REQUIRE(out.size() == vis::kMaxFramesPerSensorFrame);
    for (std::size_t i = 0; i < out.size(); ++i) {
        CHECK(out[i].captureTimeUs == static_cast<sdk::SimTimeUs>(i / 256 + 1) * 5000);  // oldest first
        CHECK(out[i].cameraId == "cam" + std::to_string(i % 256));                       // then config order
    }
    std::vector<sdk::VisionFrame> rest;
    system->collectDue(6 * 5000, rest);
    REQUIRE(rest.size() == 256);
    CHECK(rest.front().captureTimeUs == 25000);
    std::vector<sdk::VisionFrame> none;
    system->collectDue(6 * 5000, none);
    CHECK(none.empty());
}

TEST_CASE("vision system: the engine limits mirror the IPC codec caps") {
    CHECK(vis::kMaxFramesPerSensorFrame == wpiemu::ipc::kMaxVisionFrames);
    CHECK(vis::kMaxObservationsPerFrame == wpiemu::ipc::kMaxVisionObservations);
    CHECK(vis::kMaxEnabledCameras == wpiemu::ipc::kMaxVisionCameras);
}

TEST_CASE("vision system: create refuses configurations it cannot simulate") {
    const sdk::FiducialLayout layout = wallLayout();
    auto error = [&](const sdk::VisionConfig& v, sdk::SimTimeUs step = 5000) {
        auto r = vis::VisionSystem::create(v, layout, step);
        return r.ok() ? std::string() : r.error().message;
    };
    CHECK(error(visionOf({frontCamera()})).empty());
    CHECK_FALSE(error(visionOf({frontCamera()}), 0).empty());
    CHECK_FALSE(error(visionOf({frontCamera()}), -5).empty());
    CHECK(error(visionOf({frontCamera("a"), frontCamera("a")})).find("duplicate") != std::string::npos);
    CHECK_FALSE(error(visionOf({frontCamera("")})).empty());
    for (const double fps : {0.0, -5.0, 241.0, std::nan(""), 1e-9}) {
        sdk::CameraConfig c = frontCamera();
        c.frameRateHz = fps;
        CHECK(error(visionOf({c})).find("frame rate") != std::string::npos);
    }
    {
        sdk::CameraConfig c = frontCamera();
        c.intrinsics.widthPx = 0;
        CHECK_FALSE(error(visionOf({c})).empty());
        c = frontCamera();
        c.intrinsics.hfovRad = 0.0;
        CHECK_FALSE(error(visionOf({c})).empty());
        c = frontCamera();
        c.intrinsics.hfovRad = kPi;
        CHECK_FALSE(error(visionOf({c})).empty());
        c = frontCamera();
        c.intrinsics.hfovRad = 0.0;
        c.intrinsics.fxPx = 800.0;  // an explicit focal length makes the field of view irrelevant
        CHECK(error(visionOf({c})).empty());
    }
    {
        sdk::CameraConfig c = frontCamera();
        c.latencyUs = -1;
        CHECK_FALSE(error(visionOf({c})).empty());
        c = frontCamera();
        c.maxViewAngleRad = 0.0;
        CHECK_FALSE(error(visionOf({c})).empty());
        c = frontCamera();
        c.realism.dropoutProbability = 1.5;
        CHECK_FALSE(error(visionOf({c})).empty());
        c = frontCamera();
        c.realism.cornerNoisePx = -1.0;
        CHECK_FALSE(error(visionOf({c})).empty());
    }
    {  // 256 enabled cameras are fine, 257 are not; disabled ones do not count
        std::vector<sdk::CameraConfig> cams;
        for (std::size_t i = 0; i < 257; ++i) {
            sdk::CameraConfig c = frontCamera("cam" + std::to_string(i));
            c.ntName = c.id;
            cams.push_back(c);
        }
        CHECK(error(visionOf(cams)).find("256") != std::string::npos);
        cams.back().enabled = false;
        CHECK(error(visionOf(cams)).empty());
    }
}

TEST_CASE("vision system: a system without enabled cameras is inert") {
    sdk::CameraConfig off = frontCamera("off");
    off.enabled = false;
    off.realism.cornerNoisePx = 1.0;
    for (const sdk::VisionConfig& v : {sdk::VisionConfig{}, visionOf({off})}) {
        Rig rig(v);
        const auto frames = rig.run(300);
        CHECK(frames.empty());
        CHECK(rig.rng.state() == sdk::Rng(2024).state());
        CHECK(rig.system->setup("uuid").empty());
        CHECK(rig.system->enabledCameraCount() == 0);
    }
}
