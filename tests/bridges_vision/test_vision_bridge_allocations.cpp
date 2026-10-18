#include <doctest/doctest.h>

#include <atomic>
#include <cstdlib>
#include <new>
#include <span>
#include <vector>

#include "vision_bridge_test_support.h"

// The steady state of the vision bridge allocates nothing: a robot without cameras runs this bridge on every tick of
// every project, so a tick with no frames, no change and nothing new on the robot must not touch the heap.
//
// This file replaces the global allocation functions of the whole test executable with counting ones (they count only
// while a test arms them). Keep it the only place that does: a second replacement would not link.

namespace {

std::atomic<bool> g_countAllocations{false};
std::atomic<std::size_t> g_allocations{0};

void* countedAllocate(std::size_t size) {
    if (g_countAllocations.load(std::memory_order_relaxed)) g_allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* memory = std::malloc(size != 0 ? size : 1)) return memory;
    throw std::bad_alloc();
}

}  // namespace

void* operator new(std::size_t size) {
    return countedAllocate(size);
}
void* operator new[](std::size_t size) {
    return countedAllocate(size);
}
void operator delete(void* memory) noexcept {
    std::free(memory);
}
void operator delete[](void* memory) noexcept {
    std::free(memory);
}
void operator delete(void* memory, std::size_t) noexcept {
    std::free(memory);
}
void operator delete[](void* memory, std::size_t) noexcept {
    std::free(memory);
}

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

// Runs `ticks` quiet ticks in the extension's order and returns how many allocations they made.
std::size_t allocationsOfQuietTicks(VisionRig& rig, int ticks) {
    std::vector<sdk::CameraRobotState> out;
    out.reserve(16);
    const std::span<const sdk::VisionFrame> noFrames;
    g_allocations.store(0);
    g_countAllocations.store(true);
    for (int i = 0; i < ticks; ++i) {
        rig.fake->advanceUs(kStepUs);
        rig.bridge->applyVision(noFrames);
        rig.bridge->poll();
        out.clear();
        rig.bridge->collectVision(out, false);
    }
    g_countAllocations.store(false);
    return g_allocations.load();
}

void warmUp(VisionRig& rig) {
    for (int i = 0; i < 40; ++i) {  // creation, the first reports and a few discovery runs
        rig.tick();
    }
}

TEST_CASE("vision bridge: the allocation counter of the steady-state tests sees allocations") {
    static volatile std::size_t extra = 0;  // keeps the compiler from folding the allocation away
    g_allocations.store(0);
    g_countAllocations.store(true);
    {
        std::vector<int> block(256 + extra);
        extra = extra + static_cast<std::size_t>(block[0]);
    }
    g_countAllocations.store(false);
    CHECK(g_allocations.load() >= 1);
}

TEST_CASE("vision bridge: the steady state with running cameras allocates nothing") {
    VisionRig rig;
    rig.recordNothing();
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    rig.start();
    warmUp(rig);
    REQUIRE(rig.bridge->stats().publishersStarted == 2);
    const std::size_t allocations = allocationsOfQuietTicks(rig, 400);
    CHECK(allocations == 0);
}

TEST_CASE("vision bridge: the steady state without ntcore allocates nothing") {
    VisionRig rig;
    rig.recordNothing();
    rig.fake->setAvailable(false);
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.start();
    warmUp(rig);
    const std::size_t allocations = allocationsOfQuietTicks(rig, 400);
    CHECK(allocations == 0);
}

TEST_CASE("vision bridge: the steady state with nothing configured allocates nothing") {
    VisionRig rig;
    rig.recordNothing();
    rig.start();
    warmUp(rig);
    const std::size_t allocations = allocationsOfQuietTicks(rig, 400);
    CHECK(allocations == 0);
}

}  // namespace
