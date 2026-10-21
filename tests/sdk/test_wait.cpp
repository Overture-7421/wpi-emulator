#include <doctest/doctest.h>

#include <chrono>
#include <thread>

#include "wpiemu/sdk/testing/wait.h"

using namespace std::chrono_literals;
using wpiemu::sdk::testing::waitUntil;

TEST_CASE("waitUntil returns at once when the condition already holds") {
    int polls = 0;
    CHECK(waitUntil([&] { return ++polls > 0; }, 5s));
    CHECK(polls == 1);
}

TEST_CASE("waitUntil reports false only after the full timeout") {
    const auto start = std::chrono::steady_clock::now();
    CHECK_FALSE(waitUntil([] { return false; }, 40ms, 1ms));
    const auto waited = std::chrono::steady_clock::now() - start;
    CHECK(waited >= 40ms);
    CHECK(waited < 10s);
}

TEST_CASE("waitUntil evaluates the condition once more after the deadline") {
    int polls = 0;
    const bool held = waitUntil(
        [&] {
            if (++polls > 1) return true;
            std::this_thread::sleep_for(80ms);  // the stall lands past the 20 ms deadline
            return false;
        },
        20ms, 1ms);
    CHECK(held);
    CHECK(polls == 2);
}

TEST_CASE("waitUntil polls no more often than the interval allows") {
    int polls = 0;
    CHECK_FALSE(waitUntil([&] { return ++polls < 0; }, 100ms, 20ms));
    CHECK(polls >= 2);
    CHECK(polls <= 6);  // five polls fit before the deadline, plus the final one
}

TEST_CASE("waitUntil runs the pump between unsuccessful polls only") {
    int pumped = 0;
    CHECK(waitUntil([&] { return pumped >= 3; }, 5s, 1ms, [&] { ++pumped; }));
    CHECK(pumped == 3);

    pumped = 0;
    CHECK(waitUntil([] { return true; }, 5s, 1ms, [&] { ++pumped; }));
    CHECK(pumped == 0);
}
