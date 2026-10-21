#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "wpiemu/sdk/core/blocking_queue.h"
#include "wpiemu/sdk/core/stoppable_thread.h"
#include "wpiemu/sdk/testing/deadlock_deadline.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace std::chrono_literals;
using wpiemu::sdk::BlockingQueue;
using wpiemu::sdk::StoppableThread;
using wpiemu::sdk::StopToken;
using wpiemu::sdk::testing::DeadlockDeadline;
using wpiemu::sdk::testing::waitUntil;

TEST_CASE("the destructor requests a stop and joins the body") {
    DeadlockDeadline deadline;
    std::atomic<bool> running{false};
    std::atomic<bool> finished{false};
    {
        StoppableThread thread;
        REQUIRE(thread.start([&](const StopToken& stop) {
            running = true;
            while (!stop.stopRequested()) std::this_thread::sleep_for(1ms);
            finished = true;
        }));
        REQUIRE(waitUntil([&] { return running.load(); }, 10s));
        CHECK(thread.joinable());
        CHECK_FALSE(thread.stopRequested());
    }
    CHECK(finished);
}

TEST_CASE("a stop request wakes a body blocked in a queue once the owner wakes the queue") {
    DeadlockDeadline deadline;
    BlockingQueue<int> queue;
    std::atomic<int> handled{0};
    std::atomic<bool> blocked{false};
    StoppableThread thread;
    thread.start([&](const StopToken& stop) {
        int item = 0;
        while (!stop.stopRequested()) {
            blocked = true;
            if (queue.pop(item, 60s) == BlockingQueue<int>::PopResult::Item) ++handled;
        }
    });
    queue.push(1);
    REQUIRE(waitUntil([&] { return handled == 1 && blocked.load(); }, 10s));
    thread.requestStop();
    queue.wake();
    thread.join();
    CHECK_FALSE(thread.joinable());
    CHECK(thread.stopRequested());
}

TEST_CASE("a closed queue ends a body that is blocked in pop") {
    DeadlockDeadline deadline;
    BlockingQueue<int> queue;
    std::atomic<bool> returned{false};
    StoppableThread thread;
    thread.start([&](const StopToken&) {
        int item = 0;
        queue.pop(item, 60s);
        returned = true;
    });
    queue.close();
    thread.join();
    CHECK(returned);
}

TEST_CASE("starting a running thread does nothing and a joined thread can start again") {
    DeadlockDeadline deadline;
    std::atomic<int> runs{0};
    std::atomic<bool> stoppedAtStart{true};
    StoppableThread thread;
    auto body = [&](const StopToken& stop) {
        stoppedAtStart = stop.stopRequested();
        ++runs;
        while (!stop.stopRequested()) std::this_thread::sleep_for(1ms);
    };
    REQUIRE(thread.start(body));
    CHECK_FALSE(thread.start(body));
    thread.stop();
    CHECK(runs == 1);

    thread.resetStop();
    REQUIRE(thread.start(body));
    REQUIRE(waitUntil([&] { return runs == 2; }, 10s));
    CHECK_FALSE(stoppedAtStart);
    thread.stop();
}

TEST_CASE("a stop requested before start is still pending for the body") {
    DeadlockDeadline deadline;
    std::atomic<bool> stoppedAtStart{false};
    StoppableThread thread;
    thread.requestStop();
    thread.start([&](const StopToken& stop) { stoppedAtStart = stop.stopRequested(); });
    thread.join();
    CHECK(stoppedAtStart);
    CHECK(thread.stopRequested());
}

TEST_CASE("stop and join are idempotent and safe without a thread") {
    StoppableThread idle;
    idle.join();
    idle.stop();
    idle.stop();
    CHECK_FALSE(idle.joinable());

    DeadlockDeadline deadline;
    StoppableThread thread;
    thread.start([](const StopToken& stop) {
        while (!stop.stopRequested()) std::this_thread::sleep_for(1ms);
    });
    thread.stop();
    thread.stop();
    thread.join();
    CHECK_FALSE(thread.joinable());
}

TEST_CASE("the body can request its own stop and its owner observes it") {
    DeadlockDeadline deadline;
    StoppableThread thread;
    thread.start([&thread](const StopToken&) { thread.requestStop(); });
    REQUIRE(waitUntil([&] { return thread.stopRequested(); }, 10s));
    thread.join();
}

TEST_CASE("many threads started and stopped at once all join") {
    DeadlockDeadline deadline(60s);
    constexpr int kThreads = 16;
    std::atomic<int> started{0};
    std::atomic<int> finished{0};
    for (int round = 0; round < 10; ++round) {
        started = 0;
        finished = 0;
        std::vector<StoppableThread> threads(kThreads);
        for (StoppableThread& thread : threads)
            thread.start([&](const StopToken& stop) {
                ++started;
                while (!stop.stopRequested()) std::this_thread::yield();
                ++finished;
            });
        REQUIRE(waitUntil([&] { return started == kThreads; }, 20s));
        for (StoppableThread& thread : threads) thread.requestStop();
        threads.clear();  // joins each in its destructor
        CHECK(finished == kThreads);
    }
}
