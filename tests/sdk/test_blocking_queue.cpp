#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/blocking_queue.h"
#include "wpiemu/sdk/testing/deadlock_deadline.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace std::chrono_literals;
using wpiemu::sdk::BlockingQueue;
using wpiemu::sdk::testing::DeadlockDeadline;
using wpiemu::sdk::testing::waitUntil;

namespace {

using IntQueue = BlockingQueue<int>;

// Replacing or destroying it calls back into the queue it was popped from, which deadlocks if that queue still holds its lock.
struct Toucher {
    BlockingQueue<Toucher>* queue = nullptr;
    Toucher() = default;
    explicit Toucher(BlockingQueue<Toucher>* q) : queue(q) {}
    Toucher(Toucher&& other) noexcept : queue(std::exchange(other.queue, nullptr)) {}
    Toucher& operator=(Toucher&& other) noexcept {
        if (this != &other) {
            release();
            queue = std::exchange(other.queue, nullptr);
        }
        return *this;
    }
    ~Toucher() { release(); }
    void release();
};

void Toucher::release() {
    if (queue != nullptr) (void)queue->size();
    queue = nullptr;
}

}  // namespace

TEST_CASE("queue hands items out in order and times out when empty") {
    IntQueue q;
    for (int i = 1; i <= 3; ++i) CHECK(q.push(int{i}) == IntQueue::PushResult::Pushed);
    CHECK(q.size() == 3);
    int out = 0;
    for (int i = 1; i <= 3; ++i) {
        REQUIRE(q.pop(out, 0ms) == IntQueue::PopResult::Item);
        CHECK(out == i);
    }
    CHECK(q.pop(out, 0ms) == IntQueue::PopResult::Timeout);
    const auto start = std::chrono::steady_clock::now();
    CHECK(q.pop(out, 30ms) == IntQueue::PopResult::Timeout);
    CHECK(std::chrono::steady_clock::now() - start >= 25ms);
}

TEST_CASE("a full queue rejects a push and leaves the item with the caller") {
    BlockingQueue<std::unique_ptr<int>> q(2);
    CHECK(q.push(std::make_unique<int>(1)) == BlockingQueue<std::unique_ptr<int>>::PushResult::Pushed);
    CHECK(q.push(std::make_unique<int>(2)) == BlockingQueue<std::unique_ptr<int>>::PushResult::Pushed);
    auto rejected = std::make_unique<int>(3);
    CHECK(q.push(std::move(rejected)) == BlockingQueue<std::unique_ptr<int>>::PushResult::Full);
    REQUIRE(rejected != nullptr);
    CHECK(*rejected == 3);

    std::unique_ptr<int> out;
    REQUIRE(q.pop(out, 0ms) == BlockingQueue<std::unique_ptr<int>>::PopResult::Item);
    CHECK(*out == 1);
    CHECK(q.push(std::move(rejected)) == BlockingQueue<std::unique_ptr<int>>::PushResult::Pushed);
}

TEST_CASE("push after close is rejected but queued items still drain before Closed") {
    IntQueue q;
    q.push(1);
    q.push(2);
    q.close();
    q.close();
    CHECK(q.closed());
    CHECK(q.push(3) == IntQueue::PushResult::Closed);
    int out = 0;
    REQUIRE(q.pop(out, 0ms) == IntQueue::PopResult::Item);
    CHECK(out == 1);
    REQUIRE(q.pop(out, 5s) == IntQueue::PopResult::Item);
    CHECK(out == 2);
    CHECK(q.pop(out, 5s) == IntQueue::PopResult::Closed);
    CHECK(q.pop(out, 0ms) == IntQueue::PopResult::Closed);
}

TEST_CASE("closeAndDrain returns what was queued and closes the queue") {
    IntQueue q;
    q.push(7);
    q.push(8);
    const std::deque<int> pending = q.closeAndDrain();
    CHECK(pending == std::deque<int>{7, 8});
    CHECK(q.size() == 0);
    CHECK(q.closed());
    CHECK(q.push(9) == IntQueue::PushResult::Closed);
    int out = 0;
    CHECK(q.pop(out, 0ms) == IntQueue::PopResult::Closed);
    CHECK(q.closeAndDrain().empty());
}

TEST_CASE("close wakes every consumer blocked in pop") {
    DeadlockDeadline deadline;
    IntQueue q;
    std::atomic<int> blocked{0};
    std::atomic<int> closed{0};
    std::vector<std::thread> consumers;
    for (int i = 0; i < 4; ++i)
        consumers.emplace_back([&] {
            int out = 0;
            ++blocked;
            if (q.pop(out, 60s) == IntQueue::PopResult::Closed) ++closed;
        });
    REQUIRE(waitUntil([&] { return blocked == 4; }, 10s));
    std::this_thread::sleep_for(20ms);  // let them reach the wait
    q.close();
    REQUIRE(waitUntil([&] { return closed == 4; }, 10s));
    for (auto& t : consumers) t.join();
}

TEST_CASE("wake ends a wait early with Timeout and a wake before a wait ends the next one") {
    DeadlockDeadline deadline;
    IntQueue q;
    std::atomic<bool> waiting{false};
    std::atomic<int> result{-1};
    std::thread waiter([&] {
        int out = 0;
        waiting = true;
        result = static_cast<int>(q.pop(out, 60s));
    });
    REQUIRE(waitUntil([&] { return waiting.load(); }, 10s));
    std::this_thread::sleep_for(20ms);
    q.wake();
    REQUIRE(waitUntil([&] { return result.load() != -1; }, 10s));
    waiter.join();
    CHECK(result == static_cast<int>(IntQueue::PopResult::Timeout));

    int out = 0;
    q.wake();  // nobody waits: the next pop that would block returns at once, and only that one
    const auto start = std::chrono::steady_clock::now();
    CHECK(q.pop(out, 30s) == IntQueue::PopResult::Timeout);
    CHECK(std::chrono::steady_clock::now() - start < 10s);
    const auto second = std::chrono::steady_clock::now();
    CHECK(q.pop(out, 30ms) == IntQueue::PopResult::Timeout);
    CHECK(std::chrono::steady_clock::now() - second >= 25ms);
}

TEST_CASE("pop destroys the value it replaces after releasing the queue's lock") {
    DeadlockDeadline deadline;
    BlockingQueue<Toucher> q;
    Toucher out(&q);  // replaced by pop, and its destruction locks the queue
    q.push(Toucher{});
    REQUIRE(q.pop(out, 0ms) == BlockingQueue<Toucher>::PopResult::Item);
    CHECK(out.queue == nullptr);
}

TEST_CASE("producers and consumers move every item exactly once through a bounded queue") {
    DeadlockDeadline deadline(60s);
    constexpr int kProducers = 4;
    constexpr int kEach = 3000;
    constexpr int kConsumers = 3;
    IntQueue q(64);
    std::vector<std::vector<int>> received(kConsumers);
    std::vector<std::thread> consumers;
    for (int c = 0; c < kConsumers; ++c)
        consumers.emplace_back([&, c] {
            int item = 0;
            for (;;) {
                const auto result = q.pop(item, 50ms);
                if (result == IntQueue::PopResult::Closed) return;
                if (result == IntQueue::PopResult::Item) received[static_cast<std::size_t>(c)].push_back(item);
            }
        });
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p)
        producers.emplace_back([&, p] {
            for (int i = 0; i < kEach; ++i) {
                int id = p * kEach + i;
                while (q.push(std::move(id)) == IntQueue::PushResult::Full) std::this_thread::yield();
            }
        });
    for (auto& t : producers) t.join();
    q.close();
    for (auto& t : consumers) t.join();

    std::vector<int> all;
    for (const auto& r : received) all.insert(all.end(), r.begin(), r.end());
    std::sort(all.begin(), all.end());
    REQUIRE(all.size() == static_cast<std::size_t>(kProducers * kEach));
    for (std::size_t i = 0; i < all.size(); ++i)
        if (all[i] != static_cast<int>(i)) FAIL("item " << i << " is missing or duplicated");
    CHECK(q.size() == 0);
}

TEST_CASE("an item the queue accepted before close is never lost") {
    DeadlockDeadline deadline(60s);
    for (int round = 0; round < 20; ++round) {
        IntQueue q(1000);
        std::atomic<long> accepted{0};
        std::atomic<long> popped{0};
        std::atomic<bool> go{false};
        std::vector<std::thread> producers;
        for (int p = 0; p < 4; ++p)
            producers.emplace_back([&] {
                while (!go) std::this_thread::yield();
                for (;;) {
                    int item = 1;
                    const auto result = q.push(std::move(item));
                    if (result == IntQueue::PushResult::Closed) return;
                    if (result == IntQueue::PushResult::Pushed) ++accepted;
                    else std::this_thread::yield();
                }
            });
        std::vector<std::thread> consumers;
        for (int c = 0; c < 2; ++c)
            consumers.emplace_back([&] {
                int item = 0;
                for (;;) {
                    const auto result = q.pop(item, 50ms);
                    if (result == IntQueue::PopResult::Closed) return;
                    if (result == IntQueue::PopResult::Item) ++popped;
                }
            });
        go = true;
        REQUIRE(waitUntil([&] { return popped.load() > 100; }, 10s));
        q.close();
        for (auto& t : producers) t.join();
        for (auto& t : consumers) t.join();
        CHECK(popped == accepted);
    }
}
