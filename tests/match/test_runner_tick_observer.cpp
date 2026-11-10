#include <doctest/doctest.h>

#include <vector>

#include "../runtime/runner_rig.h"

using namespace wpiemu::test_runtime;
namespace wire = wpiemu::sdk::match;

namespace {

struct RecordingObserver final : ISimTickObserver {
    std::vector<RunnerObservation> polls;
    std::vector<RunnerObservation> tickEnds;
    std::vector<std::uint64_t> simTimeAtTickEnd;
    const SimulationSession* session = nullptr;

    void onPoll(const RunnerObservation& o) override { polls.push_back(o); }
    void onTickEnd(const RunnerObservation& o) override {
        tickEnds.push_back(o);
        simTimeAtTickEnd.push_back(static_cast<std::uint64_t>(session->clock().simTimeUs()));
    }
};

}  // namespace

TEST_CASE("runner tick observer: onPoll before every tick decision, onTickEnd after each tick") {
    Rig rig;
    RecordingObserver observer;
    observer.session = rig.session.get();
    rig.runner->setTickObserver(&observer);

    rig.runner->poll(1ms);   // no robot yet
    REQUIRE_FALSE(observer.polls.empty());
    CHECK(observer.polls.back().matchState == wire::MatchState::Lobby);
    CHECK(observer.tickEnds.empty());

    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 20; }));
    CHECK(observer.tickEnds.size() == rig.runner->ticksSent());
    CHECK(observer.polls.size() >= observer.tickEnds.size());
    CHECK(observer.polls.back().matchState == wire::MatchState::Running);
    // onTickEnd runs after the session advanced its clock: the sim time grows by one step per tick.
    for (std::size_t i = 1; i < observer.simTimeAtTickEnd.size(); ++i) CHECK(observer.simTimeAtTickEnd[i] > observer.simTimeAtTickEnd[i - 1]);
    const std::uint64_t epoch = observer.polls.back().runEpoch;
    CHECK(epoch == rig.runner->runEpoch());

    SUBCASE("pause and resume show as Paused and Running") {
        proto::SimControl pause;
        pause.command = proto::SimCommand::Pause;
        rig.runner->control(pause);
        REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Paused; }));
        CHECK(observer.polls.back().matchState == wire::MatchState::Paused);
        proto::SimControl resume;
        resume.command = proto::SimCommand::Resume;
        rig.runner->control(resume);
        REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Running; }));
        CHECK(observer.polls.back().matchState == wire::MatchState::Running);
    }
    SUBCASE("a reset changes the observed run epoch") {
        proto::SimControl reset;
        reset.command = proto::SimCommand::Reset;
        rig.runner->control(reset);
        REQUIRE(rig.run([&] { return observer.polls.back().runEpoch != epoch; }));
        CHECK(observer.polls.back().runEpoch == rig.runner->runEpoch());
    }
    SUBCASE("without an observer nothing changes") {
        rig.runner->setTickObserver(nullptr);
        const std::size_t polls = observer.polls.size();
        REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 40; }));
        CHECK(observer.polls.size() == polls);
    }
}
