#include <doctest/doctest.h>

#include <QSettings>
#include <QTemporaryDir>

#include <string>
#include <vector>

#include "sound_cue_model.h"
#include "test_support.h"

using namespace wpiemu;
using namespace wpiemu::app::runview;
using runview_test::RecordingSoundOutput;

namespace {

sdk::SoundCueInfo cue(const char* id, bool interrupts = false) {
    sdk::SoundCueInfo c;
    c.id = id;
    c.asset = std::string("g/") + id + ".wav";
    c.interruptsOthers = interrupts;
    return c;
}

struct Feed {
    RecordingSoundOutput out;
    SoundCueModel model{&out, nullptr};

    Feed() { model.setCatalog({cue("g.a"), cue("g.b"), cue("g.stop", true)}); }

    // One snapshot at `tick` with the given (id, sequence, raisedAtUs) cues.
    void snap(std::uint64_t tick, std::vector<sdk::SoundCueEvent> cues, sdk::SimTimeUs simUs = -1) {
        sdk::SimSnapshotData d = runview_test::baseSnapshot();
        d.tick = tick;
        d.simTimeUs = simUs >= 0 ? simUs : static_cast<sdk::SimTimeUs>(tick) * 5000;
        d.soundCues = std::move(cues);
        const sdk::SimSnapshot s{std::move(d)};
        model.onSnapshot(&s);
    }
};

sdk::SoundCueEvent ev(const char* id, std::uint64_t seq, sdk::SimTimeUs atUs) { return {id, seq, atUs}; }

}  // namespace

TEST_CASE("sound cues: each sequence plays once, in order, however often the snapshot repeats") {
    Feed f;
    f.snap(1, {});  // primes
    f.snap(2, {ev("g.a", 1, 10000)});
    f.snap(3, {ev("g.a", 1, 10000)});
    f.snap(4, {ev("g.a", 1, 10000), ev("g.b", 2, 15000), ev("g.a", 3, 15000)});
    f.snap(5, {ev("g.b", 2, 15000), ev("g.a", 3, 15000)});
    CHECK(f.out.events == std::vector<std::string>{"g.a", "g.b", "g.a"});
}

TEST_CASE("sound cues: cues that already happened are not replayed when the model starts watching") {
    Feed f;
    f.snap(100, {ev("g.a", 1, 400000), ev("g.b", 2, 450000)});  // first snapshot: only learns the sequence
    f.snap(101, {ev("g.a", 1, 400000), ev("g.b", 2, 450000)});
    CHECK(f.out.events.empty());

    f.model.suspend();  // page hidden
    f.snap(200, {ev("g.a", 1, 400000), ev("g.b", 2, 450000), ev("g.a", 3, 1000000)});
    CHECK(f.out.events == std::vector<std::string>{"<stop>"});  // primed again, silent
    f.snap(201, {ev("g.b", 4, 1005000)});
    CHECK(f.out.events == std::vector<std::string>{"<stop>", "g.b"});
}

TEST_CASE("sound cues: a restart of the tick counter stops the sound and starts a fresh sequence") {
    Feed f;
    f.snap(1, {});
    f.snap(1000, {ev("g.a", 7, 5000000)});
    REQUIRE(f.out.events == std::vector<std::string>{"g.a"});

    f.snap(3, {});  // Reset or a new robot session: ticks and sequences start over
    CHECK(f.out.events == std::vector<std::string>{"g.a", "<stop>"});
    f.snap(4, {ev("g.b", 1, 20000)});
    CHECK(f.out.events == std::vector<std::string>{"g.a", "<stop>", "g.b"});
}

TEST_CASE("sound cues: stale cues are skipped") {
    Feed f;
    f.snap(1, {});
    f.snap(2, {ev("g.a", 1, 0)}, SoundCueModel::kMaxCueAgeUs + 1);
    f.snap(3, {ev("g.b", 2, 2000000)}, 2000000 + SoundCueModel::kMaxCueAgeUs);
    CHECK(f.out.events == std::vector<std::string>{"g.b"});
}

TEST_CASE("sound cues: a cue that interrupts others stops what is playing first") {
    Feed f;
    f.snap(1, {});
    f.snap(2, {ev("g.a", 1, 10000), ev("g.stop", 2, 10000)});
    CHECK(f.out.events == std::vector<std::string>{"g.a", "<stop>", "g.stop"});
}

TEST_CASE("sound cues: an unknown cue is reported once and never played") {
    Feed f;
    std::vector<QString> problems;
    QObject::connect(&f.model, &SoundCueModel::problem, [&](const QString& m) { problems.push_back(m); });
    f.snap(1, {});
    f.snap(2, {ev("g.nope", 1, 10000)});
    f.snap(3, {ev("g.nope", 1, 10000), ev("g.nope", 2, 15000)});
    CHECK(f.out.events.empty());
    CHECK(problems.size() == 1);
}

TEST_CASE("sound cues: switching the sounds off silences them and the choice is remembered") {
    QTemporaryDir dir;
    const QString file = dir.filePath(QStringLiteral("s.ini"));
    {
        QSettings settings{file, QSettings::IniFormat};
        RecordingSoundOutput out;
        SoundCueModel model{&out, &settings};
        CHECK(model.enabled());  // on by default
        model.setCatalog({cue("g.a")});
        auto snap = [&](std::uint64_t tick, std::vector<sdk::SoundCueEvent> cues) {
            sdk::SimSnapshotData d = runview_test::baseSnapshot();
            d.tick = tick;
            d.simTimeUs = static_cast<sdk::SimTimeUs>(tick) * 5000;
            d.soundCues = std::move(cues);
            const sdk::SimSnapshot s{std::move(d)};
            model.onSnapshot(&s);
        };
        snap(1, {});
        model.setEnabled(false);
        snap(2, {ev("g.a", 1, 10000)});
        CHECK(out.events == std::vector<std::string>{"<stop>"});
        snap(3, {ev("g.a", 1, 10000), ev("g.a", 2, 15000)});  // muted cues are consumed, not replayed later
        model.setEnabled(true);
        snap(4, {ev("g.a", 1, 10000), ev("g.a", 2, 15000)});
        CHECK(out.events == std::vector<std::string>{"<stop>"});
        model.setEnabled(false);
    }
    QSettings settings{file, QSettings::IniFormat};
    RecordingSoundOutput out;
    SoundCueModel again{&out, &settings};
    CHECK_FALSE(again.enabled());
}

TEST_CASE("sound cues: the audio is reloaded only when the cue list changes") {
    Feed f;
    CHECK(f.out.catalogLoads == 1);
    f.model.setCatalog({cue("g.a"), cue("g.b"), cue("g.stop", true)});
    CHECK(f.out.catalogLoads == 1);
    f.model.setCatalog({cue("g.a")});
    CHECK(f.out.catalogLoads == 2);
    CHECK(f.out.catalogSize == 1);
}

TEST_CASE("sound cues: problems of the output reach the model") {
    Feed f;
    QString seen;
    QObject::connect(&f.model, &SoundCueModel::problem, [&](const QString& m) { seen = m; });
    f.out.problem("no audio device");
    CHECK(seen == QStringLiteral("no audio device"));
}
