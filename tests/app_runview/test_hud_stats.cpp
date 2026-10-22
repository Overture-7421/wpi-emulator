#include <doctest/doctest.h>

#include <QSignalSpy>

#include "hud_model.h"
#include "sim_stats.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

sdk::SimSnapshot snapshotWith(std::initializer_list<sdk::HudValue> values, std::uint64_t tick = 0, sdk::SimTimeUs us = 0) {
    sdk::SimSnapshotData d;
    d.tick = tick;
    d.simTimeUs = us;
    d.hud = values;
    return sdk::SimSnapshot(std::move(d));
}

sdk::HudWidgetDescriptor descriptor(sdk::HudWidgetKind kind, const char* key) {
    sdk::HudWidgetDescriptor d;
    d.id = std::string("t.") + key;
    d.title = key;
    d.kind = kind;
    d.valueKey = key;
    return d;
}

}  // namespace

TEST_CASE("hud: counter, text and timer widgets") {
    auto counter = descriptor(sdk::HudWidgetKind::Counter, "score");
    counter.unit = "pts";
    const auto snap = snapshotWith({{"score", 12}, {"msg", "ABC"}, {"t", 83.4}});
    const HudItemState c = evaluateHud(counter, &snap);
    CHECK(c.valid);
    CHECK(c.text == QStringLiteral("12 pts"));

    CHECK(evaluateHud(descriptor(sdk::HudWidgetKind::Text, "msg"), &snap).text == QStringLiteral("ABC"));
    CHECK(evaluateHud(descriptor(sdk::HudWidgetKind::Timer, "t"), &snap).text == QStringLiteral("1:23"));
    CHECK(formatClock(-3.0) == QStringLiteral("0:00"));
    CHECK(formatClock(140.0) == QStringLiteral("2:20"));
}

TEST_CASE("hud: meter uses the dynamic maximum when given") {
    auto meter = descriptor(sdk::HudWidgetKind::Meter, "fuel");
    meter.maxKey = "cap";
    meter.maxValue = 99.0;
    const auto snap = snapshotWith({{"fuel", 15}, {"cap", 30}});
    const HudItemState m = evaluateHud(meter, &snap);
    CHECK(m.fraction == doctest::Approx(0.5));
    CHECK(m.text == QStringLiteral("15 / 30"));

    const auto noMax = snapshotWith({{"fuel", 15}});
    CHECK(evaluateHud(meter, &noMax).fraction == doctest::Approx(15.0 / 99.0));

    const auto over = snapshotWith({{"fuel", 50}, {"cap", 30}});
    CHECK(evaluateHud(meter, &over).fraction == doctest::Approx(1.0));
}

TEST_CASE("hud: scoreboard highlights the leader, shows the margin and needs both scores") {
    auto board = descriptor(sdk::HudWidgetKind::Scoreboard, "phase");
    board.sides = {{"BLUE", "blue", "blueStatus", "#1a73e8"}, {"RED", "red", "redStatus", "#d93025"}};

    const auto redAhead = snapshotWith({{"blue", 4}, {"red", 10}, {"phase", std::string("SHIFT 2")},
                                        {"blueStatus", std::string("Inactive")}, {"redStatus", std::string("Active 0:18")}});
    const HudItemState s = evaluateHud(board, &redAhead);
    REQUIRE(s.valid);
    REQUIRE(s.sides.size() == 2);
    CHECK(s.sides[0].label == QStringLiteral("BLUE"));   // sides[0] is the left one
    CHECK(s.sides[0].score == QStringLiteral("4"));
    CHECK_FALSE(s.sides[0].leading);
    CHECK_FALSE(s.sides[0].statusActive);
    CHECK(s.sides[1].leading);
    CHECK(s.sides[1].statusActive);
    CHECK(s.leader == 1);
    CHECK(s.lead == QStringLiteral("RED +6"));
    CHECK(s.text == QStringLiteral("SHIFT 2"));

    const auto tied = snapshotWith({{"blue", 3}, {"red", 3}});   // no caption, no status
    const HudItemState t = evaluateHud(board, &tied);
    CHECK(t.valid);
    CHECK(t.leader == -1);
    CHECK(t.lead == QStringLiteral("TIED"));
    CHECK(t.sides[0].status.isEmpty());

    const auto missing = snapshotWith({{"blue", 3}});
    CHECK_FALSE(evaluateHud(board, &missing).valid);
    CHECK_FALSE(evaluateHud(board, nullptr).valid);
}

TEST_CASE("hud: badge reads booleans and state strings") {
    const auto badge = descriptor(sdk::HudWidgetKind::Badge, "hub");
    const auto on = snapshotWith({{"hub", true}});
    const auto off = snapshotWith({{"hub", false}});
    const auto word = snapshotWith({{"hub", "Inactive"}});
    CHECK(evaluateHud(badge, &on).active);
    CHECK_FALSE(evaluateHud(badge, &off).active);
    CHECK_FALSE(evaluateHud(badge, &word).active);
    CHECK(evaluateHud(badge, &word).text == QStringLiteral("Inactive"));
}

TEST_CASE("hud: missing values and a missing snapshot show a placeholder") {
    const auto d = descriptor(sdk::HudWidgetKind::Counter, "nothing");
    const auto snap = snapshotWith({});
    CHECK_FALSE(evaluateHud(d, &snap).valid);
    CHECK_FALSE(evaluateHud(d, nullptr).valid);
    CHECK(evaluateHud(d, nullptr).text == QStringLiteral("—"));
}

TEST_CASE("hud model: sorted per anchor, emits only for changed rows") {
    HudModel model;
    auto a = descriptor(sdk::HudWidgetKind::Counter, "a");
    a.anchor = sdk::HudAnchor::TopRight;
    a.order = 2;
    auto b = descriptor(sdk::HudWidgetKind::Counter, "b");
    b.anchor = sdk::HudAnchor::TopRight;
    b.order = 1;
    auto c = descriptor(sdk::HudWidgetKind::Counter, "c");
    c.anchor = sdk::HudAnchor::BottomCenter;
    model.setDescriptors({a, b, c});
    REQUIRE(model.rowCount() == 3);

    auto* topRight = qobject_cast<QAbstractItemModel*>(model.anchorModel(static_cast<int>(sdk::HudAnchor::TopRight)));
    REQUIRE(topRight != nullptr);
    CHECK(topRight->rowCount() == 2);
    CHECK(topRight->index(0, 0).data(HudModel::TitleRole).toString() == QStringLiteral("b"));
    CHECK(qobject_cast<QAbstractItemModel*>(model.anchorModel(static_cast<int>(sdk::HudAnchor::BottomCenter)))->rowCount() == 1);
    CHECK(model.anchorModel(99) == nullptr);

    QSignalSpy spy(&model, &QAbstractItemModel::dataChanged);
    const auto s1 = snapshotWith({{"a", 1}, {"b", 2}, {"c", 3}});
    model.update(&s1);
    CHECK(spy.count() == 3);
    spy.clear();
    model.update(&s1);
    CHECK(spy.count() == 0);
    const auto s2 = snapshotWith({{"a", 1}, {"b", 5}, {"c", 3}});
    model.update(&s2);
    CHECK(spy.count() == 1);
}

TEST_CASE("sim stats compute rates from consecutive snapshots") {
    SimStats stats;
    CHECK_FALSE(stats.values().hasData);
    const auto first = snapshotWith({}, 0, 0);
    CHECK(stats.update(&first, 1000));
    CHECK(stats.values().hasData);
    CHECK(stats.values().tickRateHz == doctest::Approx(0.0));

    const auto later = snapshotWith({}, 200, 1'000'000);
    stats.update(&later, 2000);   // 1 s of wall time: 200 ticks, 1 s of sim time
    CHECK(stats.values().tickRateHz == doctest::Approx(200.0));
    CHECK(stats.values().realtimeFactor == doctest::Approx(1.0));

    const auto fast = snapshotWith({}, 1000, 5'000'000);
    stats.update(&fast, 3000);    // 4x speed
    CHECK(stats.values().realtimeFactor == doctest::Approx(4.0));

    CHECK(stats.update(nullptr, 3500));
    CHECK_FALSE(stats.values().hasData);
    CHECK(formatSimTime(75.3) == QStringLiteral("01:15.3"));
}

TEST_CASE("sim stats carry the physics step time and worker count") {
    sdk::SimSnapshotData d;
    d.physicsStats.lastStepMs = 0.4;
    d.physicsStats.workerCount = 4;
    const sdk::SimSnapshot snapshot(std::move(d));
    SimStats stats;
    stats.update(&snapshot, 1000);
    CHECK(stats.values().physicsStepMs == doctest::Approx(0.4));
    CHECK(stats.values().physicsWorkers == 4);
}
