#include <doctest/doctest.h>

#include <QApplication>
#include <QFile>
#include <QSettings>
#include <QSignalSpy>
#include <QTemporaryDir>

#include <utility>
#include <vector>

#include "driver_station_model.h"
#include "fake_joystick.h"
#include "fake_run_backend.h"
#include "run_page.h"
#include "sfx_catalog.h"
#include "sfx_player.h"
#include "wpiemu/sdk/game/sound_cue.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

struct RecordingOutput final : ISfxOutput {
    bool device = true;
    std::vector<QUrl> preloaded;
    std::vector<std::pair<QUrl, double>> played;
    int stops = 0;

    bool available() const override { return device; }
    void preload(const QUrl& url) override { preloaded.push_back(url); }
    void play(const QUrl& url, double volume) override { played.emplace_back(url, volume); }
    void stopAll() override { ++stops; }
};

QString id(std::string_view cue) { return QString::fromUtf8(cue.data(), static_cast<qsizetype>(cue.size())); }

QStringList cuesOf(const QSignalSpy& spy) {
    QStringList out;
    for (const QList<QVariant>& args : spy) out << args.at(0).toString();
    return out;
}

// A Practice match of auto / gap / teleop seconds on a Driver Station with a robot.
struct PracticeDs {
    explicit PracticeDs(double autoS = 2.0, double gapS = 1.0, double teleopS = 3.0) : spy(&ds, &DriverStationModel::sfxRequested) {
        ds.setRobotAvailable(true);
        MatchSettings m;
        m.autoSeconds = autoS;
        m.gapSeconds = gapS;
        m.teleopSeconds = teleopS;
        ds.setMatchSettings(m);
        ds.setDsMode(DsMode::Practice);
    }
    DriverStationModel ds;
    QSignalSpy spy;
};

}  // namespace

TEST_CASE("sfx catalog: every common cue plays an embedded file") {
    SfxCatalog catalog;
    CHECK(catalog.set(QString(), {}).isEmpty());
    for (const sdk::SoundCueInfo& cue : sdk::commonSoundCues()) {
        const auto url = catalog.resolve(QString::fromStdString(cue.id));
        REQUIRE(url.has_value());
        CHECK(QFile::exists(QLatin1Char(':') + url->path()));
    }
    CHECK_FALSE(catalog.resolve(QStringLiteral("nope")).has_value());
    CHECK(catalog.urls().size() == 7);   // eight cues, the AUTO and MATCH buzzers share one file
}

TEST_CASE("sfx catalog: a game's cues need its prefix (so none can shadow a common cue) and an embedded file") {
    SfxCatalog catalog;
    const QStringList problems = catalog.set(
        QStringLiteral("game_x"),
        {{"game_x.ok", "", "", "missing_file"}, {"other.cue", "", "", "start"}, {std::string(sdk::sound_cue::kMatchStart), "", "", "start"}});
    CHECK(problems.size() == 3);
    CHECK_FALSE(catalog.resolve(QStringLiteral("game_x.ok")).has_value());     // not embedded
    CHECK_FALSE(catalog.resolve(QStringLiteral("other.cue")).has_value());     // wrong prefix
    CHECK(catalog.resolve(QString::fromUtf8(sdk::sound_cue::kMatchStart.data())).value() == commonSoundUrl(QStringLiteral("start")));
    CHECK(pluginSoundUrl(QStringLiteral("game_x"), QStringLiteral("a")).toString() == QLatin1String("qrc:/wpiemu/plugins/game_x/sfx/a.wav"));
}

TEST_CASE("sfx player: plays at the chosen volume only while enabled, active and not muted") {
    RecordingOutput out;
    SfxPlayer player(&out);
    player.setCatalog(QString(), {});
    CHECK(out.preloaded.size() == 7);

    player.play(id(sdk::sound_cue::kMatchStart));
    REQUIRE(out.played.size() == 1);
    CHECK(out.played[0].first == commonSoundUrl(QStringLiteral("start")));
    CHECK(out.played[0].second == doctest::Approx(0.7));

    player.setVolumePercent(250);
    CHECK(player.volumePercent() == 100);
    player.play(id(sdk::sound_cue::kMatchEnd));
    CHECK(out.played.back().second == doctest::Approx(1.0));

    const std::size_t before = out.played.size();
    player.setVolumePercent(0);
    player.play(id(sdk::sound_cue::kMatchEnd));
    player.setVolumePercent(50);
    player.setEnabled(false);
    CHECK(out.stops == 1);
    player.play(id(sdk::sound_cue::kMatchEnd));
    player.setEnabled(true);
    player.setActive(false);
    CHECK(out.stops == 2);
    player.play(id(sdk::sound_cue::kMatchEnd));
    CHECK(out.played.size() == before);
    player.setActive(true);
    player.play(id(sdk::sound_cue::kMatchEnd));
    CHECK(out.played.size() == before + 1);
}

TEST_CASE("sfx player: an unknown cue and a missing audio device are reported once") {
    RecordingOutput out;
    SfxPlayer player(&out);
    player.setCatalog(QString(), {});
    QSignalSpy problems(&player, &SfxPlayer::problem);

    player.play(QStringLiteral("game_y.mystery"));
    player.play(QStringLiteral("game_y.mystery"));
    CHECK(problems.count() == 1);

    out.device = false;
    player.play(id(sdk::sound_cue::kMatchEnd));
    player.play(id(sdk::sound_cue::kMatchEnd));
    CHECK(problems.count() == 2);
    CHECK(out.played.empty());

    SfxPlayer silent(nullptr);   // no output at all: nothing to do, nothing reported
    silent.setCatalog(QString(), {});
    silent.play(id(sdk::sound_cue::kMatchEnd));
}

TEST_CASE("match timer reports each boundary an advance crosses, in order") {
    MatchTimer t;
    t.begin({{sdk::proto::RobotMode::Autonomous, true, 1.0}, {sdk::proto::RobotMode::Teleop, false, 1.0},
             {sdk::proto::RobotMode::Teleop, true, 1.0}});
    CHECK_FALSE(t.advance(0.5));
    CHECK(t.crossed().empty());
    CHECK(t.advance(1.0));
    REQUIRE(t.crossed().size() == 1);
    CHECK(t.crossed()[0].from == 0);
    CHECK(t.crossed()[0].to == 1);
    CHECK_FALSE(t.advance(0.1));
    CHECK(t.crossed().empty());
    CHECK(t.advance(5.0));
    REQUIRE(t.crossed().size() == 2);
    CHECK(t.crossed()[0].to == 2);
    CHECK(t.crossed()[1].from == 2);
    CHECK(t.crossed()[1].to == -1);
}

TEST_CASE("driver station sfx: a Practice match sounds start, auto end, teleop start and end (manual Table 5-4)") {
    PracticeDs p;
    p.ds.setEnabled(true);
    CHECK(cuesOf(p.spy) == QStringList{id(sdk::sound_cue::kMatchStart)});
    p.ds.onSimTime(0);
    p.ds.onSimTime(1'000'000);
    p.ds.onSimTime(2'100'000);
    CHECK(cuesOf(p.spy).back() == id(sdk::sound_cue::kAutoEnd));
    p.ds.onSimTime(3'200'000);
    CHECK(cuesOf(p.spy).back() == id(sdk::sound_cue::kTeleopStart));
    p.ds.onSimTime(6'300'000);
    CHECK(cuesOf(p.spy) == QStringList{id(sdk::sound_cue::kMatchStart), id(sdk::sound_cue::kAutoEnd),
                                       id(sdk::sound_cue::kTeleopStart), id(sdk::sound_cue::kMatchEnd)});
}

TEST_CASE("driver station sfx: one long advance still sounds every boundary, with or without a gap") {
    PracticeDs p(1.0, 1.0, 1.0);
    p.ds.setEnabled(true);
    p.ds.onSimTime(0);
    p.ds.onSimTime(3'500'000);
    CHECK(cuesOf(p.spy).mid(1) == QStringList{id(sdk::sound_cue::kAutoEnd), id(sdk::sound_cue::kTeleopStart),
                                              id(sdk::sound_cue::kMatchEnd)});

    PracticeDs noGap(1.0, 0.0, 1.0);
    noGap.ds.setEnabled(true);
    noGap.ds.onSimTime(0);
    noGap.ds.onSimTime(1'100'000);
    CHECK(cuesOf(noGap.spy).mid(1) == QStringList{id(sdk::sound_cue::kAutoEnd), id(sdk::sound_cue::kTeleopStart)});
}

TEST_CASE("driver station sfx: only an E-Stop of a running Practice match sounds the foghorn") {
    {
        PracticeDs p;
        p.ds.setEnabled(true);
        p.ds.eStop();
        CHECK(cuesOf(p.spy) == QStringList{id(sdk::sound_cue::kMatchStart), id(sdk::sound_cue::kMatchStopped)});
        p.ds.eStop();   // already latched: no second foghorn
        CHECK(p.spy.count() == 2);
    }
    {
        PracticeDs p;   // Disable, a mode change and a robot loss end the match without a sound
        p.ds.setEnabled(true);
        p.ds.disable();
        p.ds.setEnabled(true);
        p.ds.setMode(sdk::proto::RobotMode::Teleop);
        p.ds.setDsMode(DsMode::Practice);
        p.ds.setEnabled(true);
        p.ds.setRobotAvailable(false);
        CHECK(cuesOf(p.spy) == QStringList(3, id(sdk::sound_cue::kMatchStart)));
    }
    {
        PracticeDs p;   // no match running: nothing to stop
        p.ds.eStop();
        CHECK(p.spy.isEmpty());
    }
    {
        DriverStationModel ds;   // outside Practice there is no match clock and no sound
        QSignalSpy spy(&ds, &DriverStationModel::sfxRequested);
        ds.setRobotAvailable(true);
        ds.setDsMode(DsMode::Teleop);
        ds.setEnabled(true);
        ds.eStop();
        ds.clearEStop();
        ds.onSimReset();
        CHECK(spy.isEmpty());
    }
}

TEST_CASE("driver station sfx: a restart in Practice chimes the field reset and not the foghorn") {
    PracticeDs p;
    p.ds.setEnabled(true);
    p.ds.onSimReset();
    CHECK(cuesOf(p.spy) == QStringList{id(sdk::sound_cue::kMatchStart), id(sdk::sound_cue::kFieldReset)});
}

namespace {

struct SfxFixture {
    QTemporaryDir dir;
    QSettings settings{dir.filePath(QStringLiteral("s.ini")), QSettings::IniFormat};
    FakeRunBackend backend{[] {
        FakeRunBackend::Options o;
        o.autoRun = false;
        o.startupDelayMs = 0;
        return o;
    }()};
    FakeJoystickProvider gamepads;
    RecordingOutput output;
    RunPage page{backend, [this] {
                     RunPageOptions o;
                     o.settings = &settings;
                     o.gamepads = &gamepads;
                     o.sfx = &output;
                     return o;
                 }()};

    SfxFixture() {
        page.resize(1200, 800);
        page.show();
        QApplication::processEvents();
    }
};

}  // namespace

TEST_CASE("run page sfx: host cues and Driver Station cues reach the player, and preferences and visibility gate them") {
    SfxFixture f;
    emit f.backend.soundCue(id(sdk::sound_cue::kEndGame));
    REQUIRE(f.output.played.size() == 1);
    CHECK(f.output.played[0].first == commonSoundUrl(QStringLiteral("warning")));

    emit f.page.driverStation().sfxRequested(id(sdk::sound_cue::kMatchStart));
    CHECK(f.output.played.size() == 2);

    f.page.setSoundEffects(false, 40);
    emit f.backend.soundCue(id(sdk::sound_cue::kEndGame));
    CHECK(f.output.played.size() == 2);
    f.page.setSoundEffects(true, 40);
    emit f.backend.soundCue(id(sdk::sound_cue::kEndGame));
    REQUIRE(f.output.played.size() == 3);
    CHECK(f.output.played.back().second == doctest::Approx(0.4));

    f.page.hide();   // leaving the Run page silences it
    emit f.backend.soundCue(id(sdk::sound_cue::kEndGame));
    CHECK(f.output.played.size() == 3);
    f.page.show();
    emit f.backend.soundCue(id(sdk::sound_cue::kEndGame));
    CHECK(f.output.played.size() == 4);
}
