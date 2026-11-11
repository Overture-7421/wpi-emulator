#include <doctest/doctest.h>

#include <QApplication>
#include <QColor>
#include <QSignalSpy>
#include <QSettings>
#include <QTemporaryDir>

#include "fake_joystick.h"
#include "fake_run_backend.h"
#include "remote_rigs.h"
#include "run_page.h"
#include "scene_model.h"
#include "test_support.h"
#include "ui/qt_text.h"
#include "ui/scene_support.h"
#include "wpiemu/sdk/testing/demo_game.h"

using namespace wpiemu::app::runview;
using namespace runview_test;
using wpiemu::app::MatchRoster;
using wpiemu::app::RemoteParticipant;
namespace sdk = wpiemu::sdk;
namespace demo = wpiemu::sdk::testing::demo_game;
namespace ui = wpiemu::app::ui;

namespace {

std::shared_ptr<const sdk::match::RobotAppearance> demoAppearance() {
    return std::make_shared<const sdk::match::RobotAppearance>(FakeRunBackend::makeDemoAppearance());
}

RemoteParticipant participant(sdk::match::ParticipantId id, const char* name, sdk::proto::AllianceStation station,
                              std::shared_ptr<const sdk::match::RobotAppearance> appearance = demoAppearance()) {
    RemoteParticipant p;
    p.id = id;
    p.name = name;
    p.station = station;
    p.appearance = std::move(appearance);
    return p;
}

std::shared_ptr<const MatchRoster> rosterOf(std::vector<RemoteParticipant> participants) {
    return std::make_shared<const MatchRoster>(std::move(participants));
}

// A remote robot's sample: 4 wheels named like the CLIENT's spec (not like the appearance), one active zone and one moved zone.
sdk::RobotSample sampleOf(sdk::match::ParticipantId id, sdk::Vec3 at, float yaw = 0.f) {
    const sdk::match::RobotAppearance a = FakeRunBackend::makeDemoAppearance();
    sdk::RobotSample s;
    s.participantId = id;
    s.body.pose = {at, sdk::Quat::fromYaw(yaw)};
    const std::vector<sdk::WheelSpec> wheels = sdk::match::toWheelSpecs(a);
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        sdk::WheelSample w;
        w.name = "client_wheel_" + std::to_string(i);
        w.contactPoint = s.body.pose.transformPoint({wheels[i].positionM.x, wheels[i].positionM.y, -at.z});
        w.contactPoint.z = 0.f;
        w.steerAngleRad = 0.25 * static_cast<double>(i + 1);
        w.spinAngleRad = 0.5 * static_cast<double>(i + 1);
        s.wheels.push_back(std::move(w));
    }
    for (std::size_t i = 0; i < a.zones.size(); ++i) s.zones.push_back({"z" + std::to_string(i), false, std::nullopt});
    return s;
}

const ScenePart* findPart(const std::vector<ScenePart>& parts, const std::string& name) {
    for (const ScenePart& p : parts)
        if (p.name == name) return &p;
    return nullptr;
}

constexpr auto kBlue1 = sdk::proto::AllianceStation::Blue1;
constexpr auto kRed2 = sdk::proto::AllianceStation::Red2;

}  // namespace

TEST_CASE("remote rigs: the appearance becomes a chassis with wheels, primitives and zones at their rest poses") {
    const sdk::match::RobotAppearance a = FakeRunBackend::makeDemoAppearance();
    REQUIRE_FALSE(a.empty());
    const sdk::RobotConfig c = robotConfigOfAppearance(a);
    CHECK(c.chassis.sizeM.x == doctest::Approx(0.76f).epsilon(0.01));
    CHECK(c.chassis.primitives.size() == a.primitives.size());
    REQUIRE(c.drivetrain.spec.wheels.size() == a.wheels.size());
    CHECK(c.drivetrain.spec.wheels[0].radiusM == doctest::Approx(sdk::match::appearanceMetres(a.wheels[0].radiusMm)));
    REQUIRE(c.zones.size() == a.zones.size());
    for (std::size_t i = 0; i < c.zones.size(); ++i) {
        CHECK(c.zones[i].kind == a.zones[i].kind);
        CHECK(c.zones[i].attachment.onChassis());
        CHECK(near(c.zones[i].local.position, sdk::match::appearanceZoneRestPose(a.zones[i]).position));
    }
}

TEST_CASE("remote rigs: a rig draws the frame, the wheels and the zone volumes of the appearance") {
    RemoteRigSet set;
    set.setZoneKinds(demo::zoneKinds());
    set.setRoster(rosterOf({participant(1, "Owls", kBlue1)}));
    REQUIRE(set.size() == 1);
    const RemoteRig& rig = set.at(0);
    CHECK(rig.id() == 1);
    CHECK(rig.name() == "Owls");
    // 4 wheels x (tire + marker) beyond the chassis primitives and the front marker
    CHECK(rig.robot().parts().size() == rig.robot().staticCount() + 8);
    CHECK(findPart(rig.robot().parts(), "front.marker") != nullptr);
    // the pickup volume (1 part) and the shooter muzzle (marker + arrow)
    CHECK(rig.zones().parts().size() == 3);
    CHECK_FALSE(rig.present());
    CHECK(rig.marker().size.z > 0.f);
    CHECK(rig.labelAnchor().z > rig.marker().pose.position.z);
}

TEST_CASE("remote rigs: rigs join and leave with the roster and only the changed ones are rebuilt") {
    RemoteRigSet set;
    set.setZoneKinds(demo::zoneKinds());
    const auto owls = demoAppearance();
    const auto rockets = demoAppearance();

    auto diff = set.setRoster(rosterOf({participant(1, "Owls", kBlue1, owls)}));
    CHECK(diff.added == std::vector<std::size_t>{0});
    CHECK(diff.removed.empty());

    diff = set.setRoster(rosterOf({participant(1, "Owls", kBlue1, owls), participant(2, "Rockets", kRed2, rockets)}));
    CHECK(diff.added == std::vector<std::size_t>{1});
    CHECK(diff.removed.empty());
    REQUIRE(set.size() == 2);
    const RemoteRig* first = &set.at(0);

    diff = set.setRoster(rosterOf({participant(1, "Owls", kBlue1, owls), participant(2, "Rockets", kRed2, rockets)}));
    CHECK_FALSE(diff.any());   // nothing changed
    CHECK(&set.at(0) == first);

    diff = set.setRoster(rosterOf({participant(1, "Owls", kBlue1, owls)}));   // Rockets left
    CHECK(diff.removed == std::vector<std::size_t>{1});
    CHECK(diff.added.empty());
    CHECK(set.size() == 1);
    CHECK(&set.at(0) == first);   // the remaining robot was kept

    diff = set.setRoster(rosterOf({participant(1, "Owls", sdk::proto::AllianceStation::Blue3, owls)}));
    CHECK(diff.identityChanged);
    CHECK(set.at(0).station() == sdk::proto::AllianceStation::Blue3);

    diff = set.setRoster(nullptr);
    CHECK(diff.removed == std::vector<std::size_t>{0});
    CHECK(set.size() == 0);
}

TEST_CASE("remote rigs: a participant whose appearance is unknown is not drawn until it is known") {
    RemoteRigSet set;
    set.setRoster(rosterOf({participant(1, "Old host", kBlue1, nullptr)}));
    CHECK(set.size() == 0);
    set.setRoster(rosterOf({participant(1, "Old host", kBlue1, std::make_shared<const sdk::match::RobotAppearance>())}));
    CHECK(set.size() == 0);   // an empty appearance means "unknown"
    const auto diff = set.setRoster(rosterOf({participant(1, "Old host", kBlue1)}));
    CHECK(diff.added == std::vector<std::size_t>{0});
}

TEST_CASE("remote rigs: a robot's sample moves its chassis, steers and spins its wheels by position, and shows it") {
    RemoteRigSet set;
    set.setZoneKinds(demo::zoneKinds());
    set.setRoster(rosterOf({participant(1, "Owls", kBlue1), participant(2, "Rockets", kRed2)}));

    std::vector<sdk::RobotSample> samples{sampleOf(1, {5.f, 3.f, 0.2f}, 0.5f)};
    CHECK(set.update(samples));
    const RemoteRig& owls = *set.find(1);
    const RemoteRig& rockets = *set.find(2);
    CHECK(owls.present());
    CHECK(near(owls.pose().position, {5.f, 3.f, 0.2f}));
    CHECK_FALSE(rockets.present());   // no sample for it: hidden, not removed
    CHECK(set.size() == 2);

    // wheel 1 spins and steers although its sample name is the client's own
    const ScenePart* tire = findPart(owls.robot().parts(), "wheel1");
    REQUIRE(tire != nullptr);
    CHECK_FALSE(tire->pose.rotation == sdk::Quat{});
    const sdk::Quat expected = sdk::Quat::fromYaw(0.5f) * ui::kWheelAxisXToY.conjugate() * sdk::Quat::fromAxisAngle({0.f, 0.f, 1.f}, 1.0f);
    CHECK(tire->pose.rotation.x == doctest::Approx(expected.x).epsilon(1e-3));
    CHECK(tire->pose.rotation.w == doctest::Approx(expected.w).epsilon(1e-3));

    // unchanged input: nothing to redraw
    CHECK_FALSE(set.update(samples));
    // the robot leaves the snapshot (it dropped): hidden again
    CHECK(set.update({}));
    CHECK_FALSE(set.find(1)->present());
}

TEST_CASE("remote rigs: zones follow the activation and the pose the client commanded") {
    RemoteRigSet set;
    set.setZoneKinds(demo::zoneKinds());
    set.setRoster(rosterOf({participant(1, "Owls", kBlue1)}));
    const sdk::Pose3 rest = sdk::match::appearanceZoneRestPoses(FakeRunBackend::makeDemoAppearance())[0];

    std::vector<sdk::RobotSample> samples{sampleOf(1, {5.f, 3.f, 0.2f})};
    (void)set.update(samples);
    const ScenePart* volume = &set.find(1)->zones().parts()[0];
    CHECK(near(volume->pose.position, rest.position));   // at rest
    CHECK_FALSE(volume->highlighted);

    samples[0].zones[0].active = true;
    sdk::Pose3 moved = rest;
    moved.position.x += 0.3f;
    moved.position.z += 0.1f;
    samples[0].zones[0].poseInChassis = moved;
    CHECK(set.update(samples));
    volume = &set.find(1)->zones().parts()[0];
    CHECK(volume->highlighted);
    CHECK(near(volume->pose.position, moved.position));
    CHECK(set.changes()[0].zoneParts);

    // the override ends: the zone returns to its rest pose
    samples[0].zones[0].poseInChassis.reset();
    (void)set.update(samples);
    CHECK(near(set.find(1)->zones().parts()[0].pose.position, rest.position));
}

TEST_CASE("remote rigs: a zone of an unknown kind is left out and the others keep their place") {
    RemoteRigSet set;
    std::vector<sdk::ZoneKindInfo> onlyShooter;
    for (const auto& k : demo::zoneKinds())
        if (k.kind == demo::kShooterZone) onlyShooter.push_back(k);
    set.setZoneKinds(onlyShooter);
    set.setRoster(rosterOf({participant(1, "Owls", kBlue1)}));
    CHECK(set.at(0).zones().parts().size() == 2);   // the muzzle only

    std::vector<sdk::RobotSample> samples{sampleOf(1, {5.f, 3.f, 0.2f})};
    samples[0].zones[1].active = true;   // zone 1 is the shooter: paired by index although zone 0 is not drawn
    (void)set.update(samples);
    CHECK(set.at(0).zones().parts()[0].highlighted);
}

TEST_CASE("remote robot model: rows follow the roster and carry the alliance colour of the station") {
    RemoteRobotModel model;
    model.setZoneKinds(demo::zoneKinds());
    QSignalSpy reset(&model, &QAbstractItemModel::modelReset);

    model.setRoster(rosterOf({participant(1, "Owls", kBlue1), participant(2, "Rockets", kRed2)}));
    CHECK(model.rowCount() == 2);
    CHECK(reset.count() >= 1);
    const QModelIndex blue = model.index(0);
    const QModelIndex red = model.index(1);
    CHECK(model.data(blue, RemoteRobotModel::NameRole).toString() == QStringLiteral("Owls"));
    CHECK(model.data(blue, RemoteRobotModel::ParticipantRole).toInt() == 1);
    CHECK(model.data(blue, RemoteRobotModel::StationRole).toString() == ui::stationText(kBlue1));
    CHECK(model.data(blue, RemoteRobotModel::ColorRole).value<QColor>() == ui::allianceColor(sdk::Alliance::Blue));
    CHECK(model.data(red, RemoteRobotModel::ColorRole).value<QColor>() == ui::allianceColor(sdk::Alliance::Red));
    CHECK(ui::allianceColor(sdk::Alliance::Blue) != ui::allianceColor(sdk::Alliance::Red));
    // the plate on the chassis is drawn in the alliance colour
    REQUIRE(model.markerParts(1) != nullptr);
    CHECK(model.markerParts(1)->rowCount() == 1);
    CHECK(model.markerParts(1)->data(model.markerParts(1)->index(0), PartListModel::ColorRole).value<QColor>() ==
          ui::allianceColor(sdk::Alliance::Red));
    CHECK(model.robotParts(0)->rowCount() > 8);
    CHECK(model.zoneParts(0)->rowCount() == 3);
    CHECK_FALSE(model.data(blue, RemoteRobotModel::ShownRole).toBool());   // no sample yet

    QSignalSpy changed(&model, &QAbstractItemModel::dataChanged);
    std::vector<sdk::RobotSample> samples{sampleOf(1, {5.f, 3.f, 0.2f}), sampleOf(2, {9.f, 4.f, 0.2f})};
    model.update(samples);
    CHECK(model.data(blue, RemoteRobotModel::ShownRole).toBool());
    CHECK(model.data(red, RemoteRobotModel::PositionRole).value<QVector3D>() == QVector3D(9.f, 4.f, 0.2f));
    CHECK(changed.count() >= 2);

    // one robot leaves: one row remains and it is the right one
    model.setRoster(rosterOf({participant(2, "Rockets", kRed2)}));
    CHECK(model.rowCount() == 1);
    CHECK(model.data(model.index(0), RemoteRobotModel::NameRole).toString() == QStringLiteral("Rockets"));
    model.setRoster(nullptr);
    CHECK(model.rowCount() == 0);
}

TEST_CASE("scene model: the snapshot's remote robots drive the remote rows without a second snapshot path") {
    SceneModel scene;
    wpiemu::app::HostScene info;
    info.config = std::make_shared<const sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    info.zoneKinds = demo::zoneKinds();
    scene.setSceneInfo(info);
    scene.setMatchRoster(rosterOf({participant(1, "Owls", kBlue1)}));
    REQUIRE(scene.remoteModel().rowCount() == 1);

    sdk::SimSnapshotData d = runview_test::baseSnapshot();
    d.remoteRobots.push_back(sampleOf(1, {7.f, 2.f, 0.2f}));
    const sdk::SimSnapshot snapshot(d);
    scene.update(&snapshot);
    CHECK(scene.remoteModel().data(scene.remoteModel().index(0), RemoteRobotModel::ShownRole).toBool());
    CHECK(scene.remoteModel().rigs().at(0).pose().position.x == doctest::Approx(7.f));

    scene.update(nullptr);   // no snapshot: nobody is drawn
    CHECK_FALSE(scene.remoteModel().data(scene.remoteModel().index(0), RemoteRobotModel::ShownRole).toBool());
}

TEST_CASE("run page: robots of the host's roster appear and disappear in the scene as players join and leave") {
    QTemporaryDir dir;
    QSettings settings{dir.filePath(QStringLiteral("s.ini")), QSettings::IniFormat};
    FakeRunBackend::Options options;
    options.autoRun = false;
    options.startupDelayMs = 0;
    FakeRunBackend backend(options);
    RunPageOptions pageOptions;
    FakeJoystickProvider gamepads;
    pageOptions.gamepads = &gamepads;
    pageOptions.settings = &settings;
    RunPage page(backend, pageOptions);
    page.resize(1200, 800);
    page.show();
    QApplication::processEvents();
    CHECK_MESSAGE(page.viewport()->sceneLoaded(), page.viewport()->sceneError().toStdString());   // the QML with the remote robots parses
    CHECK(page.scene().remoteModel().rowCount() == 0);

    QSignalSpy rosterSpy(&backend, &wpiemu::app::IRuntimeHost::matchRosterChanged);
    backend.addRemoteRobot(1, QStringLiteral("Owls"), kBlue1);
    backend.addRemoteRobot(2, QStringLiteral("Rockets"), kRed2);
    CHECK(rosterSpy.count() == 2);
    CHECK(page.scene().remoteModel().rowCount() == 2);

    backend.startSimulation();
    backend.tick(3);
    page.pollNow();
    CHECK(page.scene().remoteModel().data(page.scene().remoteModel().index(1), RemoteRobotModel::ShownRole).toBool());

    backend.removeRemoteRobot(1);
    CHECK(page.scene().remoteModel().rowCount() == 1);
    backend.removeRemoteRobot(2);
    CHECK(page.scene().remoteModel().rowCount() == 0);
}
