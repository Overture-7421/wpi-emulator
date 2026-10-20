#include <doctest/doctest.h>

#include <QSignalSpy>

#include "fake_run_backend.h"
#include "state_model.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

RunSceneInfo sceneInfo() {
    RunSceneInfo info;
    info.config = std::make_shared<const sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    sdk::GameValueInfo fuel;
    fuel.id = "demo.fuel.count";
    fuel.label = "Fuel held";
    fuel.unit = sdk::PropertyUnit::Count;
    info.gameValues = {fuel};
    sdk::ZoneKindInfo kind;
    kind.kind = "demo_game.pickup";
    kind.displayName = "Pickup";
    info.zoneKinds = {kind};
    return info;
}

QString text(const StateModel& m, const QString& key, int column = 1) {
    return m.findKey(key).siblingAtColumn(column).data().toString();
}

sdk::SimSnapshotData snapshotData() {
    sdk::SimSnapshotData d;
    d.robotConnected = true;
    sdk::MechanismState intake;
    intake.id = "intake";
    intake.type = "mechanism";
    intake.active = true;
    intake.values = {{"angle", 3.14159265358979, "rad"}, {"speed", 62.8318530718, "rad/s"}, {"height", 0.5, "m"}};
    d.mechanisms.push_back(intake);
    d.sensors.push_back({"hopper_sensor", 8.0, 0.1});
    d.sensors.push_back({"feed_sensor", std::nullopt, 0.0});
    d.zones.push_back({"pickup", true});
    d.cameras.push_back({"cam_rear", {}, true, {3, 7}});
    return d;
}

}  // namespace

TEST_CASE("state model lists mechanisms, sensors, zones and cameras with readable values") {
    ViewportTheme theme;
    StateModel model(theme);
    model.setScene(sceneInfo());
    CHECK(model.isEmpty());

    const sdk::SimSnapshot snap(snapshotData());
    model.update(&snap);
    REQUIRE(model.rowCount() == 4);   // Mechanisms, Sensors, Zones, Cameras

    CHECK(text(model, "mechanism:intake", 0) == "Intake");
    CHECK(text(model, "mechanism:intake") == "active");
    CHECK(text(model, "mechanism:intake/angle") == "180.0 °");      // rad -> deg
    CHECK(text(model, "mechanism:intake/speed") == "600.0 rpm");    // rad/s -> rpm
    CHECK(text(model, "mechanism:intake/height") == "0.50 m");

    CHECK(text(model, "sensor:hopper_sensor") == "0.10 m");
    CHECK(text(model, "sensor:hopper_sensor/Source") == "Fuel held");
    CHECK(text(model, "sensor:hopper_sensor/Input") == "8");
    CHECK(text(model, "sensor:feed_sensor") == "false");
    CHECK(text(model, "sensor:feed_sensor/Input") == "—");    // source not published
    CHECK(model.findKey("sensor:feed_sensor").siblingAtColumn(1).data(StateModel::ToneRole).toInt() ==
          static_cast<int>(StateModel::Tone::Warning));

    CHECK(text(model, "zone:pickup") == "active");
    CHECK(text(model, "zone:pickup/Kind") == "Pickup");
    CHECK(text(model, "camera:cam_rear/Visible tags") == "3, 7");

    model.update(nullptr);
    CHECK(model.isEmpty());
}

TEST_CASE("state model only reports the cells that changed and keeps its rows") {
    ViewportTheme theme;
    StateModel model(theme);
    model.setScene(sceneInfo());
    const sdk::SimSnapshot first(snapshotData());
    model.update(&first);

    QSignalSpy changed(&model, &QAbstractItemModel::dataChanged);
    QSignalSpy reset(&model, &QAbstractItemModel::modelReset);
    sdk::SimSnapshotData d = snapshotData();
    d.sensors[0] = {"hopper_sensor", 2.0, 1.0};
    const sdk::SimSnapshot second(d);
    model.update(&second);
    CHECK(reset.count() == 0);
    CHECK(changed.count() == 2);   // the reading and its Input row
    CHECK(text(model, "sensor:hopper_sensor") == "1.00 m");

    model.update(&second);
    CHECK(changed.count() == 2);   // identical snapshot: nothing emitted

    d.zones.clear();
    const sdk::SimSnapshot third(d);
    model.update(&third);
    CHECK(reset.count() == 1);     // a section disappeared
    CHECK_FALSE(model.findKey("section:zones").isValid());
}
