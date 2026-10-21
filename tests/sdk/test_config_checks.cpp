#include <doctest/doctest.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/config_checks.h"
#include "wpiemu/sdk/config/property_builders.h"

using namespace wpiemu::sdk;

namespace {

class FakeCatalog final : public IConfigCatalog {
public:
    FakeCatalog() {
        MechanismTemplateInfo arm;
        arm.type = "test.arm";
        LinkSpec tip;
        tip.name = "tip";
        arm.links.push_back(tip);
        templates.push_back(arm);

        ZoneKindInfo pickup;
        pickup.kind = "game_x.pickup";
        pickup.shape = ZoneShape::Volume;
        pickup.properties.properties = {props::integer("capacity", "Capacity", "Pickup", 30, 1, 100)};
        zoneKinds.push_back(pickup);
        ZoneKindInfo muzzle;
        muzzle.kind = "game_x.muzzle";
        muzzle.shape = ZoneShape::Muzzle;
        zoneKinds.push_back(muzzle);

        GameValueInfo fuel;
        fuel.id = "game_x.fuel";
        gameValues.push_back(fuel);
    }

    const MechanismTemplateInfo* mechanismTemplate(std::string_view type) const override {
        for (const auto& t : templates)
            if (t.type == type) return &t;
        return nullptr;
    }
    std::string_view gamePluginId() const override { return pluginId; }
    const ZoneKindInfo* zoneKind(std::string_view kind) const override {
        for (const auto& k : zoneKinds)
            if (k.kind == kind) return &k;
        return nullptr;
    }
    const GameValueInfo* gameValue(std::string_view id) const override {
        for (const auto& v : gameValues)
            if (v.id == id) return &v;
        return nullptr;
    }

    std::string pluginId = "game_x";
    std::vector<MechanismTemplateInfo> templates;
    std::vector<ZoneKindInfo> zoneKinds;
    std::vector<GameValueInfo> gameValues;
};

// One mechanism ("arm", with a link "tip") and nothing else.
RobotConfig baseConfig() {
    RobotConfig c;
    MechanismConfig arm;
    arm.id = "arm";
    arm.type = "test.arm";
    c.mechanisms.push_back(arm);
    return c;
}

ZoneConfig makeZone(const std::string& id, const std::string& kind = "game_x.pickup") {
    ZoneConfig z;
    z.id = id;
    z.kind = kind;
    return z;
}

CameraConfig makeCamera(const std::string& id, const std::string& name = "") {
    CameraConfig cam;
    cam.id = id;
    cam.name = name;
    return cam;
}

SensorConfig makeSensor(const std::string& id, const std::string& source = "game_x.fuel") {
    SensorConfig s;
    s.id = id;
    s.source = source;
    return s;
}

StartPosition makeStart(const std::string& id, float x = 1.f) {
    StartPosition p;
    p.id = id;
    p.pose.xM = x;
    return p;
}

std::vector<Issue> zoneIssues(const RobotConfig& c, const IConfigCatalog& catalog) {
    std::vector<Issue> out;
    validateZones(c, catalog, out);
    return out;
}

std::vector<Issue> cameraIssues(const RobotConfig& c, const IConfigCatalog& catalog) {
    std::vector<Issue> out;
    validateCameras(c, catalog, out);
    return out;
}

std::vector<Issue> sensorIssues(const RobotConfig& c, const IConfigCatalog& catalog) {
    std::vector<Issue> out;
    validateSensors(c, catalog, out);
    return out;
}

bool has(const std::vector<Issue>& issues, const char* code, const std::string& path, Severity severity = Severity::Error) {
    return std::any_of(issues.begin(), issues.end(),
                       [&](const Issue& i) { return i.code == code && i.path == path && i.severity == severity; });
}

const Issue* find(const std::vector<Issue>& issues, const char* code) {
    const auto it = std::find_if(issues.begin(), issues.end(), [&](const Issue& i) { return i.code == code; });
    return it == issues.end() ? nullptr : &*it;
}

}  // namespace

TEST_CASE("start positions need a unique id and a finite pose") {
    RobotConfig c;
    c.startPositions = {makeStart("a"), makeStart("a"), makeStart(""), makeStart("inf", std::numeric_limits<float>::infinity())};
    std::vector<Issue> out;
    validateStartPositions(c, out);
    CHECK_FALSE(has(out, validation_code::kStartPositionIdDuplicate, "startPositions[0].id"));
    CHECK(has(out, validation_code::kStartPositionIdDuplicate, "startPositions[1].id"));
    CHECK(has(out, validation_code::kStartPositionIdInvalid, "startPositions[2].id"));
    CHECK(has(out, validation_code::kStartPositionInvalid, "startPositions[3].pose"));
    CHECK(out.size() == 3);
}

TEST_CASE("zone ids must be set and distinct from zones, mechanisms and cameras") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    c.cameras.push_back(makeCamera("cam"));
    c.zones = {makeZone(""), makeZone("arm"), makeZone("cam"), makeZone("z"), makeZone("z")};
    const auto issues = zoneIssues(c, catalog);
    CHECK(has(issues, validation_code::kZoneIdInvalid, "zones[0].id"));
    CHECK(has(issues, validation_code::kZoneIdDuplicate, "zones[1].id"));
    CHECK(has(issues, validation_code::kZoneIdDuplicate, "zones[2].id"));
    CHECK_FALSE(has(issues, validation_code::kZoneIdDuplicate, "zones[3].id"));
    CHECK(has(issues, validation_code::kZoneIdDuplicate, "zones[4].id"));
}

TEST_CASE("a zone kind the plugin does not own is an error but an owned undeclared kind is a warning") {
    FakeCatalog catalog;
    RobotConfig c = baseConfig();
    c.zones = {makeZone("foreign", "other.pickup"), makeZone("undeclared", "game_x.mystery"), makeZone("ok")};
    const auto issues = zoneIssues(c, catalog);
    CHECK(has(issues, validation_code::kZoneKindNotOwned, "zones[0].kind"));
    CHECK(has(issues, validation_code::kZoneKindUnknown, "zones[1].kind", Severity::Warning));
    CHECK_FALSE(has(issues, validation_code::kZoneKindNotOwned, "zones[1].kind"));
    CHECK(issues.size() == 2);

    catalog.pluginId.clear();
    const auto noPlugin = zoneIssues(c, catalog);
    CHECK(has(noPlugin, validation_code::kZoneKindNotOwned, "zones[2].kind"));
    REQUIRE(find(noPlugin, validation_code::kZoneKindNotOwned) != nullptr);
    CHECK(find(noPlugin, validation_code::kZoneKindNotOwned)->message.find("needs a game plugin") != std::string::npos);
}

TEST_CASE("zone properties follow the kind schema and only a Volume needs a size") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    ZoneConfig bad = makeZone("bad");
    bad.properties["capacity"] = 1000;
    ZoneConfig flat = makeZone("flat");
    flat.sizeM.y = 0.f;
    ZoneConfig muzzle = makeZone("muzzle", "game_x.muzzle");
    muzzle.sizeM = {0.f, 0.f, 0.f};
    c.zones = {bad, flat, muzzle};
    const auto issues = zoneIssues(c, catalog);
    CHECK(has(issues, validation_code::kZonePropertyInvalid, "zones[0].properties.capacity"));
    CHECK(has(issues, validation_code::kZoneGeometryInvalid, "zones[1].sizeM"));
    CHECK_FALSE(has(issues, validation_code::kZoneGeometryInvalid, "zones[2].sizeM"));
}

TEST_CASE("zone activation conditions name a value, a finite threshold and a known mechanism") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    ZoneConfig z = makeZone("z");
    ZoneCondition ghost;
    ghost.mechanismId = "ghost";
    ZoneCondition unnamed;
    unnamed.mechanismId = "arm";
    unnamed.value.clear();
    z.activation.conditions = {ghost, unnamed};
    c.zones = {z};
    const auto issues = zoneIssues(c, catalog);
    CHECK(has(issues, validation_code::kZoneConditionMechanismUnknown, "zones[0].activation.conditions[0].mechanism", Severity::Warning));
    CHECK(has(issues, validation_code::kZoneConditionInvalid, "zones[0].activation.conditions[1].value"));
    CHECK(issues.size() == 2);
}

TEST_CASE("zones and cameras attach to the chassis root or to a link of an existing mechanism") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    const auto attach = [](const std::string& parent, const std::string& link) {
        MechanismAttachment a;
        a.parentMechanismId = parent;
        a.parentLink = link;
        return a;
    };
    const std::vector<MechanismAttachment> bad = {attach("ghost", "root"), attach("arm", "nolink"), attach("chassis", "tip"), attach("arm", "")};
    const std::vector<const char*> codes = {validation_code::kAttachmentParentMissing, validation_code::kAttachmentLinkMissing,
                                            validation_code::kAttachmentLinkMissing, validation_code::kAttachmentLinkMissing};
    for (std::size_t i = 0; i < bad.size(); ++i) {
        ZoneConfig z = makeZone("z" + std::to_string(i));
        z.attachment = bad[i];
        c.zones.push_back(z);
        CameraConfig cam = makeCamera("c" + std::to_string(i), "n" + std::to_string(i));
        cam.attachment = bad[i];
        c.cameras.push_back(cam);
    }
    ZoneConfig onTip = makeZone("onTip");
    onTip.attachment = attach("arm", "tip");
    c.zones.push_back(onTip);

    const auto zones = zoneIssues(c, catalog);
    const auto cameras = cameraIssues(c, catalog);
    for (std::size_t i = 0; i < bad.size(); ++i) {
        CHECK_MESSAGE(has(zones, codes[i], "zones[" + std::to_string(i) + "].attachment"), i);
        CHECK_MESSAGE(has(cameras, codes[i], "cameras[" + std::to_string(i) + "].attachment"), i);
    }
    CHECK_FALSE(has(zones, validation_code::kAttachmentLinkMissing, "zones[4].attachment"));
}

TEST_CASE("camera ids are distinct from mechanisms and zones and a robot-code name is unique per kind") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    c.zones.push_back(makeZone("zone"));
    CameraConfig otherKind = makeCamera("e", "same");
    otherKind.kind = CameraKind::PhotonVision;
    c.cameras = {makeCamera(""), makeCamera("arm", "a"), makeCamera("zone", "b"), makeCamera("d", "same"), makeCamera("d2", "same"),
                 otherKind, makeCamera("d", "other")};
    const auto issues = cameraIssues(c, catalog);
    CHECK(has(issues, validation_code::kCameraIdInvalid, "cameras[0].id"));
    CHECK(has(issues, validation_code::kCameraIdDuplicate, "cameras[1].id"));
    CHECK(has(issues, validation_code::kCameraIdDuplicate, "cameras[2].id"));
    CHECK(has(issues, validation_code::kCameraNameDuplicate, "cameras[4].name"));
    CHECK_FALSE(has(issues, validation_code::kCameraNameDuplicate, "cameras[5].name"));
    CHECK(has(issues, validation_code::kCameraIdDuplicate, "cameras[6].id"));
}

TEST_CASE("camera optics are bounded and the boundary values are valid") {
    const FakeCatalog catalog;
    constexpr double notANumber = std::numeric_limits<double>::quiet_NaN();
    RobotConfig c = baseConfig();
    CameraConfig bad = makeCamera("bad");
    bad.hFovDeg = 180.0;
    bad.vFovDeg = 0.0;
    bad.maxRangeM = 0.0;
    bad.maxIncidenceDeg = 91.0;
    bad.fps = 241.0;
    bad.latencyMs = 2001.0;
    bad.widthPx = 0;
    bad.heightPx = -1;
    bad.positionNoiseM = -0.1;
    bad.yawNoiseDeg = notANumber;
    bad.local.position.x = std::numeric_limits<float>::quiet_NaN();
    CameraConfig edge = makeCamera("edge", "edge");
    edge.maxIncidenceDeg = 90.0;
    edge.fps = 240.0;
    edge.latencyMs = 0.0;
    c.cameras = {bad, edge};
    const auto issues = cameraIssues(c, catalog);
    for (const char* field : {"hFovDeg", "vFovDeg", "maxRangeM", "maxIncidenceDeg", "fps", "latencyMs", "widthPx", "heightPx",
                              "positionNoiseM", "yawNoiseDeg"})
        CHECK_MESSAGE(has(issues, validation_code::kCameraOpticsInvalid, std::string("cameras[0].") + field), field);
    CHECK(has(issues, validation_code::kCameraGeometryInvalid, "cameras[0].pose"));
    for (const Issue& i : issues) CHECK_MESSAGE(i.path.find("cameras[1]") == std::string::npos, i.path);
}

TEST_CASE("sensor ids are not reserved or dotted and are distinct from every other id") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    c.zones.push_back(makeZone("zone"));
    c.cameras.push_back(makeCamera("cam"));
    c.sensors = {makeSensor(""), makeSensor("drivetrain"), makeSensor("a.b"), makeSensor("arm"), makeSensor("zone"), makeSensor("cam"),
                 makeSensor("s"), makeSensor("s")};
    const auto issues = sensorIssues(c, catalog);
    for (const char* path : {"sensors[0].id", "sensors[1].id", "sensors[2].id"})
        CHECK_MESSAGE(has(issues, validation_code::kSensorIdInvalid, path), path);
    for (const char* path : {"sensors[3].id", "sensors[4].id", "sensors[5].id", "sensors[7].id"})
        CHECK_MESSAGE(has(issues, validation_code::kSensorIdDuplicate, path), path);
    CHECK_FALSE(has(issues, validation_code::kSensorIdDuplicate, "sensors[6].id"));
}

TEST_CASE("a sensor source is a warning when missing or undeclared and an error when the plugin does not own it") {
    FakeCatalog catalog;
    RobotConfig c = baseConfig();
    c.sensors = {makeSensor("none", ""), makeSensor("foreign", "other.fuel"), makeSensor("undeclared", "game_x.mystery"), makeSensor("ok")};
    const auto issues = sensorIssues(c, catalog);
    CHECK(has(issues, validation_code::kSensorSourceMissing, "sensors[0].source", Severity::Warning));
    CHECK(has(issues, validation_code::kSensorSourceNotOwned, "sensors[1].source"));
    CHECK(has(issues, validation_code::kSensorSourceUnknown, "sensors[2].source", Severity::Warning));
    CHECK(issues.size() == 3);

    catalog.pluginId.clear();
    const auto noPlugin = sensorIssues(c, catalog);
    CHECK(has(noPlugin, validation_code::kSensorSourceNotOwned, "sensors[3].source"));
    CHECK(has(noPlugin, validation_code::kSensorSourceNotOwned, "sensors[2].source"));
}

TEST_CASE("a linear distance sensor needs a non-empty input range") {
    const FakeCatalog catalog;
    RobotConfig c = baseConfig();
    SensorConfig s = makeSensor("range");
    s.properties[sensor_prop::kMode] = sensor_prop::kModeLinear;
    s.properties[sensor_prop::kInputMin] = 2.0;
    s.properties[sensor_prop::kInputMax] = 2.0;
    c.sensors = {s};
    CHECK(has(sensorIssues(c, catalog), validation_code::kSensorPropertyInvalid, "sensors[0].properties.inputMax"));
    c.sensors[0].properties[sensor_prop::kInputMax] = 3.0;
    CHECK(sensorIssues(c, catalog).empty());
}
