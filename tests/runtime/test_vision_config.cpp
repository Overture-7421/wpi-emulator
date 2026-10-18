#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::config;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;

namespace {

// A camera with every field away from its default.
CameraConfig fullCamera() {
    CameraConfig c;
    c.id = "left";
    c.label = "Left Limelight";
    c.vendor = CameraVendor::Limelight;
    c.ntName = "limelight-left";
    c.enabled = false;
    c.attachment.parentMechanismId = "intake";
    c.attachment.parentLink = "tip";
    c.local = Pose3{Vec3{0.25f, -0.3f, 0.45f}, Quat::fromYaw(0.5f)};
    c.intrinsics.widthPx = 1920;
    c.intrinsics.heightPx = 1080;
    c.intrinsics.hfovRad = 1.1;
    c.intrinsics.fxPx = 1450.5;
    c.intrinsics.fyPx = 1451.25;
    c.intrinsics.cxPx = 960.5;
    c.intrinsics.cyPx = 539.75;
    c.intrinsics.distortion = {0.1, -0.2, 0.001, -0.002, 0.03};
    c.frameRateHz = 59.94;
    c.latencyUs = 31'500;
    c.captureLatencyUs = 12'250;
    c.tagSizeM = 0.1524;
    c.minTagPixels = 8.5;
    c.maxViewAngleRad = 1.2;
    c.occlusion = false;
    c.multiTag = false;
    c.realism.cornerNoisePx = 0.35;
    c.realism.latencyJitterUs = 4'000;
    c.realism.dropoutProbability = 0.02;
    c.assumedPose = Pose3{Vec3{0.2f, -0.25f, 0.5f}, Quat::fromYaw(-0.4f)};
    return c;
}

CameraConfig camera(const std::string& id) {
    CameraConfig c;
    c.id = id;
    return c;
}

std::vector<std::string> keysOf(const Value& object) {
    std::vector<std::string> keys;
    for (std::size_t i = 0; i < object.object().size(); ++i)
        keys.push_back(object.object().keyAt(i));
    return keys;
}

std::string decodeError(const Value& v) {
    const auto r = RobotConfigCodec{}.fromValue(v);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::ParseError);
    return r.error().message;
}

// A stored config with one default camera, for tests that corrupt single fields.
Value storedWithCamera() {
    RobotConfig c = sampleConfig();
    c.vision.cameras = {camera("front")};
    return RobotConfigCodec{}.toValue(c);
}

std::vector<Issue> validate(const RobotConfig& c, Registries& r,
                            const ConfigValidator& v = ConfigValidator{}) {
    if (IGameModule* p = selectPlugin(c, r.plugins)) REQUIRE(r.registrar.ensureRegistered(*p).ok());
    return v.validate(c, r.mechanisms, r.drivetrains, r.plugins);
}

// Issues with this code, of this severity, at exactly this path.
std::ptrdiff_t count(const std::vector<Issue>& issues, const char* code, const std::string& path,
                     Severity severity = Severity::Error) {
    return std::count_if(issues.begin(), issues.end(), [&](const Issue& i) {
        return i.code == code && i.path == path && i.severity == severity;
    });
}

RobotConfig withCameras(std::vector<CameraConfig> cameras) {
    RobotConfig c = sampleConfig();
    c.vision.cameras = std::move(cameras);
    return c;
}

// A game plugin that offers two AprilTag layouts.
class LayoutGameModule final : public wpiemu::sdk::testing::FakeGameModule {
public:
    using FakeGameModule::FakeGameModule;
    std::vector<FiducialLayout> contributeFiducialLayouts() const override {
        FiducialLayout welded;
        welded.id = "fake_game.welded";
        FiducialLayout andymark;
        andymark.id = "fake_game.andymark";
        return {welded, andymark};
    }
};

}  // namespace

TEST_CASE("vision config: every field survives config -> Value -> JSON text -> Value -> config") {
    RobotConfig c = sampleConfig();
    c.vision.layoutId = "game_2026.welded";
    c.vision.layoutFile = "C:/fields/custom.json";
    c.vision.photonMessageUuid = "4b2ff16a964b5e2bf04be0c1454d91c4";
    c.vision.cameras = {fullCamera(), camera("front")};

    const RobotConfigCodec codec;
    const JsonValueCodec json;
    const Value v = codec.toValue(c);

    const Value* vision = v.find("vision");
    REQUIRE(vision != nullptr);
    CHECK(keysOf(*vision) ==
          std::vector<std::string>{"layoutId", "layoutFile", "photonMessageUuid", "cameras"});
    const Value& left = vision->find("cameras")->array()[0];
    CHECK(keysOf(left) == std::vector<std::string>{
                              "id", "label", "vendor", "ntName", "enabled", "attachment", "local",
                              "intrinsics", "frameRateHz", "latencyUs", "captureLatencyUs",
                              "tagSizeM", "minTagPixels", "maxViewAngleRad", "occlusion",
                              "multiTag", "realism", "assumedPose"});
    CHECK(keysOf(*left.find("attachment")) ==
          std::vector<std::string>{"parentMechanismId", "parentLink"});
    CHECK(keysOf(*left.find("intrinsics")) ==
          std::vector<std::string>{"widthPx", "heightPx", "hfovRad", "fxPx", "fyPx", "cxPx", "cyPx",
                                   "distortion"});
    CHECK(keysOf(*left.find("realism")) ==
          std::vector<std::string>{"cornerNoisePx", "latencyJitterUs", "dropoutProbability"});
    CHECK(left.find("vendor")->asString() == "limelight");
    CHECK(left.find("intrinsics")->find("distortion")->array().size() == 5);
    const Value& front = vision->find("cameras")->array()[1];
    CHECK(front.find("vendor")->asString() == "photonvision");
    CHECK(front.find("attachment")->find("parentMechanismId")->asString() == "chassis");
    CHECK(front.find("assumedPose") == nullptr);  // unset: not written

    const auto parsed = json.parse(json.dump(v, true));
    REQUIRE(parsed.ok());
    const auto back = codec.fromValue(parsed.value());
    REQUIRE_MESSAGE(back.ok(), (back.ok() ? "" : back.error().message));
    CHECK(back.value().vision == c.vision);
    REQUIRE(back.value().findCamera("left") != nullptr);
    CHECK(back.value().findCamera("left")->assumedPose.has_value());
    CHECK_FALSE(back.value().findCamera("front")->assumedPose.has_value());
    CHECK(json.dump(codec.toValue(back.value()), true) == json.dump(v, true));
}

TEST_CASE("vision config: an absent vision section or absent camera fields read as the defaults") {
    const RobotConfigCodec codec;
    Value v = codec.toValue(sampleConfig());
    REQUIRE(v.find("vision") != nullptr);  // always written
    CHECK(v.find("vision")->find("cameras")->array().empty());
    v.object().erase("vision");
    const auto none = codec.fromValue(v);
    REQUIRE(none.ok());
    CHECK(none.value().vision == VisionConfig{});
    CHECK(none.value().vision.empty());

    Value minimal = Value::makeObject();
    minimal["schemaVersion"] = 1;
    minimal["vision"]["cameras"].push_back(Value::makeObject());
    const auto bare = codec.fromValue(minimal);
    REQUIRE_MESSAGE(bare.ok(), (bare.ok() ? "" : bare.error().message));
    REQUIRE(bare.value().vision.cameras.size() == 1);
    CHECK(bare.value().vision.cameras[0] == CameraConfig{});

    Value partial = Value::makeObject();
    partial["schemaVersion"] = 1;
    Value cam = Value::makeObject();
    cam["id"] = "a";
    cam["intrinsics"]["hfovRad"] = 1.0;
    cam["assumedPose"] = Value{};  // null reads as unset
    partial["vision"]["layoutId"] = "x";
    partial["vision"]["cameras"].push_back(std::move(cam));
    const auto some = codec.fromValue(partial);
    REQUIRE_MESSAGE(some.ok(), (some.ok() ? "" : some.error().message));
    CHECK(some.value().vision.layoutId == "x");
    const CameraConfig& a = some.value().vision.cameras[0];
    CHECK(a.id == "a");
    CHECK(a.intrinsics.hfovRad == 1.0);
    CHECK(a.intrinsics.widthPx == 1280);  // the rest keeps its default
    CHECK(a.frameRateHz == 30.0);
    CHECK_FALSE(a.assumedPose.has_value());
}

TEST_CASE("vision config: an unknown vendor and mistyped fields are reported with their path") {
    Value v = storedWithCamera();
    v["vision"]["cameras"].array()[0]["vendor"] = "orangepi";
    const std::string vendor = decodeError(v);
    CHECK(vendor.find("vision.cameras[0].vendor") != std::string::npos);
    CHECK(vendor.find("'orangepi'") != std::string::npos);
    CHECK(vendor.find("photonvision|limelight") != std::string::npos);

    v = storedWithCamera();
    v["vision"]["cameras"].array()[0]["frameRateHz"] = "fast";
    CHECK(decodeError(v).find("vision.cameras[0].frameRateHz") != std::string::npos);

    v = storedWithCamera();
    v["vision"]["cameras"].array()[0]["latencyUs"] = 12.5;  // microseconds are integers
    CHECK(decodeError(v).find("vision.cameras[0].latencyUs") != std::string::npos);

    v = storedWithCamera();
    v["vision"]["cameras"].array()[0]["intrinsics"]["widthPx"] = 1e12;
    CHECK(decodeError(v).find("vision.cameras[0].intrinsics.widthPx") != std::string::npos);

    v = storedWithCamera();
    Value fourCoefficients = Value::makeArray();
    for (int i = 0; i < 4; ++i) fourCoefficients.push_back(0.0);
    v["vision"]["cameras"].array()[0]["intrinsics"]["distortion"] = fourCoefficients;
    CHECK(decodeError(v).find("vision.cameras[0].intrinsics.distortion") != std::string::npos);

    v = storedWithCamera();
    v["vision"]["cameras"].array()[0]["intrinsics"]["distortion"].array()[2] = "k";
    CHECK(decodeError(v).find("vision.cameras[0].intrinsics.distortion[2]") != std::string::npos);

    v = storedWithCamera();
    v["vision"]["cameras"].array()[0]["assumedPose"] = "origin";
    CHECK(decodeError(v).find("vision.cameras[0].assumedPose") != std::string::npos);

    v = storedWithCamera();
    v["vision"]["cameras"] = 5;
    CHECK(decodeError(v).find("vision.cameras: expected an array") != std::string::npos);

    v = storedWithCamera();
    v["vision"] = "none";
    CHECK(decodeError(v).find("vision: expected an object") != std::string::npos);
}

TEST_CASE("vision config: numbers are written and read with the classic locale") {
    struct CommaPunctuation final : std::numpunct<char> {
        char do_decimal_point() const override { return ','; }
        char do_thousands_sep() const override { return '.'; }
        std::string do_grouping() const override { return "\3"; }
    };
    struct RestoreLocale {
        std::locale previous;
        ~RestoreLocale() { std::locale::global(previous); }
    } restore{std::locale()};
    std::locale::global(std::locale(std::locale::classic(), new CommaPunctuation));
    std::ostringstream processFormatted;
    processFormatted << 1234.5;
    REQUIRE(processFormatted.str() == "1.234,5");

    RobotConfig c = sampleConfig();
    c.vision.cameras = {fullCamera()};
    const RobotConfigCodec codec;
    const JsonValueCodec json;
    const std::string text = json.dump(codec.toValue(c), true);
    CHECK(text.find("\"frameRateHz\": 59.94") != std::string::npos);
    CHECK(text.find("\"hfovRad\": 1.1") != std::string::npos);
    CHECK(text.find("\"fxPx\": 1450.5") != std::string::npos);
    CHECK(text.find("\"latencyUs\": 31500") != std::string::npos);
    CHECK(text.find("1,1") == std::string::npos);

    const auto parsed = json.parse(text);
    REQUIRE(parsed.ok());
    const auto back = codec.fromValue(parsed.value());
    REQUIRE_MESSAGE(back.ok(), (back.ok() ? "" : back.error().message));
    CHECK(back.value().vision == c.vision);
}

TEST_CASE(
    "vision validator: cameras with sound parameters are clean on the chassis and on a mechanism "
    "link") {
    Registries reg;
    CameraConfig turret = fullCamera();
    turret.id = "turret";
    turret.enabled = true;
    turret.ntName = "limelight-turret";
    ZoneConfig zone;
    zone.id = "pickup";
    zone.kind = "fake_game.fake_zone";
    RobotConfig c = withCameras({camera("front"), turret});
    c.zones.push_back(zone);
    c.vision.layoutId = "anything";  // no plugin layouts to judge it against
    const auto issues = validate(c, reg);
    for (const Issue& i : issues) MESSAGE(i.path << ": " << i.message << " [" << i.code << "]");
    CHECK(issues.empty());
}

TEST_CASE(
    "vision validator: camera ids are non-empty, not reserved and unique across cameras, "
    "mechanisms and zones") {
    Registries reg;
    ZoneConfig zone;
    zone.id = "pickup";
    zone.kind = "fake_game.fake_zone";
    RobotConfig c =
        withCameras({camera(""), camera("chassis"), camera("root"), camera("dup"), camera("dup"),
                     camera("intake"), camera("pickup"), camera("fine")});
    c.zones.push_back(zone);
    const auto issues = validate(c, reg);
    CHECK(count(issues, validation_code::kCameraIdReserved, "vision.cameras[0].id") == 1);
    CHECK(count(issues, validation_code::kCameraIdReserved, "vision.cameras[1].id") == 1);
    CHECK(count(issues, validation_code::kCameraIdReserved, "vision.cameras[2].id") == 1);
    CHECK(count(issues, validation_code::kCameraIdDuplicate, "vision.cameras[3].id") ==
          0);  // the first one is fine
    CHECK(count(issues, validation_code::kCameraIdDuplicate, "vision.cameras[4].id") == 1);
    CHECK(count(issues, validation_code::kCameraIdDuplicate, "vision.cameras[5].id") ==
          1);  // a mechanism id
    CHECK(count(issues, validation_code::kCameraIdDuplicate, "vision.cameras[6].id") ==
          1);  // a zone id
    CHECK(count(issues, validation_code::kCameraIdDuplicate, "vision.cameras[7].id") == 0);
    CHECK(count(issues, validation_code::kCameraIdReserved, "vision.cameras[7].id") == 0);
}

TEST_CASE(
    "vision validator: an enabled camera cannot share its vendor and network name with another "
    "enabled camera") {
    Registries reg;
    const auto named = [](const std::string& id, CameraVendor vendor, const std::string& ntName,
                          bool enabled = true) {
        CameraConfig c = camera(id);
        c.vendor = vendor;
        c.ntName = ntName;
        c.enabled = enabled;
        return c;
    };
    const auto issues = validate(
        withCameras({named("a", CameraVendor::PhotonVision, "shared"),
                     named("b", CameraVendor::PhotonVision, "shared"),  // 1: same vendor and name
                     named("ll1", CameraVendor::Limelight, ""),         // "limelight"
                     named("ll2", CameraVendor::Limelight, ""),         // 3: "limelight" again
                     named("shared", CameraVendor::PhotonVision, ""),   // 4: the id is the name
                     named("ll3", CameraVendor::Limelight, "shared"),   // other vendor: fine
                     named("d", CameraVendor::PhotonVision, "shared", false)}),  // disabled: fine
        reg);
    const auto path = [](int i) { return "vision.cameras[" + std::to_string(i) + "].ntName"; };
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(0)) == 0);
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(1)) == 1);
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(2)) == 0);
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(3)) == 1);
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(4)) == 1);
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(5)) == 0);
    CHECK(count(issues, validation_code::kCameraNameDuplicate, path(6)) == 0);
}

TEST_CASE("vision validator: the attachment follows the same link rules as zones") {
    Registries reg;
    CameraConfig noParent = camera("noParent");
    noParent.attachment.parentMechanismId = "ghost";
    CameraConfig noLink = camera("noLink");
    noLink.attachment.parentMechanismId = "intake";
    noLink.attachment.parentLink = "nolink";
    CameraConfig chassisLink = camera("chassisLink");
    chassisLink.attachment.parentLink = "tip";
    CameraConfig emptyLink = camera("emptyLink");
    emptyLink.attachment.parentLink = "";
    CameraConfig onTip = camera("onTip");
    onTip.attachment.parentMechanismId = "intake";
    onTip.attachment.parentLink = "tip";

    const auto issues =
        validate(withCameras({noParent, noLink, chassisLink, emptyLink, onTip}), reg);
    CHECK(count(issues, validation_code::kAttachmentParentMissing,
                "vision.cameras[0].attachment") == 1);
    CHECK(count(issues, validation_code::kAttachmentLinkMissing, "vision.cameras[1].attachment") ==
          1);
    CHECK(count(issues, validation_code::kAttachmentLinkMissing, "vision.cameras[2].attachment") ==
          1);
    CHECK(count(issues, validation_code::kAttachmentLinkMissing, "vision.cameras[3].attachment") ==
          1);
    for (const Issue& i : issues) CHECK(i.path != "vision.cameras[4].attachment");
}

TEST_CASE("vision validator: intrinsics, rate, latency, tag and realism limits") {
    Registries reg;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // The mutation must produce exactly one issue with this code at this path.
    const auto bad = [&](const char* code, const std::string& path, auto mutate) {
        CameraConfig cam = camera("cam");
        mutate(cam);
        const auto issues = validate(withCameras({cam}), reg);
        CHECK_MESSAGE(count(issues, code, "vision.cameras[0]." + path) == 1, path);
    };
    // The mutation is within the limits: no issue at all.
    const auto fine = [&](auto mutate) {
        CameraConfig cam = camera("cam");
        mutate(cam);
        const auto issues = validate(withCameras({cam}), reg);
        for (const Issue& i : issues) MESSAGE(i.path << ": " << i.message << " [" << i.code << "]");
        CHECK(issues.empty());
    };

    using namespace validation_code;
    bad(kCameraIntrinsicsInvalid, "intrinsics.widthPx",
        [](CameraConfig& c) { c.intrinsics.widthPx = 0; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.heightPx",
        [](CameraConfig& c) { c.intrinsics.heightPx = 20000; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.hfovRad",
        [](CameraConfig& c) { c.intrinsics.hfovRad = 0.0; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.hfovRad",
        [&](CameraConfig& c) { c.intrinsics.hfovRad = nan; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.hfovRad",
        [](CameraConfig& c) { c.intrinsics.hfovRad = 3.2; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.fxPx",
        [](CameraConfig& c) { c.intrinsics.fxPx = -1.0; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.fyPx",
        [&](CameraConfig& c) { c.intrinsics.fyPx = nan; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.cxPx",
        [](CameraConfig& c) { c.intrinsics.cxPx = -0.5; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.cyPx",
        [](CameraConfig& c) { c.intrinsics.cyPx = -0.5; });
    bad(kCameraIntrinsicsInvalid, "intrinsics.distortion",
        [&](CameraConfig& c) { c.intrinsics.distortion[3] = nan; });
    fine([](CameraConfig& c) {
        c.intrinsics.hfovRad = -7.0;
        c.intrinsics.fxPx = 800.0;
    });  // the FOV is not used when fxPx is set
    fine([](CameraConfig& c) {
        c.intrinsics.widthPx = 16384;
        c.intrinsics.heightPx = 1;
    });

    bad(kCameraFrameRateInvalid, "frameRateHz", [](CameraConfig& c) { c.frameRateHz = 0.0; });
    bad(kCameraFrameRateInvalid, "frameRateHz", [](CameraConfig& c) { c.frameRateHz = 240.5; });
    bad(kCameraFrameRateInvalid, "frameRateHz", [&](CameraConfig& c) { c.frameRateHz = nan; });
    fine([](CameraConfig& c) { c.frameRateHz = 240.0; });
    fine([](CameraConfig& c) { c.frameRateHz = 0.25; });

    bad(kCameraLatencyInvalid, "latencyUs", [](CameraConfig& c) { c.latencyUs = -1; });
    bad(kCameraLatencyInvalid, "latencyUs", [](CameraConfig& c) { c.latencyUs = 1'000'001; });
    bad(kCameraLatencyInvalid, "captureLatencyUs",
        [](CameraConfig& c) { c.captureLatencyUs = -1; });
    fine([](CameraConfig& c) {
        c.latencyUs = 0;
        c.captureLatencyUs = 0;
    });
    fine([](CameraConfig& c) { c.latencyUs = 1'000'000; });
    fine([](CameraConfig& c) {
        c.latencyUs = 5'000;
        c.captureLatencyUs = 50'000;
    });  // the capture share is clamped to the latency at run time

    bad(kCameraTagParamsInvalid, "tagSizeM", [](CameraConfig& c) { c.tagSizeM = -0.1; });
    bad(kCameraTagParamsInvalid, "tagSizeM", [&](CameraConfig& c) { c.tagSizeM = nan; });
    bad(kCameraTagParamsInvalid, "minTagPixels", [](CameraConfig& c) { c.minTagPixels = -1.0; });
    bad(kCameraTagParamsInvalid, "maxViewAngleRad",
        [](CameraConfig& c) { c.maxViewAngleRad = 0.0; });
    bad(kCameraTagParamsInvalid, "maxViewAngleRad",
        [](CameraConfig& c) { c.maxViewAngleRad = 3.2; });
    fine([](CameraConfig& c) {
        c.tagSizeM = 0.0;
        c.minTagPixels = 0.0;
    });  // 0 = the layout's size, no minimum
    fine([](CameraConfig& c) { c.maxViewAngleRad = 1.5708; });
    fine([](CameraConfig& c) { c.maxViewAngleRad = kPi; });

    bad(kCameraRealismInvalid, "realism.cornerNoisePx",
        [](CameraConfig& c) { c.realism.cornerNoisePx = -0.1; });
    bad(kCameraRealismInvalid, "realism.latencyJitterUs",
        [](CameraConfig& c) { c.realism.latencyJitterUs = -1; });
    bad(kCameraRealismInvalid, "realism.latencyJitterUs",
        [](CameraConfig& c) { c.realism.latencyJitterUs = 25'001; });  // above the 25 ms latency
    bad(kCameraRealismInvalid, "realism.dropoutProbability",
        [](CameraConfig& c) { c.realism.dropoutProbability = 1.01; });
    bad(kCameraRealismInvalid, "realism.dropoutProbability",
        [](CameraConfig& c) { c.realism.dropoutProbability = -0.01; });
    fine([](CameraConfig& c) {
        c.realism.cornerNoisePx = 2.0;
        c.realism.latencyJitterUs = 25'000;
        c.realism.dropoutProbability = 1.0;
    });
}

TEST_CASE(
    "vision validator: a layout id the selected plugin does not provide is a warning, and only "
    "then") {
    Registries reg(false);
    REQUIRE(reg.plugins.add(std::make_unique<LayoutGameModule>("fake_game", 2999)).ok());
    RobotConfig c = withCameras({camera("front")});

    const auto warns = [&](const RobotConfig& config) {
        const auto issues = validate(config, reg);
        CHECK_FALSE(ConfigValidator::hasErrors(issues));
        return count(issues, validation_code::kCameraLayoutUnknown, "vision.layoutId",
                     Severity::Warning) == 1;
    };
    CHECK_FALSE(warns(c));  // "" = the plugin's first layout
    c.vision.layoutId = "fake_game.andymark";
    CHECK_FALSE(warns(c));
    c.vision.layoutId = "fake_game.typo";
    CHECK(warns(c));
    c.vision.layoutFile = "C:/fields/custom.json";  // a layout file replaces the id
    CHECK_FALSE(warns(c));
    c.vision.layoutFile.clear();
    c.vision.cameras.clear();  // no cameras: vision is inert and not judged
    CHECK_FALSE(warns(c));

    Registries none(false);  // no plugin selected: nothing to judge the id against
    RobotConfig noPlugin = withCameras({camera("front")});
    noPlugin.vision.layoutId = "fake_game.typo";
    CHECK(validate(noPlugin, none).empty());
}

TEST_CASE(
    "vision pipeline: the vision section survives the load pipeline and its problems come back as "
    "issues") {
    Registries reg;
    RobotConfigCodec codec;
    ConfigValidator validator;
    ConfigPipeline pipeline(codec, validator, reg.registrar, reg.drivetrains, reg.plugins);

    RobotConfig c = withCameras({fullCamera(), camera("front")});
    c.vision.layoutFile = "fields/custom.json";
    const auto loaded = pipeline.load(codec.toValue(c));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    CHECK(loaded.value().config.vision == c.vision);
    CHECK(loaded.value().valid());

    c.vision.cameras[1].frameRateHz = 500.0;
    const auto invalid = pipeline.load(codec.toValue(c));
    REQUIRE(invalid.ok());
    CHECK_FALSE(invalid.value().valid());
    CHECK(count(invalid.value().issues, validation_code::kCameraFrameRateInvalid,
                "vision.cameras[1].frameRateHz") == 1);
}
