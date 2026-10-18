#include <doctest/doctest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "photon_test_support.h"
#include "wpiemu/bridges/vision/photon_codec.h"
#include "wpiemu/bridges/vision/photon_mapper.h"
#include "wpiemu/bridges/vision/photon_profile.h"

namespace {

namespace sdk = wpiemu::sdk;
using namespace wpiemu::bridges::vision;
using wpiemu::sdk::ErrorCode;
using wpiemu::tests::photon::loadFixtures;
using wpiemu::tests::photon::PhotonFixture;

PhotonCodec codecFor(const PhotonProfile& profile) {
    auto codec = PhotonCodec::create(profile);
    REQUIRE_MESSAGE(codec.ok(), wpiemu::tests::photon::why(codec));
    return std::move(codec).value();
}

PhotonCodec newestCodec() {
    return codecFor(newestPhotonProfile());
}

std::string firstDifference(const std::vector<std::uint8_t>& expected, const std::vector<std::uint8_t>& actual) {
    if (expected.size() != actual.size()) {
        return "length " + std::to_string(actual.size()) + " instead of " + std::to_string(expected.size());
    }
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (expected[i] != actual[i]) return "byte " + std::to_string(i) + " differs";
    }
    return "identical";
}

// `value` with the member `name` replaced (or removed when `replacement` is absent and `remove` is set).
PhotonValue replaced(const PhotonValue& value, std::string_view name, PhotonValue replacement, bool remove = false) {
    PhotonValue copy = PhotonValue::structure();
    for (std::size_t i = 0; i < value.names().size(); ++i) {
        if (value.names()[i] != name) {
            copy.set(value.names()[i], value.children()[i]);
        } else if (!remove) {
            copy.set(value.names()[i], replacement);
        }
    }
    return copy;
}

PhotonFixture fixtureNamed(std::string_view uuid, std::string_view name) {
    for (PhotonFixture& fixture : loadFixtures(uuid)) {
        if (fixture.name == name) return std::move(fixture);
    }
    FAIL("no fixture named " << name);
    return {};
}

const std::vector<std::string> kExpectedCases = {"edge_values",          "empty",           "max_corners",
                                                 "max_targets",          "one_target",      "three_targets_multitag",
                                                 "transform3d_identity", "transform3d_pose"};

// ---- the real library --------------------------------------------------------------------------------------------

TEST_CASE("photon codec golden: encoding the inputs of every fixture gives the bytes the real PhotonLib serde packed") {
    for (const PhotonProfile& profile : knownPhotonProfiles()) {
        CAPTURE(std::string(profile.uuid));
        const PhotonCodec codec = codecFor(profile);
        const std::vector<PhotonFixture> fixtures = loadFixtures(profile.uuid);
        REQUIRE_MESSAGE(!fixtures.empty(), "no golden fixtures for the profile: run tools/gen_photon_fixtures.py");
        std::vector<std::string> names;
        for (const PhotonFixture& fixture : fixtures) {
            CAPTURE(fixture.name);
            names.push_back(fixture.name);
            REQUIRE_FALSE(fixture.bytes.empty());
            std::vector<std::uint8_t> bytes;
            const sdk::Status status = codec.encode(fixture.message, fixture.value, bytes);
            REQUIRE_MESSAGE(status.ok(), wpiemu::tests::photon::why(status));
            CHECK_MESSAGE(bytes == fixture.bytes, firstDifference(fixture.bytes, bytes));
        }
        for (const std::string& expected : kExpectedCases) {
            CHECK_MESSAGE(std::find(names.begin(), names.end(), expected) != names.end(),
                          "missing fixture " << expected);
        }
    }
}

TEST_CASE("photon codec golden: decoding the bytes of the real serde gives back the inputs") {
    const PhotonCodec codec = newestCodec();
    for (const PhotonFixture& fixture : loadFixtures(newestPhotonProfile().uuid)) {
        CAPTURE(fixture.name);
        const auto decoded = codec.decode(fixture.message, fixture.bytes);
        REQUIRE_MESSAGE(decoded.ok(), wpiemu::tests::photon::why(decoded));
        CHECK(decoded.value() == fixture.value);
    }
}

TEST_CASE("photon codec golden: the fixture sizes are the sizes the layout predicts") {
    const std::string uuid(newestPhotonProfile().uuid);
    // 32 bytes of metadata, a count byte, an optional flag.
    CHECK(fixtureNamed(uuid, "empty").bytes.size() == 34);
    // target: 4 doubles, 2 ints, a float, two Transform3d (56 each), a double, two list counts, 16 bytes per corner.
    constexpr std::size_t kTarget = 32 + 12 + 112 + 8 + 2;
    CHECK(fixtureNamed(uuid, "one_target").bytes.size() == 34 + kTarget + 8 * 16);
    // + multi-tag: two Transform3d, three doubles, an id count and 2 bytes per id.
    CHECK(fixtureNamed(uuid, "three_targets_multitag").bytes.size() ==
          34 + 3 * (kTarget + 8 * 16) + 112 + 24 + 1 + 3 * 2);
    CHECK(fixtureNamed(uuid, "max_targets").bytes.size() == 34 + 127 * kTarget + 112 + 24 + 1 + 127 * 2);
    CHECK(fixtureNamed(uuid, "max_corners").bytes.size() == 34 + kTarget + 254 * 16);
    CHECK(fixtureNamed(uuid, "transform3d_pose").bytes.size() == 56);
}

// ---- the encoder ---------------------------------------------------------------------------------------------

TEST_CASE("photon codec: an absent optional is one zero byte, a present one is the flag then the payload") {
    const PhotonCodec codec = newestCodec();
    const std::string uuid(newestPhotonProfile().uuid);
    const PhotonFixture empty = fixtureNamed(uuid, "empty");
    CHECK(empty.bytes.back() == 0);
    CHECK(empty.bytes[empty.bytes.size() - 2] == 0);  // no targets

    const PhotonFixture withMulti = fixtureNamed(uuid, "three_targets_multitag");
    std::vector<std::uint8_t> bytes;
    REQUIRE(
        codec.encode(kPhotonResultMessage, replaced(withMulti.value, "multitagResult", PhotonValue::absent()), bytes)
            .ok());
    const std::size_t multiBytes = 112 + 24 + 1 + 3 * 2;
    CHECK(bytes.size() == withMulti.bytes.size() - multiBytes);
    CHECK(bytes.back() == 0);
    CHECK(withMulti.bytes[withMulti.bytes.size() - multiBytes - 1] == 1);
}

TEST_CASE("photon codec: a value the schema cannot be filled from is an error that names where, never a default") {
    const PhotonCodec codec = newestCodec();
    const PhotonFixture base = fixtureNamed(newestPhotonProfile().uuid, "one_target");
    std::vector<std::uint8_t> bytes{1, 2, 3};

    SUBCASE("a missing field") {
        const PhotonValue target = base.value.find("targets")->children()[0];
        PhotonValue targets = PhotonValue::list();
        targets.push(replaced(target, "poseAmbiguity", PhotonValue::absent(), true));
        const sdk::Status status = codec.encode(kPhotonResultMessage, replaced(base.value, "targets", targets), bytes);
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().code == ErrorCode::InvalidArgument);
        CHECK(status.error().message.find("targets[0].poseAmbiguity") != std::string::npos);
        CHECK(bytes.empty());  // never a half-written packet
    }
    SUBCASE("a missing nested field") {
        const PhotonValue target = base.value.find("targets")->children()[0];
        PhotonValue pose = *target.find("bestCameraToTarget");
        PhotonValue rotation = *pose.find("rotation");
        PhotonValue quaternion = *rotation.find("q");
        PhotonValue targets = PhotonValue::list();
        targets.push(
            replaced(target, "bestCameraToTarget",
                     replaced(pose, "rotation", replaced(rotation, "q", replaced(quaternion, "w", {}, true)))));
        const sdk::Status status = codec.encode(kPhotonResultMessage, replaced(base.value, "targets", targets), bytes);
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().message.find("targets[0].bestCameraToTarget.rotation.q.w") != std::string::npos);
    }
    SUBCASE("a required field that is absent") {
        const sdk::Status status =
            codec.encode(kPhotonResultMessage, replaced(base.value, "metadata", PhotonValue::absent()), bytes);
        CHECK_FALSE(status.ok());
    }
    SUBCASE("the wrong kind of value") {
        CHECK_FALSE(
            codec.encode(kPhotonResultMessage, replaced(base.value, "targets", PhotonValue::real(1.0)), bytes).ok());
        CHECK_FALSE(
            codec.encode(kPhotonResultMessage, replaced(base.value, "metadata", PhotonValue::list()), bytes).ok());
        PhotonValue metadata = replaced(*base.value.find("metadata"), "sequenceID", PhotonValue::real(1.0));
        CHECK_FALSE(codec.encode(kPhotonResultMessage, replaced(base.value, "metadata", metadata), bytes)
                        .ok());  // a real is no int64
        metadata = replaced(*base.value.find("metadata"), "sequenceID", PhotonValue::boolean(true));
        CHECK_FALSE(codec.encode(kPhotonResultMessage, replaced(base.value, "metadata", metadata), bytes).ok());
        CHECK_FALSE(codec.encode(kPhotonResultMessage, PhotonValue::list(), bytes).ok());
    }
    SUBCASE("an integer out of range for its width") {
        const PhotonValue target = base.value.find("targets")->children()[0];
        PhotonValue targets = PhotonValue::list();
        targets.push(replaced(target, "fiducialId", PhotonValue::integer(2147483648LL)));  // int32
        CHECK_FALSE(codec.encode(kPhotonResultMessage, replaced(base.value, "targets", targets), bytes).ok());

        const PhotonFixture multi = fixtureNamed(newestPhotonProfile().uuid, "three_targets_multitag");
        PhotonValue ids = PhotonValue::list();
        ids.push(PhotonValue::integer(32768));  // int16
        PhotonValue broken = replaced(*multi.value.find("multitagResult"), "fiducialIDsUsed", ids);
        CHECK_FALSE(codec.encode(kPhotonResultMessage, replaced(multi.value, "multitagResult", broken), bytes).ok());
    }
    SUBCASE("a list above 127 items") {
        PhotonValue targets = PhotonValue::list();
        const PhotonValue target = base.value.find("targets")->children()[0];
        for (int i = 0; i < 128; ++i) targets.push(target);
        const sdk::Status status = codec.encode(kPhotonResultMessage, replaced(base.value, "targets", targets), bytes);
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().message.find("128") != std::string::npos);
        CHECK(status.error().message.find("127") != std::string::npos);
    }
    SUBCASE("an unknown message") {
        const sdk::Status status = codec.encode("NoSuchMessage", base.value, bytes);
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().code == ErrorCode::NotFound);
    }
}

TEST_CASE("photon codec: members the schema does not have are ignored, integers may fill a float field") {
    const PhotonCodec codec = newestCodec();
    const PhotonFixture base = fixtureNamed(newestPhotonProfile().uuid, "one_target");
    PhotonValue extra = base.value;
    extra.set("somethingNew", PhotonValue::integer(5));
    std::vector<std::uint8_t> bytes;
    REQUIRE(codec.encode(kPhotonResultMessage, extra, bytes).ok());
    CHECK(bytes == base.bytes);

    PhotonValue target = base.value.find("targets")->children()[0];
    PhotonValue targets = PhotonValue::list();
    targets.push(replaced(target, "area", PhotonValue::integer(1)));
    REQUIRE(codec.encode(kPhotonResultMessage, replaced(base.value, "targets", targets), bytes).ok());
    CHECK(bytes.size() == base.bytes.size());
}

// ---- the decoder ---------------------------------------------------------------------------------------------

TEST_CASE("photon codec: the decoder refuses a message that is cut anywhere, has trailing bytes or is malformed") {
    const PhotonCodec codec = newestCodec();
    const std::vector<std::uint8_t> bytes = fixtureNamed(newestPhotonProfile().uuid, "three_targets_multitag").bytes;
    REQUIRE(codec.decode(kPhotonResultMessage, bytes).ok());

    for (std::size_t length = 0; length < bytes.size(); ++length) {
        CAPTURE(length);
        const std::vector<std::uint8_t> cut(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(length));
        const auto result = codec.decode(kPhotonResultMessage, cut);
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().code == ErrorCode::ProtocolError);
    }

    std::vector<std::uint8_t> trailing = bytes;
    trailing.push_back(0);
    const auto extra = codec.decode(kPhotonResultMessage, trailing);
    REQUIRE_FALSE(extra.ok());
    CHECK(extra.error().message.find("trailing") != std::string::npos);

    std::vector<std::uint8_t> longList = bytes;
    longList[32] = 128;  // the target count
    CHECK_FALSE(codec.decode(kPhotonResultMessage, longList).ok());

    std::vector<std::uint8_t> badFlag = fixtureNamed(newestPhotonProfile().uuid, "empty").bytes;
    badFlag.back() = 2;  // an optional flag is 0 or 1
    CHECK_FALSE(codec.decode(kPhotonResultMessage, badFlag).ok());

    CHECK(codec.decode("NoSuchMessage", bytes).error().code == ErrorCode::NotFound);
}

TEST_CASE("photon codec: a Transform3d is 56 bytes, translation then the quaternion w x y z") {
    const PhotonCodec codec = newestCodec();
    std::vector<std::uint8_t> bytes;
    REQUIRE(codec.encode("Transform3d", photonTransform3d(1.0, 2.0, 3.0, 0.5, 0.25, 0.125, 0.0625), bytes).ok());
    REQUIRE(bytes.size() == 56);
    const auto decoded = codec.decode("Transform3d", bytes);
    REQUIRE(decoded.ok());
    CHECK(decoded.value().find("translation")->find("z")->realValue() == 3.0);
    CHECK(decoded.value().find("rotation")->find("q")->find("w")->realValue() == 0.5);
    CHECK(decoded.value().find("rotation")->find("q")->find("z")->realValue() == 0.0625);
    // 1.0 as little-endian IEEE double
    CHECK(std::vector<std::uint8_t>(bytes.begin(), bytes.begin() + 8) ==
          std::vector<std::uint8_t>{0, 0, 0, 0, 0, 0, 0xf0, 0x3f});
}

}  // namespace
