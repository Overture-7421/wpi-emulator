#include <string>
#include <string_view>

#include <doctest/doctest.h>

#include "wpiemu/bridges/vision/photon_encoding.h"

using namespace wpiemu::bridges::vision;

namespace {

// Golden bytes produced by encodePhotonPipelineResult and decoded AND re-encoded byte-for-byte by the real
// PhotonLib PhotonPipelineResult.photonStruct (photonlib-java v2026.3.4).
constexpr std::string_view kFullGolden =
    "393000000000000000b8880000000000405489000000000000000000000000000200000000000016400000000000000a"
    "c00000000000000440000000000000000007000000ffffffff000080bf00000000000000409a9999999999c93f9a9999"
    "999999b93f000000000000000000000000000000000000000000000000000000000000f03f00000000000000409a9999"
    "999999c93f9a9999999999b93f000000000000000000000000000000000000000000000000000000000000f03f000000"
    "000000c03f040000000000c0824000000000000079400000000000408540000000000000794000000000004085400000"
    "000000007e400000000000c082400000000000007e40040000000000c082400000000000007940000000000040854000"
    "0000000000794000000000004085400000000000007e400000000000c082400000000000007e4000000000000024c000"
    "0000000000f03f000000000000f03f000000000000000003000000ffffffff000080bf00000000000008400000000000"
    "00e0bf0000000000000000000000000000000000000000000000000000000000000000000000000000f03f0000000000"
    "000840000000000000e0bf00000000000000000000000000000000000000000000000000000000000000000000000000"
    "00f03f000000000000c03f040000000000c0824000000000000079400000000000408540000000000000794000000000"
    "004085400000000000007e400000000000c082400000000000007e40040000000000c082400000000000007940000000"
    "0000408540000000000000794000000000004085400000000000007e400000000000c082400000000000007e40010000"
    "0000000000406666666666660a40000000000000e03fcd3b7f669ea0e63f00000000000000000000000000000000cd3b"
    "7f669ea0e63f00000000000000406666666666660a40000000000000e03fcd3b7f669ea0e63f00000000000000000000"
    "000000000000cd3b7f669ea0e63f7b14ae47e17a843f7b14ae47e17a943f00000000000000000203000700";

constexpr std::string_view kEmptyGolden = "01000000000000000500000000000000060000000000000000000000000000000000";

std::string toHex(const std::vector<std::uint8_t>& bytes) {
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string out;
    for (std::uint8_t b : bytes) {
        out.push_back(kDigits[b >> 4]);
        out.push_back(kDigits[b & 0xf]);
    }
    return out;
}

PhotonTarget target(int id, double yaw, double pitch, double area, double x, double y, double z) {
    PhotonTarget t;
    t.yawDeg = yaw;
    t.pitchDeg = pitch;
    t.areaPercent = area;
    t.fiducialId = id;
    t.bestCameraToTarget = {x, y, z, 0.0, 0.0, 0.0, 1.0};
    t.altCameraToTarget = t.bestCameraToTarget;
    t.poseAmbiguity = 0.125;
    t.minAreaRectCorners = {{600, 400}, {680, 400}, {680, 480}, {600, 480}};
    t.detectedCorners = t.minAreaRectCorners;
    return t;
}

}  // namespace

TEST_CASE("vision photon encoding: the type string is the PhotonLib 2026 message definition") {
    CHECK(photonPipelineResultTypeString() == "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4");
    CHECK(photonPipelineResultUuid() == "4b2ff16a964b5e2bf04be0c1454d91c4");
}

TEST_CASE("vision photon encoding: a two target multitag result matches the PhotonLib golden bytes") {
    PhotonResult r;
    r.sequenceId = 12345;
    r.captureTimestampMicros = 8'960'000;
    r.publishTimestampMicros = 9'000'000;
    r.targets = {target(7, 5.5, -3.25, 2.5, 2.0, 0.2, 0.1), target(3, -10.0, 1.0, 1.0, 3.0, -0.5, 0.0)};
    PhotonMultiTag m;
    m.best = {2.0, 3.3, 0.5, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476};
    m.alt = m.best;
    m.bestReprojErr = 0.01;
    m.altReprojErr = 0.02;
    m.fiducialIdsUsed = {3, 7};
    r.multiTag = m;
    CHECK(toHex(encodePhotonPipelineResult(r)) == kFullGolden);
}

TEST_CASE("vision photon encoding: an empty result matches the PhotonLib golden bytes") {
    PhotonResult r;
    r.sequenceId = 1;
    r.captureTimestampMicros = 5;
    r.publishTimestampMicros = 6;
    CHECK(toHex(encodePhotonPipelineResult(r)) == kEmptyGolden);
}

TEST_CASE("vision photon encoding: lists are capped at 127 entries") {
    PhotonResult r;
    r.targets.assign(200, PhotonTarget{});
    const std::vector<std::uint8_t> bytes = encodePhotonPipelineResult(r);
    CHECK(bytes[32] == 127);
    CHECK(bytes.size() == 32 + 1 + 127 * 166 + 1);  // a target without corners is 166 bytes
}

TEST_CASE("vision photon encoding: only verified photonlib versions are accepted") {
    CHECK(isVerifiedPhotonVersion("v2026.3.4"));
    CHECK(isVerifiedPhotonVersion("2026.3.4"));
    CHECK(isVerifiedPhotonVersion(kDefaultPhotonVersion));
    CHECK(isVerifiedPhotonVersion("v2025.3.2"));
    CHECK_FALSE(isVerifiedPhotonVersion("v2024.3.1"));
    CHECK_FALSE(isVerifiedPhotonVersion("v2026.3.4-beta"));
    CHECK_FALSE(isVerifiedPhotonVersion(""));
}
