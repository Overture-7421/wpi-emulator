// Source: edu/wpi/first/apriltag/2026-rebuilt-welded.json inside apriltag-java-2026.2.1.jar (WPILib 2026.2.1,
// package edu.wpi.first.apriltag; also AprilTagFields.k2026RebuiltWelded). Rows are the JSON entries in ID order:
// translation x/y/z in metres and rotation.quaternion converted from the JSON's (W,X,Y,Z) to the SDK's (x,y,z,w).
// The JSON frame (blue-origin, +X toward the red wall, +Y left, Z up) is the SDK field frame: no conversion.
// Quaternion components of exactly +-sqrt(2)/2 are written as kRootHalf.
#include "wpiemu/plugins/game_2026/april_tags.h"

#include <array>

#include "wpiemu/plugins/game_2026/field_dimensions.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr float kRootHalf = 0.70710678f;

struct TagRow {
    int id;
    float x, y, z;       // metres
    float qx, qy, qz, qw;
};

constexpr std::array<TagRow, 32> kTags = {{
    // clang-format off
    { 1, 11.8779798f, 7.4247756f, 0.889f, 0.f, 0.f, 1.f, 0.f},
    { 2, 11.9154194f, 4.63804f, 1.12395f, 0.f, 0.f, kRootHalf, kRootHalf},
    { 3, 11.3118646f, 4.3902376f, 1.12395f, 0.f, 0.f, 1.f, 0.f},
    { 4, 11.3118646f, 4.0346376f, 1.12395f, 0.f, 0.f, 1.f, 0.f},
    { 5, 11.9154194f, 3.4312352f, 1.12395f, 0.f, 0.f, kRootHalf, -kRootHalf},
    { 6, 11.8779798f, 0.6444996f, 0.889f, 0.f, 0.f, 1.f, 0.f},
    { 7, 11.9528844f, 0.6444996f, 0.889f, 0.f, 0.f, 0.f, 1.f},
    { 8, 12.2710194f, 3.4312352f, 1.12395f, 0.f, 0.f, kRootHalf, -kRootHalf},
    { 9, 12.5191774f, 3.6790376f, 1.12395f, 0.f, 0.f, 0.f, 1.f},
    {10, 12.5191774f, 4.0346376f, 1.12395f, 0.f, 0.f, 0.f, 1.f},
    {11, 12.2710194f, 4.63804f, 1.12395f, 0.f, 0.f, kRootHalf, kRootHalf},
    {12, 11.9528844f, 7.4247756f, 0.889f, 0.f, 0.f, 0.f, 1.f},
    {13, 16.5333172f, 7.4033126f, 0.55245f, 0.f, 0.f, 1.f, 0.f},
    {14, 16.5333172f, 6.9715126f, 0.55245f, 0.f, 0.f, 1.f, 0.f},
    {15, 16.5329616f, 4.3235626f, 0.55245f, 0.f, 0.f, 1.f, 0.f},
    {16, 16.5329616f, 3.8917626f, 0.55245f, 0.f, 0.f, 1.f, 0.f},
    {17, 4.6630844f, 0.6444996f, 0.889f, 0.f, 0.f, 0.f, 1.f},
    {18, 4.6256194f, 3.4312352f, 1.12395f, 0.f, 0.f, kRootHalf, -kRootHalf},
    {19, 5.2291742f, 3.6790376f, 1.12395f, 0.f, 0.f, 0.f, 1.f},
    {20, 5.2291742f, 4.0346376f, 1.12395f, 0.f, 0.f, 0.f, 1.f},
    {21, 4.6256194f, 4.63804f, 1.12395f, 0.f, 0.f, kRootHalf, kRootHalf},
    {22, 4.6630844f, 7.4247756f, 0.889f, 0.f, 0.f, 0.f, 1.f},
    {23, 4.5881798f, 7.4247756f, 0.889f, 0.f, 0.f, 1.f, 0.f},
    {24, 4.2700194f, 4.63804f, 1.12395f, 0.f, 0.f, kRootHalf, kRootHalf},
    {25, 4.0218614f, 4.3902376f, 1.12395f, 0.f, 0.f, 1.f, 0.f},
    {26, 4.0218614f, 4.0346376f, 1.12395f, 0.f, 0.f, 1.f, 0.f},
    {27, 4.2700194f, 3.4312352f, 1.12395f, 0.f, 0.f, kRootHalf, -kRootHalf},
    {28, 4.5881798f, 0.6444996f, 0.889f, 0.f, 0.f, 1.f, 0.f},
    {29, 0.007747f, 0.6659626f, 0.55245f, 0.f, 0.f, 0.f, 1.f},
    {30, 0.007747f, 1.0977626f, 0.55245f, 0.f, 0.f, 0.f, 1.f},
    {31, 0.0080772f, 3.7457126f, 0.55245f, 0.f, 0.f, 0.f, 1.f},
    {32, 0.0080772f, 4.1775126f, 0.55245f, 0.f, 0.f, 0.f, 1.f},
    // clang-format on
}};

}  // namespace

sdk::AprilTagLayout game2026AprilTagLayout() {
    sdk::AprilTagLayout layout;
    layout.name = "2026-rebuilt-welded";
    layout.fieldLengthM = dim::kFieldLengthM;
    layout.fieldWidthM = dim::kFieldWidthM;
    layout.tags.reserve(kTags.size());
    for (const TagRow& r : kTags) {
        sdk::AprilTag tag;
        tag.id = r.id;
        tag.pose = sdk::Pose3({r.x, r.y, r.z}, {r.qx, r.qy, r.qz, r.qw});
        tag.sizeM = sdk::kFrcAprilTagSizeM;
        layout.tags.push_back(tag);
    }
    return layout;
}

}  // namespace wpiemu::plugins::game_2026
