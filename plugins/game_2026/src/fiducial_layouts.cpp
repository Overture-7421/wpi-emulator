// GENERATED FILE - DO NOT EDIT. Regenerate with tools/gen_fiducial_layout.py, never patch by hand.
// clang-format off
//
// REBUILT (2026) AprilTag layouts for the vision simulation: where every fiducial is on the field
// ([AT] in docs/game_2026.md).
//
// Source: edu.wpi.first.apriltag:apriltag-java:2026.2.1 (apriltag-java-2026.2.1.jar),
// WPILib's AprilTagFieldLayout resources, read as shipped:
//   edu/wpi/first/apriltag/2026-rebuilt-welded.json
//     32 tags, field 16.541 x 8.069 m
//     sha256 315f71b1cfd7291a656f99fb4a8762d4fe4a31019a9784ec674119c0ea727f5a
//   edu/wpi/first/apriltag/2026-rebuilt-andymark.json
//     32 tags, field 16.518 x 8.043 m
//     sha256 9b44d503e056e8303b6cee7ec65b0cf770cbea2e475516c03b95f8fcfdee49ca
//
// Frame: WPILib's blue-origin field frame IS the SDK world frame (+X from the blue wall to the red
// wall, +Y left, origin at the blue/low-Y corner of the carpet), so translations (m) and
// quaternions are copied unchanged (JSON X, Y, Z, W -> x, y, z, w). Tag frame: origin at the tag
// centre, +X = the outward normal, +Z up. Numbers are the JSON decimal text verbatim; the compiler
// rounds each one to the nearest float. Tag size: the JSON has none; FIRST AprilTags are 6.5 in =
// 0.1651 m.
//
// Regenerate (repository root; add --jar <path to the jar> if WPILib is not in its default
// location, %PUBLIC%/wpilib/2026/maven on Windows or ~/wpilib/2026/maven elsewhere):
//   python tools/gen_fiducial_layout.py
// Check that this file is current (skipped when the jar is not installed):
//   python tools/gen_fiducial_layout.py --check

#include "wpiemu/plugins/game_2026/fiducial_layouts.h"

#include <span>
#include <vector>

#include "wpiemu/sdk/vision/fiducial_layout.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr double kTagSizeM = 0.1651;  // 6.5 in

// Rows: {id, {{x, y, z}, {qx, qy, qz, qw}}, sizeM}; a sizeM of 0 means FiducialLayout::tagSizeM.
// edu/wpi/first/apriltag/2026-rebuilt-welded.json
constexpr sdk::FiducialTag kWeldedTags[] = {
    {1, {{11.8779798f, 7.4247756f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {2, {{11.9154194f, 4.638039999999999f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {3, {{11.3118646f, 4.3902376f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {4, {{11.3118646f, 4.0346376f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {5, {{11.9154194f, 3.4312351999999997f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {6, {{11.8779798f, 0.6444996f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {7, {{11.9528844f, 0.6444996f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {8, {{12.2710194f, 3.4312351999999997f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {9, {{12.519177399999998f, 3.6790375999999996f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {10, {{12.519177399999998f, 4.0346376f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {11, {{12.2710194f, 4.638039999999999f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {12, {{11.9528844f, 7.4247756f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {13, {{16.5333172f, 7.4033126f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {14, {{16.5333172f, 6.9715126f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {15, {{16.5329616f, 4.3235626f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {16, {{16.5329616f, 3.8917626f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {17, {{4.6630844f, 0.6444996f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {18, {{4.6256194f, 3.4312351999999997f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {19, {{5.229174199999999f, 3.6790375999999996f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {20, {{5.229174199999999f, 4.0346376f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {21, {{4.6256194f, 4.638039999999999f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {22, {{4.6630844f, 7.4247756f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {23, {{4.5881798f, 7.4247756f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {24, {{4.2700194f, 4.638039999999999f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {25, {{4.0218614f, 4.3902376f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {26, {{4.0218614f, 4.0346376f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {27, {{4.2700194f, 3.4312351999999997f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {28, {{4.5881798f, 0.6444996f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {29, {{0.0077469999999999995f, 0.6659626f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {30, {{0.0077469999999999995f, 1.0977626f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {31, {{0.0080772f, 3.7457125999999996f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {32, {{0.0080772f, 4.1775126f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
};

// edu/wpi/first/apriltag/2026-rebuilt-andymark.json
constexpr sdk::FiducialTag kAndyMarkTags[] = {
    {1, {{11.863959f, 7.411491399999999f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {2, {{11.9013986f, 4.6247558f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {3, {{11.2978438f, 4.3769534f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {4, {{11.2978438f, 4.0213534f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {5, {{11.9013986f, 3.417951f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {6, {{11.863959f, 0.6312154f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {7, {{11.9388636f, 0.6312154f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {8, {{12.2569986f, 3.417951f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {9, {{12.5051566f, 3.6657534f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {10, {{12.5051566f, 4.0213534f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {11, {{12.2569986f, 4.6247558f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {12, {{11.9388636f, 7.411491399999999f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {13, {{16.499332f, 7.391907999999999f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {14, {{16.499332f, 6.960107999999999f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {15, {{16.4989764f, 4.3124882f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {16, {{16.4989764f, 3.8806881999999994f, 0.55245f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {17, {{4.6490636f, 0.6312154f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {18, {{4.6115986f, 3.417951f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {19, {{5.2151534f, 3.6657534f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {20, {{5.2151534f, 4.0213534f, 1.12395f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {21, {{4.6115986f, 4.6247558f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {22, {{4.6490636f, 7.411491399999999f, 0.889f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {23, {{4.574159f, 7.411491399999999f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {24, {{4.2559986f, 4.6247558f, 1.12395f},
         {0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f}}, 0.0},
    {25, {{4.007866f, 4.3769534f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {26, {{4.007866f, 4.0213534f, 1.12395f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {27, {{4.2559986f, 3.417951f, 1.12395f},
         {-0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f}}, 0.0},
    {28, {{4.574159f, 0.6312154f, 0.889f},
         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},
    {29, {{0.0136906f, 0.6507734f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {30, {{0.0136906f, 1.0825734f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {31, {{0.0140462f, 3.7301932f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
    {32, {{0.0140462f, 4.1619931999999995f, 0.55245f},
         {0.0f, 0.0f, 0.0f, 1.0f}}, 0.0},
};

sdk::FiducialLayout makeLayout(const char* id, const char* displayName, float fieldLengthM,
                               float fieldWidthM, std::span<const sdk::FiducialTag> tags) {
    sdk::FiducialLayout layout;
    layout.id = id;
    layout.displayName = displayName;
    layout.tagSizeM = kTagSizeM;
    layout.fieldLengthM = fieldLengthM;
    layout.fieldWidthM = fieldWidthM;
    layout.tags.assign(tags.begin(), tags.end());
    return layout;
}

}  // namespace

std::vector<sdk::FiducialLayout> buildFiducialLayouts() {
    std::vector<sdk::FiducialLayout> layouts;
    layouts.reserve(2);
    // The first layout is the plugin default (VisionConfig::layoutId empty).
    layouts.push_back(makeLayout("game_2026.welded", "REBUILT 2026 (welded)",
                                 16.541f, 8.069f, kWeldedTags));
    layouts.push_back(makeLayout("game_2026.andymark", "REBUILT 2026 (AndyMark)",
                                 16.518f, 8.043f, kAndyMarkTags));
    return layouts;
}

}  // namespace wpiemu::plugins::game_2026
