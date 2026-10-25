#pragma once

// REBUILT (2026) field and game-piece dimensions, SI units, field frame = SDK frame (Z-up NWU:
// +X from the blue alliance wall toward the red one, +Y left, origin at the blue/low-Y corner).
//
// Source tags (full citations and the simplifications live in seasons/2026/game/README.md):
//   [GM]  2026 Game Manual TU22, section 5 (ARENA) and 6 (Game Details)
//   [AT]  WPILib 2026 AprilTag layout "2026-rebuilt-welded.json" (field size, element centres)
//   [MS]  MapleSim REBUILT arena (community sim): hub centre, trench post, tower uprights, depot, hub exits
// Anything not stated by [GM] is derived and marked "approx".

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::plugins::game_2026::dim {

constexpr float inch(double inches) { return static_cast<float>(sdk::inchesToMeters(inches)); }

// ---- field ----------------------------------------------------------------------------------
inline constexpr float kFieldLengthM = 16.541f;  // [AT] field.length ([GM] ~651.2 in)
inline constexpr float kFieldWidthM = 8.069f;    // [AT] field.width  ([GM] ~317.7 in)
inline constexpr float kAllianceWallThicknessM = 0.30f;  // approx (wall volume only has to stop robots)
inline constexpr float kAllianceWallHeightM = 2.0f;      // [GM] driver station 36.8 in + 42 in sheet ~ 2.0 m
inline constexpr float kGuardrailHeightM = 0.508f;       // [GM] 5.2: 20.0 in
inline constexpr float kGuardrailThicknessM = 0.05f;     // approx
inline constexpr float kGroundMarginM = 3.0f;            // ground slab extends past the field on every side

// ---- hub ------------------------------------------------------------------------------------
inline constexpr float kHubSizeM = inch(47.0);           // [GM] 5.4: 47 in x 47 in prism
inline constexpr float kHubCenterXM = 4.6256f;           // [AT] tag faces 4.022 / 5.229, [MS] 4.625594
inline constexpr float kHubCenterYM = kFieldWidthM / 2.f;  // [AT] centred between the two bumps
inline constexpr float kHubTopZM = inch(72.0);           // [GM] 5.4: front edge of the opening is 72 in off the carpet
inline constexpr float kHubOpeningM = inch(41.7);        // [GM] 5.4: 41.7 in hexagonal opening
inline constexpr float kHubWallThicknessM = 0.05f;       // approx; makes the funnel tube hollow
inline constexpr float kHubBaseHeightM = 0.60f;          // approx: solid lower block so nothing falls out of the tube
inline constexpr float kHubSensorHalfXYM = 0.42f;        // approx: inside the opening
inline constexpr float kHubSensorHalfZM = 0.20f;         // >= 0.3 m deep (docs/reference/physics.md: sensors have no CCD)
inline constexpr float kHubSensorCenterZM = 1.55f;       // sensor top 1.75 m < rim 1.83 m
inline constexpr float kHubExitZM = 0.60f;               // [GM] 5.4 "exits at the base"; height approx
inline constexpr float kHubExitSpeedMps = 2.0f;          // [MS]
inline constexpr float kHubExitPitchRad = -0.2617994f;   // [MS] -15 deg
inline constexpr float kHubExitLateralM = 0.2984f;       // [MS] 4 exits at +-0.1492 / +-0.4477 m, pairs 0.2984 m apart
inline constexpr float kHubExitFanRad = 0.19634954f;     // [MS] +-11.25 deg (inner) / +-33.75 deg (outer)

// ---- bump -----------------------------------------------------------------------------------
inline constexpr float kBumpWidthM = inch(73.0);         // [GM] 5.5 (along Y)
inline constexpr float kBumpHeightM = inch(6.513);       // [GM] 5.5 (ridge)
inline constexpr float kBumpRampSlope = 0.26794919f;     // [GM] 5.5: 15 degree ramps, tan(15 deg)
// Derived: extend the nominal footprint along X so the ramps meet the carpet without an edge step.
inline constexpr float kBumpDepthM = 2.f * kBumpHeightM / kBumpRampSlope;

// ---- trench ---------------------------------------------------------------------------------
inline constexpr float kTrenchDepthM = inch(47.0);       // [GM] 5.6 (along X)
inline constexpr float kTrenchHeightM = inch(40.25);     // [GM] 5.6 overall height = top of the AprilTag panels
inline constexpr float kTrenchClearanceM = inch(22.25);  // [GM] 5.6 underside of the arm
inline constexpr float kTrenchArmLengthM = inch(6.0);    // [MS] project-supplied (along X)
inline constexpr float kTrenchArmHeightM = inch(4.0);    // [MS] project-supplied
inline constexpr float kTrenchArmTopM = kTrenchClearanceM + kTrenchArmHeightM;
inline constexpr float kTrenchOpeningM = inch(50.34);    // [GM] 5.6 free width under the arm (from the guardrail)
inline constexpr float kTrenchPostWidthM = inch(12.0);   // [MS] 12 in support next to the bump

// ---- tower ----------------------------------------------------------------------------------
inline constexpr float kTowerCenterYM = 3.7437f;         // [AT] centred tag 31 (y 3.746), [MS] uprights 3.315 / 4.173
inline constexpr float kTowerBaseWidthM = inch(39.0);    // [GM] 5.8
inline constexpr float kTowerBaseDepthM = inch(45.18);   // [GM] 5.8
inline constexpr float kTowerBaseHeightM = 0.006f;       // [GM] 5.8: 0.2 - 0.3 in
inline constexpr float kUprightCenterXM = 1.06204f;      // [MS]
inline constexpr float kUprightDepthM = inch(3.5);       // [GM] 5.8 (X)
inline constexpr float kUprightThicknessM = inch(1.5);   // [GM] 5.8 (Y)
inline constexpr float kUprightHeightM = inch(72.1);     // [GM] 5.8
inline constexpr float kUprightSpacingM = inch(33.75);   // [GM] 32.25 in gap + 1.5 in upright, centre to centre
inline constexpr float kRungRadiusM = inch(1.66) * 0.5f;  // [GM] 5.8: 1-1/4 in Sch 40 pipe, 1.66 in OD
inline constexpr float kRungLengthM = inch(32.25 + 2 * 1.5 + 2 * 5.875);  // [GM] 5.8
inline constexpr float kLowRungZM = inch(27.0);          // [GM] 5.8
inline constexpr float kMidRungZM = inch(45.0);          // [GM] 5.8
inline constexpr float kHighRungZM = inch(63.0);         // [GM] 5.8

// ---- depot ----------------------------------------------------------------------------------
inline constexpr float kDepotWidthM = inch(42.0);        // [GM] 5.7 (along Y)
inline constexpr float kDepotDepthM = inch(27.0);        // [GM] 5.7 (along X, from the alliance wall)
inline constexpr float kDepotBarrierWidthM = inch(3.0);  // [GM] 5.7
inline constexpr float kDepotBarrierHeightM = inch(1.125);  // [GM] 5.7 incl. hook fastener
inline constexpr float kDepotCenterYM = 5.987f;          // [MS] fuel grid 5.53 .. 6.44 (blue-wall depot), approx

// ---- outpost --------------------------------------------------------------------------------
inline constexpr float kOutpostCenterYM = 0.666f;        // [AT] tag 29 centred on the openings, [MS]
inline constexpr float kCorralOpeningWidthM = inch(32.0);  // [GM] 5.9.2
inline constexpr float kCorralOpeningBottomM = inch(1.88);  // [GM] 5.9.2
inline constexpr float kCorralOpeningHeightM = inch(7.0);   // [GM] 5.9.2
inline constexpr float kCorralWidthM = inch(35.8);       // [GM] 5.9.2
inline constexpr float kCorralDepthM = inch(37.6);       // [GM] 5.9.2 (behind the alliance wall)
inline constexpr float kChuteBottomM = inch(28.1);       // [GM] 5.9.2 chute opening bottom
inline constexpr float kChuteHeightM = inch(7.0);        // [GM] 5.9.2
inline constexpr float kChuteWidthM = inch(31.8);        // [GM] 5.9.2
inline constexpr float kChuteExitSpeedMps = 1.5f;        // approx: roll-out speed down the 15 deg chute

// ---- alliance wall drawing (the colliders above are unchanged; this only decides what is drawn) ------
// [GM] 5.9.1: a DRIVER STATION is a 36.8 in diamond plate base, a 6 in sponsor panel (top at 42.9 in) and a 42 in
// transparent sheet under a top rail; the OUTPOST and TOWER WALL are opaque; guardrails are 20 in of transparent
// polycarbonate between aluminum extrusions ([GM] 5.2).
inline constexpr float kStationBaseHeightM = inch(36.8);   // [GM] 5.9.1
inline constexpr float kStationSponsorTopM = inch(42.9);   // [GM] 5.9.1: opaque up to here, see-through above
inline constexpr float kStationRailHeightM = inch(2.0);    // approx: the top rail along the sheet
inline constexpr float kStationBaseThicknessM = 0.10f;     // approx
inline constexpr float kPlasticThicknessM = 0.012f;        // approx
inline constexpr float kPlasticOpacity = 0.2f;             // approx: how much of the clear sheet the renderer shows
inline constexpr float kStationShelfWidthM = inch(69.0);   // [GM] 5.9.1
inline constexpr float kStationShelfDepthM = inch(12.25);  // [GM] 5.9.1 (behind the wall)
inline constexpr float kStationShelfThicknessM = 0.02f;    // approx; its top is the base top
inline constexpr float kDriverEyeRoomBehindShelfM = 0.35f; // approx standing room behind the shelf
inline constexpr float kTowerWallWidthM = inch(49.25);     // [GM] 5.8
inline constexpr float kTowerWallHeightM = inch(78.25);    // [GM] 5.8
inline constexpr float kOutpostWidthM = 2.f * kOutpostCenterYM;  // approx: tag 29 is centred on the outpost, which starts at the guardrail
inline constexpr int kStationsOutpostSide = 1;             // [GM] 5.8: the TOWER sits between DRIVER STATIONS 2 and 3, so one
inline constexpr int kStationsFarSide = 2;                 // station on the outpost side and two beyond it (split equally, approx)
inline constexpr float kGuardrailPlasticThicknessM = 0.01f;  // approx
inline constexpr float kGuardrailExtrusionM = inch(1.5);   // approx: the aluminum top and bottom rails

// ---- fuel -----------------------------------------------------------------------------------
inline constexpr float kFuelRadiusM = 0.075f;            // [GM] 5.10.1: 5.91 in (15.0 cm) diameter
inline constexpr float kFuelMassKg = 0.215f;             // [GM] 5.10.1: 0.448 - 0.500 lb (0.203 - 0.227 kg), mid value
inline constexpr float kFuelFriction = 0.6f;
inline constexpr float kFuelRestitution = 0.55f;         // MapleSim uses 0.8: treat as upper bound
inline constexpr float kFuelRollingResistance = 0.02f;
inline constexpr int kNeutralColumns = 12;               // [GM] 6.3.4.1 / [MS]: 12 columns x up to 34 rows = 408
inline constexpr float kNeutralSpacingXM = 0.1522f;      // [MS] 5.991 in
inline constexpr float kNeutralSpacingYM = 0.1511f;      // [MS] 5.95 in
inline constexpr int kDepotColumns = 4;                  // 24 fuel = 4 (X) x 6 (Y)
inline constexpr int kDepotRows = 6;
inline constexpr int kMaxNeutralFuel = 408;              // [GM] 6.3.4: 360 - 408 in the NEUTRAL ZONE
inline constexpr int kFuelPerDepot = 24;                 // [GM] 6.3.4 A
inline constexpr int kFuelPerChute = 24;                 // [GM] 6.3.4 B
inline constexpr int kMaxPreload = 8;                    // [GM] 6.3.4 C

}  // namespace wpiemu::plugins::game_2026::dim
