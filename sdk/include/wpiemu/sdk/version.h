#pragma once

namespace wpiemu::sdk {

// SDK API version (frozen by ADR 0005, amended additively by ADR 0006).
//  * Major: bumped ONLY for a breaking change (requires an ADR); plugins declare the major they target.
//  * Minor: bumped for every additive change (new header, new defaulted field/method).
//  1.1: lock-step robustness (protocol v2), mechanism<->physics coupling, device identity/aliases,
//       RobotConfig migration hook, structured mechanism attachment (ADR 0006).
//  1.2: IGameState::gameSpecificMessage (FMS game data forwarded to the robot), PrimitiveSpec::link
//       (explicit primitive -> mechanism link). Both defaulted: existing plugins/configs are unaffected.
inline constexpr int kSdkVersionMajor = 1;
//  1.3: SimCommand::BeginLockstep / EndLockstep: an armed (discoveryOnly) session switches to lock-step and
//       back without relaunching the robot. Additive enum values; the wire layout is unchanged.
inline constexpr int kSdkVersionMinor = 3;

}  // namespace wpiemu::sdk
