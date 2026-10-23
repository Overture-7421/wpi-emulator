# 0017: One HALSim extension per WPILib season

**Status:** Accepted

## Context

WPILib 2027 changes the HAL ABI, clock units, Driver Station and device families.
A single extension cannot link safely against both the 2026 and 2027 HALs.
The robot project, not a saved emulator setup, determines the required season.

## Decision

Build each extension in an isolated season-specific CMake tree from shared sources
with compile-time HAL adapters. Compose requested seasons into one install with
year-qualified extension directories. Keep HAL-independent simulation and IPC shared.
Profile pins, artifact groups and header probes live in `versions.json`.

Inspect the live project before launch and select only its season's extension.
Reject known handshake year mismatches. An explicit `WPIEMU_EXTENSION` is the sole
candidate and permits a mismatch with a warning; it is a diagnostic override, not an
ABI compatibility guarantee. An unknown year tries the default season with a warning.

Expose HAL capabilities rather than inventing removed hardware signals.
Convert 2027 nanoseconds at the HAL boundary; simulation retains integer microseconds.
Use observed CAN bus IDs and full CAN-FD payloads, never configured identity guesses.

## Consequences

Packaging must build and exercise every requested season; a stub is not support.
2027 remains preview while upstream is alpha. Java/Gradle/native toolchains follow the
project year independently of the app's compiler. A season without a game plugin uses
core-only simulation; extension support does not imply season-specific game rules or
universal vendor-library compatibility.
