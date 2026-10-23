# WPILib 2027 preview robots

Minimal, vendor-free Java and C++ TimedRobot projects for the pinned 2027 alpha.
Each creates PWM motor 0 and DIO input 1. Teleop commands 25% with a high input,
50% with a low input, and disabled stops the motor. There is no emulator-specific
code in either robot.

Java needs JDK 25. C++ additionally needs the 2027 WPILib desktop compiler
(Visual Studio 2026 on Windows, GCC 14 on Linux).

Open `java` or `cpp` in the emulator. A multi-season package selects its 2027
extension automatically; a 2026-only build cannot run these projects.
Standalone Java simulation uses `gradlew run`; native simulation uses
`gradlew simulateNative`.

The opt-in `e2e.season_2027` checks both projects through the normal launcher.
See [building and testing](../../docs/BUILDING.md).
