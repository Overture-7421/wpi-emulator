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

## Vendor and vision probes

`vendor_java` pins Phoenix 6 `26.70.0-alpha-2` from WPILib's 2027 alpha-7
vendor registry. Unlike hardware-only Systemcore libraries, this manifest includes
desktop software-simulation libraries.

`vision_java` and `vision_cpp` pin PhotonLib
`dev-v2027.0.0-alpha-2-86-gda55d785` rather than the moving `Dev` release.
They use ordinary PhotonCamera APIs and Limelight NetworkTables entries, with version
checks enabled. The Java client exercises targets, transforms, latency, driver mode,
enabled state and pipeline requests. The host test supplies a synthetic two-tag layout;
the installed app still has no production 2027 field plugin.

**The pinned C++ PhotonLib currently throws an upstream UUID-check error.**
The C++ project builds but is a failure reproducer, not a supported vision sample.
See the [vision compatibility contract](../../docs/reference/bridges.md#photonvision).
The built `vision_cpp` executable also accepts `--uuid-probe` without starting a
robot or loading HALSim. It publishes the UUID using ntcore's C API, reads the typed
JSON value using its C++ API, and compares implicit serialization with `get_string()`.
For this pin it reports `implicitMatches=0 decodedMatches=1`: the property is correct,
but PhotonLib's implicit conversion adds JSON quotes. The necessary upstream fix is
to extract the string with `get_string()`, not to parse `GetProperty()` again.

With `WPIEMU_E2E=1`, run `e2e.vendor_2027` for CTRE and `e2e.vision_2027`
for Java vision. `WPIEMU_E2E_VISION_LANG=cpp` selects the upstream failure reproducer;
it must not be interpreted as passing C++ coverage. `WPIEMU_E2E_LAUNCH=direct`
selects external launch. These are desktop checks, not Systemcore hardware validation.

For an explicit test-only workaround, set
`WPIEMU_TEST_SKIP_PHOTON_VERSION_CHECK=1` together with
`WPIEMU_E2E_VISION_LANG=cpp`. The C++ sample logs a warning and calls PhotonLib's
`SetVersionCheckEnabled(false)`; PhotonLib itself is not patched, and checks remain
enabled by default. With this opt-in, all 37 vision assertions pass on Windows in
both Gradle and direct launch modes, including targets, transforms, calibration,
latency, controls, resets and cleanup. This verifies decoding with the check
bypassed, not normal checked compatibility. The emulator does not apply this
workaround to arbitrary robot projects.
