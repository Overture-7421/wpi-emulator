# Swerve sample robot

Read this when trying the emulator with a Phoenix 6 swerve robot, or when running the swerve end-to-end check.
Prerequisites: the emulator and [WPILib toolchain](../../docs/BUILDING.md), WPILib 2026's JDK, and
Gradle/vendor dependencies (download them before using offline mode). Keep the wrapper and vendordeps.

The [Java project](java) is ordinary WPILib code with nothing emulator-specific. Four TalonFX swerve modules,
each steered through a fused CANcoder, and a Pigeon 2 drive field-relative from joystick port 0:
the left stick translates (up to 0.4 m/s) and the right stick's X axis turns. Entering Teleop swings an
intake pivot out to 114° with Motion Magic through a fused CANcoder and runs the roller at 8 V.

| Device | CAN IDs |
|---|---|
| Swerve drive / steer / CANcoder: front left, front right, back left, back right | 4/3/12, 8/7/11, 2/1/9, 6/5/10 |
| Pigeon 2 | 13 |
| Intake roller TalonFX | 32 |
| Intake pivot TalonFX / CANcoder | 29 / 30 |

Module offsets are ±0.2826 m forward and ±0.2762 m sideways, wheel radius 0.0508 m, drive ratio 7.03,
steer ratio 287/11 (about 26.09), pivot gear ratio 40. In the setup wizard choose **Swerve**,
then map the devices above. Choose **Teleop** and **Enable** in Run's Driver Station.

## Build and run

From the repository root on Windows (adjust the JDK path):

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
Push-Location samples\swerve_robot\java
.\gradlew.bat build
Pop-Location
```

Add `--offline` only after the needed artifacts are cached. Follow the [user guide](../../docs/USER_GUIDE.md)
to open that folder.

## End-to-end check

With a real HAL-enabled emulator build, run from the repository root:

```powershell
$env:WPIEMU_E2E = "1"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
ctest --preset windows-vs2022-release -R "^e2e[.]swerve_sample$" --output-on-failure
```

The [check](../../app/host/tests/e2e_test.cpp) runs a copy of this project. It detects devices, then
runs with a model it generates from the table above and a forward stick, and compares the discovered
devices, the mappings, the drive, the steering and the pivot angle with expectations. It does not use saved
editor config. Set `WPIEMU_E2E_PROJECT` to run another robot with the same CAN layout. With `WPIEMU_E2E=1`,
the `app.ui_tests` suite also runs the Qt shell against this project.
Missing opt-in/project/wrapper/extension/JDK prerequisites can return **Skipped** (77), not a pass.
The Windows wrapper build and real-HAL check have been exercised: forward driving, near-zero
steering/yaw, 16 mapped devices and the intake pivot reaching 114° all pass. This is not
Linux/macOS execution evidence.
Retain FIRST/WPILib notices and helper attributions; the shared notice is [WPILib-License.md](../WPILib-License.md).

Next: [User guide](../../docs/USER_GUIDE.md) for the wizard and the Driver Station.
