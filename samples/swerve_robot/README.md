# Swerve sample robot

An ordinary WPILib Java robot project that demonstrates a Phoenix 6 swerve drive and a
modeled intake. The code has no emulator-specific hooks. Prerequisites: WPI Emulator, the
[WPILib toolchain](../../docs/BUILDING.md), WPILib 2026's JDK, and network access to download
Gradle/vendor dependencies on the first build. Keep the Gradle wrapper and vendordeps.

Four TalonFX modules use fused CANcoders for steering, with a Pigeon 2 for field-relative
driving. In **Teleop**, joystick port 0's left stick translates at up to 0.4 m/s and its right
stick X axis turns. Teleop also moves the intake pivot to 114° with Motion Magic and runs the
roller at 8 V; leaving Teleop stops the roller.

| Device | CAN IDs |
|---|---|
| Front-left drive / steer / CANcoder | 4 / 3 / 12 |
| Front-right drive / steer / CANcoder | 8 / 7 / 11 |
| Back-left drive / steer / CANcoder | 2 / 1 / 9 |
| Back-right drive / steer / CANcoder | 6 / 5 / 10 |
| Pigeon 2 | 13 |
| Intake roller TalonFX | 32 |
| Intake pivot TalonFX / CANcoder | 29 / 30 |

The module offsets are ±0.2826 m forward and ±0.2762 m sideways. The wheel radius is
0.0508 m, drive ratio 7.03, and steer ratio 287/11. The intake pivot ratio is 40.

## Build and run

From the repository root in Windows PowerShell (adjust the JDK path):

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
Push-Location samples\swerve_robot\java
.\gradlew.bat build
Pop-Location
```

Use `--offline` only after the required artifacts are cached. Follow the
[user guide](../../docs/USER_GUIDE.md) to open the project. In **Set up your robot**, choose
**Swerve**, model your chassis and modules using the offsets and gearing above, and map the
devices by CAN ID. In Run, choose **Teleop** and click the Driver Station's **Enabled** toggle.

## End-to-end check

With a built, HAL-enabled emulator and WPILib 2026 available, run from the repository root:

```powershell
$env:WPIEMU_E2E = "1"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
ctest --preset windows-vs2022-release -R "^e2e[.]swerve_sample$" --output-on-failure
```

The check copies this project, discovers its devices, then runs it with a generated robot
model using the layout above. It checks device discovery and mappings, forward drive, steering,
and the intake pivot reaching 114°. It does not use the editor's saved model. Set
`WPIEMU_E2E_PROJECT` to test a different project with the same CAN layout. If the opt-in,
project, wrapper, extension, or JDK prerequisite is missing, CTest may report **Skipped** (77);
that is not a pass.

This check demonstrates one robot project's integration with the emulator's real HALSim
extension and Phoenix 6 simulation. It does not test physical hardware, every Phoenix device
or control mode, or native Linux/macOS robot-process loading. Windows is the documented
execution environment.

Retain FIRST/WPILib notices and helper attributions; see the shared
[WPILib license notice](../WPILib-License.md).
