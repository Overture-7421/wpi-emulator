# Vision sample robots

Read this when trying simulated Limelight and PhotonVision cameras with ordinary robot code.
Prerequisites: the emulator and [WPILib toolchain](../../docs/BUILDING.md), WPILib 2026's JDK,
and Gradle/vendor dependencies (download them before using offline mode). C++ desktop simulation
also needs the platform's C++ compiler. Keep both projects' wrappers and vendordeps.

[Java](java) and [C++](cpp) projects fuse camera measurements into drivetrain pose estimates.
They use the same mounts and console format; neither contains emulator-specific robot code.

## Build and run

From the repository root on Windows (adjust the JDK path):

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
Push-Location samples\vision_robot\java
.\gradlew.bat build
Pop-Location
```

Use `cpp` instead of `java` for C++. Add `--offline` only after the needed artifacts are cached.
For an explicit Windows C++ desktop build, run
`.\gradlew.bat frcUserProgramWindowsx86-64ReleaseExecutable` from `cpp`; `build` alone can
assemble only RoboRIO binaries when no desktop toolchain is discovered.
Follow the [user guide](../../docs/USER_GUIDE.md) to open that folder and choose **Tank**.
Set drive ratio 8.0, wheel diameter 0.1524 m and wheel centres at Y = ±0.30 m (0.60 m track).
Map TalonFX 1/2 to left/right powered wheels and Pigeon 2 ID 10 to Robot's IMU; other wheels follow their side.

Add a Limelight named `front` (robot table `limelight-front`) at robot-frame
(+0.30, 0, 0.50) m, yaw 0°, and PhotonVision named `photon-rear` at
(-0.30, 0, 0.50) m, yaw 180°. Match the inspector's **Robot-to-camera (WPILib robot frame)**
line, not raw attachment-relative pose fields: a chassis mount is relative to its centre of mass.
Choose **Auto** and **Enable** in Run's Driver Station; Teleop uses joystick port 0.

## Expected output and limits

Console prints these `VISION` records every 0.5 s of robot time:

```text
VISION limelight tv=<0|1> tags=<n> botposeX=<m> botposeY=<m> botposeYawDeg=<deg> latencyMs=<ms> connected=<true|false>
VISION photon hasTargets=<true|false> tags=<n> poseX=<m> poseY=<m> poseYawDeg=<deg> resultAgeMs=<ms> connected=<true|false>
VISION drivetrain leftMeters=<m> rightMeters=<m> fusedX=<m> fusedY=<m> fusedYawDeg=<deg>
```

After startup, expect connected cameras and target flags when tags are visible.
The fused pose is also published at `/SmartDashboard/Vision/estimatedPose`.
Autonomous commands forward motion. The Windows Java and C++ wrapper builds and real-HAL/ntcore
checks have been exercised, including forward motion, both wheel-distance feedback values and
fused pose agreement with ground truth. Both projects retain their Phoenix 6 drivetrain and IMU;
stationary camera/fused-pose agreement alone is not drive validation.
See [bridge limits](../../docs/reference/bridges.md). Native Linux/macOS execution is unverified
for both languages.

## End-to-end check

With a real HAL/ntcore-enabled emulator build, run from the repository root:

```powershell
$env:WPIEMU_E2E = "1"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:WPIEMU_E2E_VISION_PROJECT = (Resolve-Path samples\vision_robot\java).Path
$env:WPIEMU_E2E_VISION_LANG = "java"
ctest --preset windows-vs2022-release -R "^e2e[.]vision_sample$" --output-on-failure
```

For C++, change the project to `cpp` and language to `cpp`. The check builds a project copy,
creates its own model and compares robot output with camera/pose truth; it does not use saved editor config.
Both language selections require forward driving, nonzero encoder distances and a matching fused
pose, in addition to camera connectivity, visible tags, latency and camera pose checks.
If Gradle cannot discover an installed desktop compiler, select that installation with a Gradle
init script (`--init-script <file>` for the wrapper). Before invoking CTest, set the optional,
e2e-only variable to that existing script's absolute path, for example:

```powershell
$env:WPIEMU_E2E_VISION_GRADLE_INIT = (Resolve-Path build\desktop-toolchain.init.gradle).Path
```

The script must contain your installed compiler's configuration; this variable only changes
the check's copied project's Gradle invocation, not normal app launches or the original project.
Leave it unset for normal discovery, or clear it with
`Remove-Item Env:\WPIEMU_E2E_VISION_GRADLE_INIT` after the check.
Missing opt-in/project/wrapper/extension/JDK prerequisites can return **Skipped** (77), not a pass.
Use Building's platform test presets elsewhere; this command does not establish native Linux/macOS results.
Retain FIRST/WPILib notices and helper attributions; the shared notice is [WPILib-License.md](../WPILib-License.md).

Next: [User guide](../../docs/USER_GUIDE.md) for controls and camera troubleshooting.
