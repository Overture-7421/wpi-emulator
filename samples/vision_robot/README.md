# Vision sample robots

Ordinary WPILib Java and C++ projects that fuse Limelight and PhotonVision measurements
into a tank-drive pose estimate. Neither project contains emulator-specific robot code.
Prerequisites: WPI Emulator, the [WPILib toolchain](../../docs/BUILDING.md), WPILib 2026's
JDK, and Gradle/vendor dependencies (download them before using offline mode). C++ desktop
simulation also needs a platform C++ compiler. Keep both projects' Gradle wrappers and vendordeps.

Both projects use TalonFX motors 1 and 2, a Pigeon 2 at CAN ID 10, and joystick port 0.
Autonomous drives forward; Teleop uses left-stick Y for speed and right-stick X for turning.
The camera names and mounts must match robot code:

| Camera | App camera name | Robot code name | Robot-frame mount |
|---|---|---|---|
| Limelight | `front` | `limelight-front` | (+0.30, 0, +0.50) m, yaw 0° |
| PhotonVision | `photon-rear` | `photon-rear` | (-0.30, 0, +0.50) m, yaw 180° |

The transform uses the WPILib robot frame (+X forward, +Y left, +Z up), not the raw
attachment-relative pose. See [camera setup](../../docs/USER_GUIDE.md#configure-the-game-and-cameras).

## Build and run

From the repository root in Windows PowerShell (adjust the JDK path), build Java:

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
Push-Location samples\vision_robot\java
.\gradlew.bat build
Pop-Location
```

For C++, use `cpp` in place of `java`. To explicitly build a Windows C++ desktop executable,
run this from `samples\vision_robot\cpp`:

```powershell
.\gradlew.bat frcUserProgramWindowsx86-64ReleaseExecutable
```

Without a discovered desktop C++ toolchain, `build` may assemble only RoboRIO binaries.
Add `--offline` only after required artifacts are cached.

To run either project in the emulator, follow the [user guide](../../docs/USER_GUIDE.md):
choose **Tank**, set drive ratio 8.0, wheel diameter 0.1524 m, and wheel centres at
Y = ±0.30 m (0.60 m track). Map TalonFX 1/2 to the left/right powered wheels and Pigeon 2
ID 10 to Robot's IMU; the other wheels follow their side. Add the two cameras above and match
the **Robot-to-camera (WPILib robot frame)** inspector values. Choose **Auto** and click the
Driver Station's **Enabled** toggle to drive forward, or choose **Teleop** to control it.

Every 0.5 seconds of robot time, Console prints records like these:

```text
VISION limelight tv=<0|1> tags=<n> botposeX=<m> botposeY=<m> botposeYawDeg=<deg> latencyMs=<ms> connected=<true|false>
VISION photon hasTargets=<true|false> tags=<n> poseX=<m> poseY=<m> poseYawDeg=<deg> resultAgeMs=<ms> connected=<true|false>
VISION drivetrain leftMeters=<m> rightMeters=<m> fusedX=<m> fusedY=<m> fusedYawDeg=<deg>
```

The robot also publishes its fused pose at `/SmartDashboard/Vision/estimatedPose`. Results
depend on the camera names, mount transforms, optics, latency, and visible AprilTags.

## End-to-end check

With a real HAL/ntcore-enabled emulator build and WPILib 2026, run from the repository root.
This example selects the Java project:

```powershell
$env:WPIEMU_E2E = "1"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:WPIEMU_E2E_VISION_PROJECT = (Resolve-Path samples\vision_robot\java).Path
$env:WPIEMU_E2E_VISION_LANG = "java"
ctest --preset windows-vs2022-release -R "^e2e[.]vision_sample$" --output-on-failure
```

For C++, use `cpp` for the project and set the language to `cpp`. The test copies the selected
project, creates its own robot and camera model, then compares the robot's output with simulated
camera state and chassis truth. It checks forward driving, wheel-distance feedback, fused pose,
camera connectivity, tag visibility, latency, and camera pose. It does not use saved editor
configuration.

If Gradle cannot discover the installed desktop compiler, point the test at an existing Gradle
init script that configures it:

```powershell
$env:WPIEMU_E2E_VISION_GRADLE_INIT = (Resolve-Path build\desktop-toolchain.init.gradle).Path
```

The script is passed to the copied project's Gradle invocation only; it does not affect normal
app launches or the original project. Leave the variable unset for normal discovery, or clear
it afterward with `Remove-Item Env:\WPIEMU_E2E_VISION_GRADLE_INIT`. Missing opt-in, project,
wrapper, extension, or JDK prerequisites can return **Skipped** (77), not a pass.

The check demonstrates geometric camera results and robot-code integration for these sample
projects. It does not use real cameras or images, test image processing, or establish native
Linux/macOS execution. Windows is the documented execution environment. See
[bridge limits](../../docs/reference/bridges.md).

Retain FIRST/WPILib notices and helper attributions; see the shared
[WPILib license notice](../WPILib-License.md).
