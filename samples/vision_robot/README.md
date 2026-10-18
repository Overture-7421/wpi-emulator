# Vision sample robots

Two small WPILib 2026.2.1 robot projects, `java/` and `cpp/`, that use PhotonVision and Limelight
exactly as a team would. The WPI Emulator's vision tests run against them.

* Generated from the official WPILib 2026.2.1 "Timed Robot" templates (team 9999, project year 2026).
* **No simulation code.** The emulator computes the camera data and publishes the PhotonVision and
  Limelight NetworkTables entries inside the robot process; the robot cannot tell the difference.
  `lint_no_sim_code.py` fails if the robot sources reference `PhotonCameraSim`, `VisionSystemSim`,
  `SimCameraProperties`, `simulationPeriodic`, `simulationInit`, `SimDeviceSim`, `SimPhotonVision`,
  `SimulatedArena` or `maplesim`.
* Java and C++ behave identically.

## Constants

The end-to-end tests depend on these values. Transforms are WPILib `Transform3d`, robot to camera
(+X forward, +Y left, +Z up).

| | PhotonVision | Limelight |
|---|---|---|
| Name | `photon-front` | `limelight-rear` |
| Translation (m) | 0.30, 0.00, 0.50 | -0.30, 0.00, 0.45 |
| Rotation | roll 0, pitch -20 deg, yaw 0 | roll 0, pitch -25 deg, yaw 180 deg |

* The Limelight pose is sent with `setCameraPose_RobotSpace(name, x, -y, z, -rollDeg, -pitchDeg,
  -yawDeg)` because Limelight robot space is +Y right: forward -0.30, side 0, up 0.45, roll 0,
  pitch 25, yaw -180.
* Yaw source: Phoenix 6 `Pigeon2`, CAN id 0, rio bus.
* Field layout: 2026 REBUILT welded (`AprilTagFields.k2026RebuiltWelded`,
  `frc::AprilTagField::k2026RebuiltWelded`).
* Vendordeps: PhotonLib v2026.3.4 (the version whose wire format is verified) and Phoenix 6.
  `LimelightHelpers` is vendored as source (MIT, see `LimelightHelpers-LICENSE.txt`).

## Behavior

All vision work is in `robotPeriodic()`, so it runs in every mode.

* PhotonVision: `PhotonCamera` and `PhotonPoseEstimator`. Every loop drains `getAllUnreadResults()`
  and estimates the pose with the coprocessor multi-tag solve, falling back to the lowest-ambiguity
  estimate.
* Limelight: at startup `setCameraPose_RobotSpace` and `SetIMUMode(name, 1)`. Every loop
  `SetRobotOrientation` with the Pigeon yaw plus the joystick offset, then `getTV`,
  `getFiducialID`, `getTX`, `getTY`, `getTA`, `getBotPoseEstimate_wpiBlue` (MegaTag1),
  `getBotPoseEstimate_wpiBlue_MegaTag2` and `getRawFiducials`.
* Joystick 0 (driven through the Driver Station):

  | Input | Held | Released |
  |---|---|---|
  | Button 1 | PhotonVision driver mode on | off |
  | Button 2 | PhotonVision pipeline 1 | pipeline 0 |
  | Button 3 | Limelight LEDs forced on | forced off |
  | Button 4 | Limelight pipeline 1 | pipeline 0 |
  | Axis 0 | extra yaw of axis x 90 deg added to the yaw sent to the Limelight | |

## Telemetry

Every 5th loop (loop 5, 10, ...) one line goes to stdout:

```
VISION|loop=5|fpga=0.102|yaw=0.00|ph.connected=0|ph.results=0|ph.targets=-|ph.ids=-|ph.ts=-|ph.strategy=-|ph.pose=-|ll.tv=0|ll.tid=0|ll.tx=0.00|ll.ty=0.00|ll.ta=0.000|ll.fids=0|ll.mt1=-|ll.mt2=-
```

| Key | Value |
|---|---|
| `loop`, `fpga`, `yaw` | loop count, `Timer.getFPGATimestamp()` (s), Pigeon yaw without the joystick offset (deg) |
| `ph.connected` | `PhotonCamera.isConnected()` as 0/1 |
| `ph.results` | results drained since the previous line |
| `ph.targets`, `ph.ids`, `ph.ts` | target count, fiducial ids (comma separated) and timestamp (s) of the newest result seen |
| `ph.strategy`, `ph.pose` | `multi` or `ambiguity` and the robot pose `x,y,yawDeg` estimated from that result |
| `ll.tv`, `ll.tid`, `ll.tx`, `ll.ty`, `ll.ta` | the Limelight scalars |
| `ll.fids` | number of raw fiducials |
| `ll.mt1`, `ll.mt2` | MegaTag1 / MegaTag2 `x,y,yawDeg,tagCount,timestampSeconds` |

`-` means the value does not exist: no result yet, no estimate, or a Limelight estimate with
`tagCount` 0. The same values are mirrored to NetworkTables under `/VisionSample/`: `line` (the
printed text), `loop`, `fpga`, `yaw`, `ph/{connected,results,targets,ids,ts,strategy,pose}` and
`ll/{tv,tid,tx,ty,ta,fids,mt1,mt2}`. Missing values are -1 (counts), NaN (numbers) or an empty array.

## Build and run

Use the WPILib 2026 JDK (`JAVA_HOME`). The first build needs network access for the PhotonLib and
Phoenix 6 artifacts. Use `gradlew.bat` on Windows.

```
cd java
./gradlew build              # compile
./gradlew simulateJava       # run on the simulated HAL

cd ../cpp
./gradlew build              # desktop and roboRIO executables
./gradlew simulateNative     # run the desktop executable
```

Under the emulator the launcher runs the same Gradle tasks with the HALSim extension injected;
nothing in the projects changes. Until the emulator supplies a camera, PhotonLib reports that no
PhotonVision coprocessor was found, the `ph.*` values stay at `-` and `ll.tv` is 0.

```
python lint_no_sim_code.py
```

## Differences from a freshly generated project

* `includeDesktopSupport = true` in both `build.gradle` files (the WPILib VS Code "Simulate Robot
  Code" command makes the same edit). It is required for `simulateNative`.
* The unused `WPILibNewCommands` vendordep and the IDE-only `.vscode/` folder are not included.
* The template's empty `simulationInit()` / `simulationPeriodic()` overrides are removed.
