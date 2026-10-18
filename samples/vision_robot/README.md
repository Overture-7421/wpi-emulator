# Vision sample robots

Two ordinary WPILib 2026 robot projects, one Java and one C++, that use **both** a Limelight and a PhotonVision camera
the way real teams do. They are the verification targets for the emulator's vision feature
([docs/vision.md](../../docs/vision.md)): the code knows nothing about the emulator and is not modified to run in it.

| Project | Language | Build |
|---|---|---|
| [`java/`](java) | Java 17, GradleRIO 2026.2.1 | `gradlew build` |
| [`cpp/`](cpp) | C++20, GradleRIO 2026.2.1 | `gradlew build` (needs the Visual Studio C++ toolchain for desktop simulation) |

Both builds resolve offline from the WPILib 2026 install and a warm Gradle cache
(`JAVA_HOME=C:\Users\Public\wpilib\2026\jdk`, `gradlew --offline build`).

## What the robot does

* **Drivetrain**: a tank drive. `TalonFX` 1 (left) and 2 (right), neither inverted, driven through WPILib `DifferentialDrive`
  (`set(duty)`), and a `Pigeon2` (CAN id 10) for the gyro. Phoenix 6 is used because the emulator feeds back wheel rotor
  position and Pigeon yaw for it; plain PWM controllers would give the pose estimator no wheel or heading data.
  `SensorToMechanismRatio` is 8.0, the wheel radius 0.0762 m and the track width 0.60 m (constants in `Robot`).
* **Vision** (`Vision` class, called from `robotPeriodic`):
  * Limelight `limelight-front` through the official `LimelightHelpers` (vendored unmodified): `getBotPoseEstimate_wpiBlue`
    (MegaTag1, fused when at least 2 tags are seen) and `getBotPoseEstimate_wpiBlue_MegaTag2` after
    `SetRobotOrientation` (fused with an infinite heading std dev). The camera mount is sent with `setCameraPose_RobotSpace`.
  * PhotonVision `photon-rear` through `PhotonCamera.getAllUnreadResults()` and `PhotonPoseEstimator`
    (`estimateCoprocMultiTagPose`, falling back to `estimateLowestAmbiguityPose`) on
    `AprilTagFields.k2026RebuiltWelded`.
  * A `DifferentialDrivePoseEstimator` fuses both with `addVisionMeasurement(pose, timestampSeconds)`. It is placed on the
    field by the first measurement whose heading is trustworthy (MegaTag1 with 2+ tags or a Photon multi-tag result).
* **Modes**: `autonomousPeriodic` drives straight forward at 30 % duty for the whole autonomous period (deterministic
  motion, so tags come in and out of view); `teleopPeriodic` is arcade drive from controller 0.

Camera mounts in the WPILib robot frame (+X forward, +Y left, +Z up, origin on the floor below the robot's centre):

| Camera | Name | Translation | Yaw |
|---|---|---|---|
| Limelight | `limelight-front` | x = +0.30 m, y = 0, z = 0.50 m | 0 (forward) |
| PhotonVision | `photon-rear` | x = -0.30 m, y = 0, z = 0.50 m | 180 deg (backward) |

They are the `kRobotToLimelight` / `kRobotToPhoton` constants in `Vision`.

## The `VISION` line contract

Every 0.5 s of robot time (`Timer.getFPGATimestamp()`) each project prints exactly two lines to stdout (flushed):

```
VISION limelight tv=<0|1> tags=<n> botposeX=<m> botposeY=<m> botposeYawDeg=<deg> latencyMs=<tl+cl> connected=<true|false>
VISION photon hasTargets=<true|false> tags=<n> poseX=<m> poseY=<m> poseYawDeg=<deg> resultAgeMs=<ms> connected=<true|false>
```

* Limelight: `tags` / `botpose*` come from the MegaTag1 `botpose_wpiblue` estimate (0 when nothing is published),
  `latencyMs` is `tl + cl`, and `connected` is true when the `hb` heartbeat entry of the `limelight-front` table changed in
  the last second (`LimelightHelpers` has no connected check).
* Photon: values come from the newest result seen and the pose from `PhotonPoseEstimator` on it (0 without an estimate);
  `resultAgeMs = (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000`; `connected = camera.isConnected()`.
  Before the first result the age is meaningless (large).
* Numbers use a `.` decimal point (3 decimals for metres, 2 for degrees, 1 for milliseconds).

The same values are published to NetworkTables under `/SmartDashboard/Vision/limelight/*` and `/SmartDashboard/Vision/photon/*`
(`tv`, `tags`, `botposeX`, ... / `hasTargets`, `tags`, `poseX`, ...), the fused estimate under
`/SmartDashboard/Vision/fused/{x,y,yawDeg}`, and the fused `Pose2d` as a struct at `/SmartDashboard/Vision/estimatedPose`.

## Running them with the emulator

Follow [docs/USER_GUIDE.md](../../docs/USER_GUIDE.md): open the project in the emulator, finish the wizard as a **tank**
drive, then in the editor:

1. Map the detected devices: `TalonFX 1` -> left drive, `TalonFX 2` -> right drive, `Pigeon 2` 10 -> the robot IMU. Use a drive
   gear ratio of 8.0, wheel diameter 0.1524 m and a track width of 0.60 m so the robot's odometry agrees with the simulated wheels.
2. Add two cameras (see the *Cameras* walkthrough in the user guide). Their mounts must match the constants above:

   | Camera | kind | name | attachment | position | yaw |
   |---|---|---|---|---|---|
   | front | Limelight | `front` (table `limelight-front`) | chassis | x = +0.30, z = 0.50 | 0 |
   | rear | PhotonVision | `photon-rear` | chassis | x = -0.30, z = 0.50 | 180 deg |

   **Frames**: the robot code's robot frame has its origin on the floor below the hull centre. The emulator's chassis frame
   (where a camera on the chassis is placed in `robot.json`) has its origin at the **centre of mass**. Enter the camera
   position relative to the centre of mass: `local = robotToCamera - (COM position in the robot frame)`. For a COM at the
   hull centre with `comHeightM = 0.2` that gives z = 0.30 for both cameras (x unchanged).
3. Run autonomous and watch the console: the `VISION` lines show the tags appearing and disappearing as the robot
   drives, and the fused estimate (`Vision/estimatedPose`) should follow the simulated pose.
