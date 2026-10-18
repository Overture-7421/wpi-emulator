# Manual end-to-end tests for halsim_wpiemu

Not run by ctest (they need a WPILib installation, Gradle and a robot project).

* `fake_emulator.cpp` → target `wpiemu_halsim_fake_emulator`: a headless emulator that listens on the real local
  transport, completes the handshake, waits for `program_started`, enables the robot and single-steps lock-step ticks,
  printing every announced device and every changed output.
* `wpiemu-init.gradle`: the Gradle init script of ADR 0001 (makes `simulateJava`/`simulateNative` load the extension).
* `run_robot_project.ps1`: copies a robot project to `%TEMP%`, runs the two together and cleans up. Exit 0 /
  "E2E OK" when all ticks were acknowledged by the robot.

See `docs/halsim-extension.md` ("Tests") for the command line.

## Vision scenario (`--vision`)

`wpiemu_halsim_fake_emulator --vision` (or `run_robot_project.ps1 -Vision`) adds two synthetic cameras, the ones of
`samples/vision_robot` (see its README, "Constants"), so the vision bridge can be exercised end to end without the
emulator's vision engine:

| Camera | Vendor | Mount (robot to camera) | Image | Rate | Sees |
|---|---|---|---|---|---|
| `photon-front` | PhotonVision | (0.30, 0, 0.50) m, pitch -20 deg (looks 20 deg up) | 1280x800, 75 deg hfov | 30 Hz | 2026 welded tags 25, 26 (hub faces) |
| `limelight-rear` | Limelight | (-0.30, 0, 0.45) m, pitch -25 deg, yaw 180 deg (looks backwards, 25 deg up) | 1280x800, 90 deg hfov | 90 Hz | tags 31, 32 (blue alliance wall) |

The robot stands still at (2.0, 4.0) facing +X. Both cameras use a 25 ms latency (5 ticks of 5 ms).

* The camera setup rides the `HelloAck` (`HelloAck::vision`); the tool answers the `Hello` itself because
  `ipc::performServerHandshake` cannot attach it.
* Every tick a camera is due (accumulator rule of the vision spec, at most one capture per tick), the frame that was
  captured `latency / step + 1` ticks earlier rides `SensorFrame::vision` with the camera-to-tag transforms, pixel
  corners, yaw/pitch/area and the multi-tag solution of its two tags. The geometry is plain pinhole math
  (`vision_scenario.h`, unit-tested by `test_vision_scenario.cpp`); there is no noise, no alternate solution and no
  occlusion.
* What comes back is printed, one line per camera state: `CAMERA tick=N ...` for an `ActuatorFrame` (the first ack lists
  every camera, later acks only what changed) and `CAMERA_REPORT ...` for a `VisionReport` (sent outside the ticks: once
  right after program start, later only when a camera shows up or changes), followed by a summary at the end:

  ```
  VISION_SETUP camera='photon-front' vendor=photonvision size=1280x800 rate=30Hz latency=25000us
  VISION_SETUP camera='limelight-rear' vendor=limelight size=1280x800 rate=90Hz latency=25000us
  CAMERA_REPORT name='photon-front' vendor=photonvision id='photon-front' present=1 publishing=1 status='' frames=0 pipeline=0
  CAMERA_REPORT name='limelight-rear' vendor=limelight id='limelight-rear' present=1 publishing=1 status='' frames=0 pipeline=0
  CAMERA tick=1 name='photon-front' vendor=photonvision id='photon-front' present=1 publishing=1 status='' frames=0 pipeline=0
  CAMERA tick=1 name='limelight-rear' vendor=limelight id='limelight-rear' present=1 publishing=1 status='' frames=0 pipeline=0
  ...
  VISION_SUMMARY cameras=2 framesSent=596 cameraStates=34 reports=1
  ```
  (a 5 s run: 149 PhotonVision and 447 Limelight frames; the captures of the last 30 ms are still in the latency queue).

Without the vision bridge in the extension nothing comes back (`cameraStates=0 reports=0`): the hooks are no-ops and the
run still has to finish with all ticks acknowledged. With a robot that uses the cameras (`samples/vision_robot`) expect
the `CAMERA` lines of both cameras (`status=''` when the camera is fine) and the robot's own `VISION|...` telemetry on
its stdout (`ph.connected=1`, `ph.targets=2`, `ll.tv=1`, ...).

```powershell
cmake --build build\windows-ninja --config Release --target halsim_wpiemu wpiemu_halsim_fake_emulator
.\halsim_ext\tests\manual\run_robot_project.ps1 -Project samples\vision_robot\java -BuildDir build\windows-ninja -Config Release -Vision -Seconds 10
```
