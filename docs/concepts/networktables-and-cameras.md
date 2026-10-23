# NetworkTables and simulated cameras

Read this when you want to know how robot code receives simulated Limelight or PhotonVision data.

For example, the app can simulate a camera named `front`. The camera sees field AprilTags in the
simulated world, and the extension publishes results under the NetworkTables paths expected by
the selected camera integration. Robot code reads them through the same ntcore APIs it uses with
a [coprocessor](../GLOSSARY.md).

## NetworkTables in the robot process

**[NetworkTables](../GLOSSARY.md)** is WPILib's typed, named-value messaging system. A
**[topic](../GLOSSARY.md)** has a name and value type. A
**[publisher](../GLOSSARY.md)** writes values to it; a **subscriber** or entry reads values from
it. Updates can carry timestamps, so robot code can relate a result to when it was captured.

The emulator's vision binding attaches to the ntcore library already loaded in the
[robot process](../GLOSSARY.md).
It uses ntcore's default instance rather than creating a second private instance. The C API binding
is implemented in [`ntcore_backend.cpp`](../../bridges/vision/src/ntcore_backend.cpp). See
WPILib's [default-instance API](https://github.wpilib.org/allwpilib/docs/release/cpp/group__ntcore__instance__func.html)
and [topic guide](https://docs.wpilib.org/en/stable/docs/software/networktables/tables-and-topics.html).

The robot and emulator do not exchange camera images. The app's
[vision system](../../core/runtime/src/runtime/vision_system.cpp) creates simulated detections
from the camera pose and the game's AprilTag layout. It queues each result until its configured
latency has elapsed, then sends the result to the robot-process bridge.

## What the camera model represents

The model checks tag visibility using configured range, field of view, and tag facing. It produces
tag detections and a robot pose estimate. The pose estimate is ground truth with configured
position and yaw noise; the detections have zero pose ambiguity. This is not a rendered image or
an image-processing pipeline: there is no lens distortion, image stream, occlusion test, or
perspective-based pose solve. Robot-side code can still use detections to run its own estimator.
The data structure is documented in
[`CameraFrame`](../../sdk/include/wpiemu/sdk/vision/camera_frame.h).

## Limelight and PhotonVision formats

The bridges translate the vendor-neutral camera result into each integration's expected topics and
encoding. A Limelight camera publishes under `/limelight` or `/limelight-<name>`; for example,
the blue-origin pose topic ends in `/botpose_wpiblue`. A PhotonVision camera publishes below
`/photonvision/<name>`, including its structured `rawBytes` result. The project implements these
formats in the [Limelight bridge](../../bridges/vision/src/limelight_bridge.cpp) and
[PhotonVision bridge](../../bridges/vision/src/photon_bridge.cpp).

These bridges provide selected interoperability, not full camera firmware. Limelight reports
AprilTag results and pose arrays; PhotonVision publishes the supported pipeline-result structure.
They do not supply images or simulate every setting a physical camera exposes. Version and topic
details, timestamp rules, and unsupported behavior are part of the
[vision bridge reference](../reference/bridges.md).

## Where to look in this repository

- Capture cadence, visibility, noise, and latency:
  [`vision_system.cpp`](../../core/runtime/src/runtime/vision_system.cpp).
- Vendor-neutral result: [`CameraFrame`](../../sdk/include/wpiemu/sdk/vision/camera_frame.h).
- ntcore binding and timestamps: [`ntcore_backend.cpp`](../../bridges/vision/src/ntcore_backend.cpp).
- Limelight and PhotonVision mapping: [bridges](../reference/bridges.md).
