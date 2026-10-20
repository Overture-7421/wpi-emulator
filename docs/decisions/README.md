# Architecture decision records

Numbers are unique and sequential; never reuse or renumber an ADR (update references if one must move).

| ADR | Title |
|---|---|
| [0001](0001-halsim-extension-injection.md) | HALSim extension injection |
| [0002](0002-phoenix6-interception.md) | Phoenix 6 interception via HAL SimDevices |
| [0003](0003-box3d-chassis-physics.md) | Box3D chassis physics |
| [0004](0004-modular-boundaries.md) | Modular boundaries |
| [0005](0005-sdk-freeze.md) | SDK 1.0 freeze (evolution rules superseded in part by 0013) |
| [0006](0006-sdk-1-1-amendments.md) | SDK 1.1 amendments (superseded in part by 0013) |
| [0007](0007-ipc-transport.md) | IPC transport: named pipe / unix domain socket, hand-rolled binary codec |
| [0008](0008-physics-implementation.md) | Physics implementation (`core/physics`) |
| [0009](0009-mechanisms-impl.md) | Generic mechanisms, plants and motor models (`core/mechanisms`) |
| [0010](0010-runtime.md) | Runtime, configuration and launcher (`core/runtime`) |
| [0011](0011-single-robot-instance.md) | Single long-lived robot instance (capability and reset-history amendment) |
| [0012](0012-motion-links.md) | One-way motion links between wheels and mechanism axes |
| [0013](0013-pre-release-single-sdk-version.md) | Pre-release: the SDK stays at version 1, no compatibility is kept |
| [0014](0014-multi-motor-gearboxes.md) | Several controllers per gearbox, with a per-motor Reversed flag |
| [0015](0015-single-mechanism-template.md) | One core mechanism template (Motion: rotary, arm, linear); no catalog panel |
| [0016](0016-zones.md) | Zones replace game wrapper mechanisms: generic `RobotConfig::zones`, plugin-owned kinds, activation conditions |
| [0017](0017-vision-cameras.md) | Vision cameras: simulated Limelight / PhotonVision coprocessors published through ntcore inside the robot process; `RobotConfig::cameras`, `IVisionBridge`, `VisionSetup` |
| [0018](0018-real-robot-feedback.md) | Calibrated robot feedback and explicit start placement |
| [0019](0019-game-fed-sensors.md) | Core sensors (distance, beam break) fed by plugin-published game values |
