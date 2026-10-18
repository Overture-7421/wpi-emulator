# Run view (`app/runview`)

The run-time page of the emulator: a telemetry bar, with Restart (and its robot-code restart option), speed and the home-view button as icon buttons at the bottom-right of the 3D view (overlay and console toggles are exposed via `viewActions()` for the shell's View menu), live
Qt Quick 3D scene, game HUD, mechanism overlays, camera and AprilTag overlays, toolbar telemetry, console and the built-in Driver Station (bottom-left, with Setup/Joysticks in a popup).

Static library `wpiemu_app_runview` (links only `wpiemu_sdk` + Qt). The shell constructs
`RunPage(IRunBackend&)` and supplies the real backend adapter; `FakeRunBackend` is a complete in-process
implementation used by the tests, the demo and screenshots.

## Layout

| Area | Files | Notes |
|---|---|---|
| Backend seam | `run_backend.h/.cpp` | `IRunBackend` (QObject): start/stop/restart/reset/pause/step/time scale, `setRobotPose` (teleport), `latestSnapshot()` (polled at ~60 Hz), DS state, `status()` (process + link state), `sceneInfo()` (config, field, HUD + overlay descriptors), `logEntry` signal |
| Fake backend | `fake_run_backend.*` | synthetic swerve robot, 3 mechanisms with moving links, a pickup and a muzzle zone that light up while the intake is deployed / the flywheel runs, 200 instanced "fuel" balls, HUD values, two cameras and 18 AprilTags (see [Cameras and AprilTags](#cameras-and-apriltags-adr-0017)), failure injection (`simulateCrash`, `simulateHandshakeTimeout`, `simulateCameraFault`, ...) |
| Presentation logic (Qt-light, unit tested) | `run_control.*`, `hud_model.*`, `sim_stats.*`, `scene_parts.*`, `camera_rig.*`, `camera_hud.*`, `console_model.*` | enable states, status chips/overlay text, HUD descriptor evaluation, SDK -> render parts, the camera / AprilTag overlay (`CameraRig`) and the camera cards |
| Qt Quick bridge | `scene_model.*`, `mechanism_overlay_model.*`, `qml/*.qml` | models for the QML scene (instancing for fuel); the camera is the editor's `OrbitCamera` (`common/orbit_camera.h`) with a field-framed home view, same orbit/pan/zoom controls as the editor viewport; SDK Z-up is converted to Qt's Y-up in exactly one place (`RunScene.qml`, node `world`) |
| Driver Station | `driver_station_model.*`, `driver_station_panel.*` | enable / disable / e-stop, mode, alliance, match sequencing from sim time, game message. `DsState::matchTimeS` = seconds **remaining** in the active segment while a timed match runs, `-1` otherwise (the game plugin then uses its integrated clock). The game message text is a manual fallback: when the loaded game plugin provides FMS data (`IGameState::gameSpecificMessage()`, e.g. 2026 hub order `R`/`B`) the runtime overlays it before the robot sees it |
| Joysticks | `joystick_types.*`, `joystick_source.*`, `joystick_manager.*`, `joystick_editor.*`, `sdl_gamepad_provider.*`, `fake_joystick.*` | 6 DS ports; keyboard or gamepad per port; binding editor with live preview and "press the input" learning; profiles in `QSettings` |
| Widgets | `run_page.*`, `run_viewport.*`, `console_panel.*`, `status_chip.*` | thin wiring only |

## Keyboard

| Key | Action |
|---|---|
| `Enter` | Disable the robot immediately (like the real DS) |
| `Space` | Latch the emergency stop (cleared from the panel or by a robot restart) |
| `Ctrl+R` | Restart: reset the physics and game state; the robot code keeps running |
| `Ctrl+Shift+R` | Restart robot code (Restart dropdown): reboot it and reset the simulation |

Restart actions are registered on the Run page, scoped to that page and its children, so
shortcuts work with the dropdown closed and do not activate from the editor. The shell's
File > Set up robot action has no conflicting shortcut.
| `Ctrl+0` | Return the camera to its home view (also the home button or a double-click on the view) |
| `Ctrl+J` | Show / hide the console |

The keyboard joystick (slot 0 by default) uses `W A S D` (left stick), arrow keys (right stick), `U`/`O`
(triggers), `Z X C V Q E R F G H N M` (buttons 1-12) and `I J K L` (POV). Everything is rebindable in the
**Joysticks** tab.

## Moving the robot

While a simulation runs (`RunControlState::simulating`; `RunPage::refreshControls` calls `SceneModel::setPlacementEnabled`) the robot can
be carried over the field with the mouse. `RunScene.qml` only forwards a plain left press / move / release and the wheel to
`SceneModel::beginRobotDrag` / `dragRobot` / `endRobotDrag` / `turnRobotDrag` (the model decides whether the pointer is on the robot,
otherwise the left button pans as before; the wheel turns the held robot, else it zooms) and shows a hand cursor from
`robotAt()`. The model emits `robotPoseRequested(x, y, z, yaw)` and `RunPage` hands it to `IRunBackend::setRobotPose`
(shell: `IRuntimeHost::setRobotPose` -> `ISimController::setRobotPose`, applied by the sim thread at the next tick boundary).

* Picking is a ray against the bounds of the rig parts in the chassis frame, grown by 8 cm. The drag maths is `placement::FloorDrag`
  (`common/robot_placement.h`, shared with the editor's start pose marker): the ray meets the plane through the grabbed point,
  the chassis keeps its pick-up height and stays upright, the wheel turns it 5 degrees a notch about the grabbed spot, `Ctrl` snaps to
  10 cm and 15 degrees.
* It is a live teleport, not a setting: nothing is written to the model. The robot stops where it is dropped (its velocities, the
  drivetrain state and the IMU's acceleration baseline are reset; mechanisms and game state are not); **Restart** returns it to the
  start pose.
* `FakeRunBackend::setRobotPose` moves its stand-in robot and counts the calls (`poseCount()`, `lastRobotPose()`).

## Cameras and AprilTags (ADR 0017)

Cameras are configured in the robot project (`RobotConfig::vision`) and computed by the emulator; the run view draws what the snapshot
carries and never talks to a camera: `ISimSnapshot::cameras()` (`CameraSample`: world pose with +X = the optical axis, field of view, the tags
of the latest capture, frame counters, latency, whether the robot process publishes it, status), `fiducials()` (the active layout's tags in the
world frame), `detectedCameras()` (`CameraRobotState`: what the robot code asked each camera for) and the `vision_*` HubWarnings. A project
without cameras shows none of it.

* **Scene** (`CameraRig` in `camera_rig.*`, next to `ZoneRig`, Qt-free): built from the snapshot alone. `update()` recomputes poses, states and
  highlights per snapshot without allocating in steady state and reports what changed (`camerasChanged()`, `tagsChanged()`, `raysChanged()`,
  `labelsChanged()`); a list that changes shape (other cameras, another layout, another field of view) is reported by `camerasRebuilt()` /
  `tagsRebuilt()` so the models reset instead of updating row by row. `SceneModel` feeds `CameraListModel` (body + frustum), two
  `PartListModel`s (tag plates, rays) and `LabelListModel`; everything is in SDK world coordinates and the Y-up conversion stays in the `world`
  node of `RunScene.qml`.
  * A camera is a small body marker and a translucent frustum: `FrustumGeometry` (four sides and the far face, `kFrustumDepthM` = 2 m, corners
    from `common/camera_frustum.h`, shared with the editor) built once per camera in the camera frame, so only the node it hangs on moves.
  * Colour by state (`cameraStateOf`, the tones of the status chips): **ok** green (enabled and published by the robot process), **waiting**
    blue (enabled, not published yet), **disabled** grey (`enabled = false` or status `disabled`), **problem** red (any other status). A camera
    that sees no tag is dim, one that sees any is brighter.
  * The layout's tags are thin plates on the surface they hang on (`FiducialSample::pose`, +X = outward normal), dim by default; a tag in the
    `visibleTagIds` of any camera is highlighted. A thin ray runs from each camera to the centre of every tag it sees (`rayParts()` is a
    camera x tag pool, unused entries have `visible = false`).
  * Tag ids and camera names are 2D items, since Qt Quick 3D has no 3D text. `SceneModel` projects each anchor with the `OrbitCamera` that drives
    the 3D view (`placeLabel`), so a label never lags behind the scene and the rule is unit tested: hidden behind the camera or off screen; a tag a camera sees is always
    labelled, the other tags only within `kTagLabelRangeM` (6 m) of the viewer and the camera names within `kCameraLabelRangeM` (12 m), so the home view is not a wall of text.
* **View menu**: "Cameras" (bodies, frustums, names and the camera cards) and "AprilTags" (plates and ids); the rays need both. Like the other
  overlay toggles (game HUD, mechanism values, zones) they are remembered in the page's `QSettings` (`runview/view/*`, on by default).
* **Camera cards** (`camera_hud.*`, `qml/CameraPanel.qml`; right edge of the view, vertically centred but never over the game HUD of the top-right corner, as the mechanism cards use
  the left edge): label, vendor, a state chip (Connected / Waiting for robot / Disabled / the status in words), the measured capture rate, the last
  latency, the tags in view and what the robot code asked for ("Robot asked for: pipeline 2 · driver mode · LED on", taken from the matching
  `CameraRobotState`: its `cameraId`, else the configured NetworkTables name and vendor; the LED values are named per vendor). The rate is the
  slope of `framesCaptured` over the **snapshot clock** over the last ~3 s (`RateMeter`), so it follows pause and time scale.
* **Problems**: `VisionNoticeModel` turns the `vision_layout_missing`, `vision_camera_unconfigured` and `vision_camera_status` HubWarnings into
  cards above the camera cards, with a readable title ("AprilTag layout not found", "Camera not configured", "Camera problem") and the
  warning's own message as detail, and writes each new one to the console (source `vision`, level from the severity; a problem that flickers is announced at most every 10 s). They show even when the
  project lists no camera, which is how a robot that uses a camera the project lacks is noticed. The run view had no list of HubWarnings before
  (only the editor lists them all), so nothing else is mapped here.
* **Fake backend**: a PhotonVision "front" camera on the chassis and a Limelight on the turret link (it sweeps with the turret), 18 tags on
  the walls and around the hubs, visibility by field of view, facing and apparent size with the hubs as occluders, counters that follow the sim
  clock, and a `detectedCameras()` entry per camera that reacts to joystick button 3 (driver mode) and 4 (pipeline 1). Failure injection:
  `simulateCameraFault(id, CameraFault::{Disabled, NotConnected, NotUsedByRobot, PhotonUuidUnsupported, ForeignPublisher})`,
  `simulateUnconfiguredCamera(ntName)` and `simulateLayoutMissing()`; `Options::withVision = false` removes the cameras. The geometry is
  `common/demo_vision.h`, shared with `DemoRuntimeHost` (`--demo`: the project's cameras, or one default front camera when it has none, and the
  game's tag layout, without occlusion).

## Conventions the view layer relies on

* The snapshot's chassis pose is the COM; chassis primitives and mechanism mounts are in that frame.
* Wheels are drawn from `WheelSample::contactPoint` (hub = contact + radius up), steered/spun from the sample.
* All primitives of a mechanism follow the mechanism's first link (the moving part published in `MechanismState::links`), and
  stay on its root when it publishes none.
* Zone overlays come from two sources, drawn by `ZoneRig` (`scene_parts.*`) into one list that the View menu's
  "Zones" toggle (and the per-zone submenu, keyed by label) controls:
  * `EditorPanelDescriptor::overlays` (other plugins' panels): `PanelScope::PluginConfig` boxes are read from
    `config.plugins[<panel id prefix before the first '.'>].data[propertyKey]` in the chassis frame,
    `PanelScope::Mechanism` boxes from the mechanism's `properties[propertyKey]` in the mechanism root frame.
    They are static translucent boxes.
  * **Robot zones** (`RobotConfig::zones`, ADR 0016): every zone whose `kind` is in `RunSceneInfo::zoneKinds`
    (the game module's `contributeZoneKinds()`; zones of an unknown kind are ignored). The pose in the chassis
    frame is the pose of the attachment link (chassis/root = identity, a mechanism link = the same link-pose logic
    as mechanism shapes, taken from `MechanismState::links`) times `ZoneConfig::local`. A `Volume` kind is a box of
    `sizeM`; a `Muzzle` kind is a small marker plus a thin arrow along local +X. Colour comes from
    `ZoneKindInfo::color`. `ISimSnapshot::zones()` (matched by zone id; missing = inactive) drives the look: an
    **active** zone is brighter and more opaque, an inactive one is dim.
  `ZoneRig::update()` recomputes poses and active flags per snapshot without allocating and reports changed
  parts, so `SceneModel` only emits `dataChanged` for zones that moved or switched state.
  `resolveRobotZones()` / `resolveZoneOverlays()` are the plain, testable equivalents.
* Game HUD: `HudWidgetDescriptor` + `ISimSnapshot::hud()`; nothing game specific lives in this directory.

## Gamepads

Gamepad input is behind `IJoystickProvider` / `IJoystickSource`; every pad is normalised to the WPILib Xbox
layout. The SDL2 implementation (`SdlGamepadProvider`) is compiled when SDL2 is found
(`find_package(SDL2 CONFIG)`) or when `-DWPIEMU_RUNVIEW_FETCH_SDL2=ON` lets CMake download and statically
build SDL 2.30 (needs network at configure time; default ON). Without it the keyboard still works and the
Joysticks tab says so. Disable with `-DWPIEMU_RUNVIEW_SDL2=OFF`.

## Demo and tests

```powershell
# configure with -DWPIEMU_BUILD_UI_DEMOS=ON (default)
wpiemu_runview_demo.exe --scenario running            # idle | starting | running | crash | timeout | mismatch | failed
wpiemu_runview_demo.exe --light --gamepad --joysticks
wpiemu_runview_demo.exe --scenario running --screenshot out.png --park   # grab and exit (window parked off-screen)
# cameras and AprilTags: frame the start area, keep the robot still, inject a failure, enlarge a detail of the screenshot
wpiemu_runview_demo.exe --scenario running --still --view robot --fault photon --screenshot out.png --crop 480,90,520,340,2 --park
#   --view robot|top   --fault photon|foreign|disabled|disconnected|unused|unconfigured|layout   --crop x,y,w,h[,scale]
```

`tests/app_runview/` builds `wpiemu_runview_tests` (doctest + `Qt6::Test`; CTest name `runview.ui_tests`,
`QT_QPA_PLATFORM=offscreen`). The offscreen platform cannot render Qt Quick 3D, so the GUI tests check that the
QML scene loads and exercise the view models; real rendering is checked with the demo on a real platform.
The vision tests (`test_camera_overlay.cpp`, `test_camera_hud.cpp`, `test_vision_backend.cpp`, `test_vision_page.cpp`) cover the rig and its change reporting, tag
highlighting, label placement and visibility, the models' signals and colours, the card texts and the rate measurement, the warning titles
and the one-time console line, the View menu toggles and their persistence, and the fake backend (cameras, visibility, counters, every fault).
