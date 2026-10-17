# Run view (`app/runview`)

The run-time page of the emulator: a telemetry bar, with Restart (and its robot-code restart option), speed and the home-view button as icon buttons at the bottom-right of the 3D view (overlay and console toggles are exposed via `viewActions()` for the shell's View menu), live
Qt Quick 3D scene, game HUD, mechanism overlays, toolbar telemetry, console and the built-in Driver Station (bottom-left, with Setup/Joysticks in a popup).

Static library `wpiemu_app_runview` (links only `wpiemu_sdk` + Qt). The shell constructs
`RunPage(IRunBackend&)` and supplies the real backend adapter; `FakeRunBackend` is a complete in-process
implementation used by the tests, the demo and screenshots.

## Layout

| Area | Files | Notes |
|---|---|---|
| Backend seam | `run_backend.h/.cpp` | `IRunBackend` (QObject): start/stop/restart/reset/pause/step/time scale, `latestSnapshot()` (polled at ~60 Hz), DS state, `status()` (process + link state), `sceneInfo()` (config, field, HUD + overlay descriptors), `logEntry` signal |
| Fake backend | `fake_run_backend.*` | synthetic swerve robot, 3 mechanisms with moving links, 200 instanced "fuel" balls, HUD values, failure injection (`simulateCrash`, `simulateHandshakeTimeout`, ...) |
| Presentation logic (Qt-light, unit tested) | `run_control.*`, `hud_model.*`, `sim_stats.*`, `scene_parts.*`, `console_model.*` | enable states, status chips/overlay text, HUD descriptor evaluation, SDK -> render parts |
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

## Conventions the view layer relies on

* The snapshot's chassis pose is the COM; chassis primitives and mechanism mounts are in that frame.
* Wheels are drawn from `WheelSample::contactPoint` (hub = contact + radius up), steered/spun from the sample.
* A mechanism primitive follows `PrimitiveSpec::link` when set; otherwise the link whose name is a prefix of the
  primitive name (`arm` for `arm.bar`, `arm_roller`, `arm/pad`; longest match wins, otherwise `root`; the UI-side
  convention `linkForPrimitive`).
* Zone overlays come from `EditorPanelDescriptor::overlays`: `PanelScope::PluginConfig` boxes are read from
  `config.plugins[<panel id prefix before the first '.'>].data[propertyKey]` in the chassis frame,
  `PanelScope::Mechanism` boxes from the mechanism's `properties[propertyKey]` in the mechanism root frame.
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
```

`tests/app_runview/` builds `wpiemu_runview_tests` (doctest + `Qt6::Test`; CTest name `runview.ui_tests`,
`QT_QPA_PLATFORM=offscreen`). The offscreen platform cannot render Qt Quick 3D, so the GUI tests check that the
QML scene loads and exercise the view models; real rendering is checked with the demo on a real platform.
