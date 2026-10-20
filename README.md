# WPI Emulator

Cross-platform (Windows / macOS / Linux) desktop emulator that runs an **unmodified** WPILib
robot project (Java or C++) in simulation. A HALSim extension inside the robot process talks to
a Qt 6 app that owns physics (Box3D), mechanisms, and the 3D view. Season-specific behavior lives
in year plugins; v1 ships `game_2026` (REBUILT). Current version: 0.1.0.

## Features

* **Unmodified robot code**: the project is launched through Gradle with a HALSim extension injected; no code,
  vendordep or build change in the robot project.
* **Setup wizard**: project inspection (season, language, team, vendordeps), toolchain check (JDK, Gradle wrapper,
  WPILib, C++ compiler, game module), robot description (swerve / tank / mecanum) and optional first device discovery.
* **3D robot editor**: chassis, wheels and collision shapes, one generic mechanism (rotary, arm or linear: flywheel, roller,
  turret, hood, elevator...) added from the tree's context menu, attachment links, **detect devices** and drag-and-drop mapping of detected
  devices to mechanism / wheel slots, validation, undo/redo, imperial or metric display.
* **Physics**: Box3D rigid-body chassis with tire models and swerve / tank / mecanum drivetrains, battery sag and
  brownout, exact DC-motor plants (rotary, arm, linear) with current limits and WPILib motor data.
* **Device bridges**: WPILib HAL families (PWM, DIO, analog, encoders, relays, solenoids, PCM/PH, power
  distribution, LEDs, roboRIO, accelerometer, raw CAN) and CTRE Phoenix 6 (TalonFX/FXS, CANcoder, Pigeon 2, CANdle,
  CANrange, CANdi) through HAL SimDevices; Phoenix 5 rules (not verified against the real library).
* **2026 game**: field with hub, bumps, trenches, tower, depots and outposts; fuel as physics bodies; hub shift rules,
  scoring, fuel intake and shooter wrappers, HUD, and editor panels for fuel / staging / match rules.
* **Vision cameras**: simulated **Limelight** and **PhotonVision** coprocessors for AprilTag pose estimation. Cameras are added in the
  editor to the chassis or any mechanism (a turret camera moves with the turret); the emulator computes which tags each sees (field
  of view, range, facing, frame rate, latency, optional noise) and the extension publishes the usual NetworkTables topics into the
  robot's own NT instance, so `LimelightHelpers` and PhotonLib work on the unmodified project. No occlusion, distortion or object
  detection ([docs/vision.md](docs/vision.md)).
* **Run view**: live 3D scene, game HUD, a Robot state window (mechanisms, sensors, zones, cameras), camera view pyramids, console, start / stop / restart / reset / pause / step,
  and a built-in **Driver Station** (enable, e-stop, mode, alliance, timed match, joysticks via keyboard or gamepad).
* **Deterministic, lock-step simulation**: the emulator is the clock master (default 5 ms step); runs are
  reproducible from a seed.

## Documentation

| Doc | For |
|---|---|
| [docs/USER_GUIDE.md](docs/USER_GUIDE.md) | Install, wizard, editor, run view, troubleshooting, limitations |
| [docs/BUILDING.md](docs/BUILDING.md) | Build, test, options |
| [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) | Conventions, tests, adding a year plugin / device bridge / mechanism / vendor |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Modules and the dependency diagram, threading, per-tick data flow, extension points |
| [docs/decisions/](docs/decisions/README.md) | ADRs 0001-0017 |
| [docs/vision.md](docs/vision.md) | Simulated Limelight / PhotonVision cameras: model, frames, NetworkTables topics, adding a vendor |
| [docs/packaging.md](docs/packaging.md) | Installers, AppImage / dmg, release workflow |
| [docs/game_2026.md](docs/game_2026.md), [docs/new-year-plugin.md](docs/new-year-plugin.md) | The 2026 plugin; writing a new season |

## Layout
| Dir | Target | Role |
|---|---|---|
| `sdk/` | `wpiemu_sdk` | Public interfaces (header-only). Everything else depends on this. |
| `core/` | `wpiemu_core` | Game-agnostic sim core: Box3D physics, mechanisms and plants, runtime / config / launcher. No Qt. |
| `ipc/` | `wpiemu_ipc` | Wire protocol + transport (named pipe / unix socket) between app and extension. |
| `halsim_ext/` | `halsim_wpiemu` | HALSim extension loaded into the robot process. |
| `bridges/hal`, `bridges/vendor`, `bridges/vision` | `wpiemu_bridges_*` | Device bridges (HAL, CTRE Phoenix) and vision bridges (Limelight, PhotonVision over ntcore); `bridges/builtin_bridges.cpp` aggregates them. |
| `plugins/game_2026`, `plugins/_template` | `wpiemu_plugin_*` | Year plugins; link only the SDK; `plugins/builtin_plugins.cpp` aggregates them. |
| `app/` | `wpiemu_app` | Qt Widgets + Quick 3D application (wizard, shell, editor, run view, `host/` robot session). |
| `tests/` | `wpiemu_tests` | doctest unit tests + CTest checks (plus per-module test executables). |
| `docs/`, `packaging/`, `tools/` | | Docs/ADRs, installers, dev scripts (`check_boundaries.py`). |

Module dependency rules are enforced by `tools/check_boundaries.py` (see
[ADR 0004](docs/decisions/0004-modular-boundaries.md)). Architecture overview and diagram (section 1), extension
points and the "add a year plugin / bridge / mechanism" checklists: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md);
the frozen SDK and how to change it: [ADR 0005](docs/decisions/0005-sdk-freeze.md).

## Quick start

**Use it** (Windows): install the NSIS installer or unzip the portable build (see
[docs/packaging.md](docs/packaging.md)), start `wpiemu_app`, drop your WPILib 2026 project folder on the window and
follow the wizard ([user guide](docs/USER_GUIDE.md)). `wpiemu_app --demo` explores the UI without a robot.

**Build it** (Windows, Visual Studio 2022 generator; prerequisites such as Qt 6.8 and WPILib 2026 are in
[docs/BUILDING.md](docs/BUILDING.md)):
```powershell
cmake --preset windows-vs2022 -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release
```
Linux / macOS presets (`linux`, `macos`) are described in BUILDING.md.

## Status and roadmap

v0.1.0. What is verified, and what is not:

| Area | Status |
|---|---|
| Windows x64 (MSVC 2022, Qt 6.8.3, WPILib 2026.2.1) | Built warning-free with `WPIEMU_WARNINGS_AS_ERRORS=ON`; all 693 CTest entries pass (the real-robot end-to-end test is opt-in and skipped by default). NSIS installer and portable zip built and verified. |
| End-to-end on a real 2026 robot project | Has been run on Windows through Gradle with the real extension and session (opt-in `e2e.shelby_2026`, `RealRuntimeHost` flavour in the app tests). |
| Vision (Limelight / PhotonVision) | End-to-end on Windows with the Java and C++ sample robots in `samples/vision_robot` (opt-in `e2e.vision_sample`). The C++ sample was verified with the robot stationary (Phoenix 6 driving from C++ is unverified). The ntcore binding on Linux / macOS is untested. |
| **Linux and macOS runs** | **Unverified.** The code is portable and CI is configured for both, but neither has been built or run locally. The extension's interaction with the robot's own `libwpiHal` on these platforms is the open item of [ADR 0001](docs/decisions/0001-halsim-extension-injection.md). |
| **AppImage / dmg packaging scripts** | **Written, not run.** `packaging/linux/build-appimage.sh` is syntax-checked only; `packaging/macos/*` and notarization have never executed. Expect fixes on the first CI release run. |
| Phoenix 5 | Rules derived from documentation, not tested against the real vendor library. |
| Phoenix 6 CANivore bus label | Not available (Phoenix sim keys devices by type + id; the optional L3 hook is not shipped). |
| Rendering tests | Offscreen CI cannot render Qt Quick 3D; visuals are checked manually with the demo executables. |

**Not in v1 (roadmap candidates)**: REV Robotics (SPARK / PH vendor bridge, recipe in
[docs/bridges-vendor.md](docs/bridges-vendor.md)), suspension, multiple robots, seasons other
than 2026 (copy `plugins/_template`), a real Driver Station connection, loadable (shared-library) plugins, code signing
and MSI / deb / rpm packages, and other WPILib years for the extension (validated against 2026.2.1 only). Vision is limited to AprilTags
without occlusion, distortion or object detection (see [docs/vision.md](docs/vision.md)).
Contributions: [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
