# Application shell (`app/`)

Qt Widgets shell around the editor and run view. This folder is split by stream:

| Dir | Owner | Contents |
|---|---|---|
| `common/` | shell | Models without widgets (`AppSettings`, `RecentProjects`, `ProjectSession`, `ToolchainChecker`, `GradleProjectInspector`, `RobotBasics`, `ConfigRepository`), the runtime seam (`IRuntimeHost`, `DemoRuntimeHost`), theme + icons, `AppServices` (composition root) |
| `wizard/` | shell | First-run / "Set up robot" wizard: `WizardViewModel` + thin page widgets, `RobotSchematicView` |
| `host/` | integration | Qt-free `RobotSession` (config -> session -> IPC -> launcher) + its tests and the opt-in end-to-end test |
| `shell/` | shell | `MainWindow`, `ShellController`, settings / about dialogs, welcome page, and the adapters to the editor and run view |
| `editor/`, `runview/` | other streams | `EditorPage` / `RunPage` and their backend interfaces. Optional: the shell shows a placeholder when a library is missing (`-DWPIEMU_APP_USE_EDITOR=OFF`, `-DWPIEMU_APP_USE_RUNVIEW=OFF` force it) |
| `resources/` | shell | SVG icons (monochrome, `currentColor`), compiled into `:/wpiemu/icons/` |
| `tests/` | shell | `wpiemu_app_tests` (doctest): models, view-models and offscreen GUI smoke tests; CTest `app.ui_tests` |

## Rules of the road
* No singletons: `AppServices` is created in `main()` and passed by reference. Tests build one with an in-memory store, a temp app-data folder, a fake machine (`ISystemProbe`) and `DemoRuntimeHost`.
* MVVM: pages read and write a view-model (`WizardViewModel`, `ShellController`); only view-models touch `ProjectSession`, `ConfigRepository` or the runtime.
* Persistence is core's: `core::config::ProjectConfigRepository` (app-data copy + `<project>/.wpiemu/robot.json`) behind `ConfigRepository`, validated by `ConfigPipeline`. The wizard never replaces a model wholesale: `updateRobotConfig()` keeps mechanisms, mappings and plugin sections.
* Colours come from `ThemeTokens`; style sheet and palette are generated (`buildStyleSheet`). Icons are recoloured per theme by `IconProvider`. The spin-box / combo arrows are fixed-colour SVGs used from the style sheet, so the Qt SVG image-format plugin (`imageformats/qsvg`) must be deployed with the app.
* Strings go through `tr()`; units are SI internally and converted by `UnitFormatter`.
* `createShellPages` installs the optional editor's result-aware pending-save handler on
  `ProjectSession` (guarded by `QPointer`). Project open/close, window close and setup replacement
  flush the old document before changing identity. A failed authoritative save blocks the
  transition with retry guidance; builds without the editor need no handler.
* File > Save and editor Save both use `EditorController::flushSave()`. Backend saves return
  `sdk::Status`: only app-data success marks the document clean; export failure remains a warning.

## Runtime seam
Everything that runs the simulation sits behind `IRuntimeHost` (start / stop / reset / pause / step / DS state, status, scene, snapshots, log lines). `EditorBackendAdapter` and `RunBackendAdapter` share one host, so a device found in the editor is the one the run view shows.

* `--demo` uses `DemoRuntimeHost` (scripted device discovery and chassis motion, no processes).
* Without it `AppServices` creates `RealRuntimeHost` (`common/`): it validates the model, builds the Box3D/game-plugin session, serves the IPC endpoint, launches the unmodified project through Gradle with the HALSim extension and reports process/link state, snapshots and console lines. The Qt-free work is `host/RobotSession` (see docs/runtime.md section 6); failures (no gradlew, build failure, handshake timeout, protocol mismatch, crash, user stop, app exit) map to `HostStatus`. Detect devices = the Armed phase of the single long-lived robot (ADR 0011): `launch()` starts it, `startSimulation()` sends `BeginLockstep`, `stopSimulation()` sends `EndLockstep`, `restart()` relaunches.
* `HostStatus::runGeneration` identifies robot-code launches. The editor reconciles complete
  discovery snapshots by physical identity and retains absent devices as non-live history without
  changing user mappings. `SimulationStartIntent` scopes a deferred Run request to the project
  generation: project replacement cancels it, while same-project config relaunch/restart retains it.

## Command line
`wpiemu_app [--demo] [--project <folder>] [--version]`
