# Application reference

**Read this when:** changing the shell, editor, run view, or their runtime adapters.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for dependency/thread ownership and
[Contributing](../CONTRIBUTING.md) for MVVM, units, translation, and theme conventions.
Build and test commands belong to [Building](../BUILDING.md); controls belong to the
[User guide](../USER_GUIDE.md).

For a schema-driven edit, follow [Editor properties](../guides/editor-properties.md);
for host, persistence, or rendering diagnosis, use [Maintenance](../guides/maintenance.md).

## Composition and runtime boundary

[`AppServices`](../../app/common/app_services.h) is the application composition root.
It owns the registries, persistence/model services, project session, and injected runtime host;
`main()` passes it to the shell by reference. Built-in games enter through the plugin
aggregator, not direct season dependencies. Inside `app/` the boundary checker also enforces the
layering: the host contract, the shared Qt support (`ui`) and the page libraries (editor, run view) include
and link only the SDK (the page libraries also `ui`, the run view also the contract) plus two shared Qt-free
headers; `common`, `host`, `wizard` and `shell` compose them.

[`IRuntimeHost`](../../app/contract/runtime_host.h) is the UI-thread seam for robot-process and
link status, scene descriptions, immutable snapshots, logs, and run commands. It is compiled into the
SDK+Qt `wpiemu_app_ui` target (with `app/contract` enforced as its own include boundary), so the run view
consumes it without core. The editor adapter
and `RunPage` share the same host: discovery and the run view refer to the same robot instance.
Selecting the open project is a separate interface, `IProjectBinding`, reached only through
`AppServices::bindRuntime`; hosts implement both (`IProjectRuntimeHost`).

Normal startup constructs `RealRuntimeHost`, which delegates process/session work to the
Qt-free [`RobotSession`](../../app/host/robot_session.h). That host builds the configured
simulation, serves IPC through the runner, launches unmodified robot code through Gradle, and
owns teardown. Keep Qt event-loop adaptation in `RealRuntimeHost`, not in `RobotSession`.
Process and link state, session mode and console entries use one vocabulary,
[`session_status.h`](../../sdk/include/wpiemu/sdk/telemetry/session_status.h), from session to
UI; `HostStatus`/`HostLogLine` only add Qt strings through `toHostStatus`/`toHostLogLine`.
Session construction belongs to [runtime](runtime.md); injection and robot-process behavior
belong to [extension/IPC](extension-ipc.md).

Launching robot code is different from beginning simulation. The live robot can stay Armed
for discovery; starting and stopping simulation enter and leave lock-step without a reboot.
Restarting robot code relaunches it; resetting simulation does not. A same-project config edit
applies to the next run and may require relaunch. Deferred start intent survives same-project
relaunches but is canceled when the project changes. The selected start position follows the same
rule in every host through [`StartPositionSelection`](../../app/contract/start_position_selection.h):
it accepts only an id the config has and is dropped on a project switch or when its entry disappears.

Keep process state, link state, `programStarted`, and `simulating` distinct when enabling UI
actions. `HostStatus::runGeneration` changes for robot launches, not physics resets.
Discovery snapshots are reconciled by physical identity; absent devices remain non-live history
without erasing user mappings. Declared addresses are not physical identities; see [bridges](bridges.md).

`--demo` selects `DemoRuntimeHost`, a scripted host without a robot process.
`FakeEditorBackend` and `FakeRunBackend` are separate page-level implementations for tests and
standalone demos; they do not exercise the real host, launcher, or IPC. The fake editor backend
validates with the same SDK config checks as core, over its own `EditorCatalog` ([runtime](runtime.md)).
Both play the neutral [demo game](../../sdk/include/wpiemu/sdk/testing/demo_game.h): its plugin id, zone
kinds, game values, HUD, panels, field and tags are defined once there, every id starting with `demo_game.`,
and only the two demo robots differ. Season data belongs to a game plugin, never to a fake or a page library.

## Shell, wizard, and persistence

The shell coordinates the selected project through `ProjectSession` and `ShellController`;
the setup pages edit `WizardViewModel`. The wizard's `updateRobotConfig()` merges robot basics
into the existing model rather than replacing mechanisms, mappings, and plugin sections.
Do not build a fresh model over an edited robot merely to update wizard fields.
The wizard's wheels come from the SDK's [wheel layouts](../../sdk/include/wpiemu/sdk/physics/wheel_layout.h).
`basics_limits` holds the ranges its spin boxes offer and `validateBasics()` reports; they are presentation
limits, tighter than the physical validity [core validates](runtime.md), and a test keeps them inside it.

`ConfigRepository` adapts core's `ProjectConfigRepository`; widgets do not own persistence.
A save writes the project's `.wpiemu/robot.json`, the only copy. A failure keeps edits
dirty and offers retry guidance.

Shell Save, editor Save, autosave, and project-transition flushing use the same controller
persistence path, `EditorController::flushSave()` for explicit flushes.
The page factory installs a guarded pending-save handler on `ProjectSession`. Opening or
closing a project, closing the window, and setup replacement flush the old document before
changing identity; a failed authoritative save blocks the transition.
The adapter binds pending edits to their original project so they cannot be saved to another.

## Editor document and backend contract

`EditorPage` talks only to [`IEditorBackend`](../../app/editor/editor_backend.h), not core,
IPC, or concrete plugins. The backend supplies config, validation, discovered devices, and an
`EditorCatalog` assembled from registries and selected-game descriptors. Edits are submitted
live with `submitConfig()`; persistence is separately requested with `saveConfig()`.

`EditorDocument` owns the editable config and `QUndoStack`. Config operations produce before/
after snapshots for `ConfigEditCommand`; compatible consecutive property edits and complete
gizmo drags merge into undo steps. New edit surfaces must use this document path, not mutate
backend config or write files independently.

A backend `configReplaced` reload clears undo history. Distinguish authoritative undo/redo/
load notifications from echoes of live typing: authoritative changes update even a focused
field, while edit echoes and validation refreshes preserve its text and cursor.
Discovery directory replacement must also retain non-live device history, not stale liveness.

Config, selection, validation issues, and render items are separate presentation concerns.
The editor's scene represents the editable robot, not the running game's mutable state.
Keep transformations and picking in the controller/model contract rather than introducing
simulation access from QML. The [extension recipes](../extending.md) own new-property wiring.

## Metadata-driven inspector

The inspector combines editable names with **Drive**, **Configure**, **Placement**, and
**Shapes** cards. Forms come from
[`PropertySchema`](../../sdk/include/wpiemu/sdk/config/property_schema.h): persisted keys,
types, defaults, bounds, unit hints, choices, and conditional visibility.

`PropertySpec::section == PropertySection::Drive` selects Drive properties;
`PropertySpec::axis` splits mechanism Drive forms by motion axis, with an empty axis joining
the first group. A property's display `group` does not select that structural card.
Other part properties appear in Configure; attachment/pose and geometry have their own
generated groups. A steered wheel's steering controller is in Configure, not its drive group.
See [property grouping](../../app/editor/model/property_groups.cpp) before changing metadata.

Motion-link editing is per axis. Linking removes that axis's motor mappings in the same undo
operation; sensor slots remain. Followers appear under their source as read-only tree rows,
not draggable attachment children. Multi-motor rows are generated from slot count metadata;
dropping onto a full growable motor group can add a motor and mapping atomically.
Underlying motion and slot semantics belong to [mechanisms](mechanisms.md).

The selected game's schema, panels, zone kinds, game values, and field frame flow through
`EditorCatalog`; widgets do not recognize season ids or concrete zone kinds.
The game node's configuration is schema-generated, and contributed templates/zone kinds
supply context-menu entries. Do not add a season-specific inspector.

Zones get metadata-defined properties, attachment/placement, and an Activation card.
Volumes have editable size; muzzles have a point and direction. The editor does not show live
activation; [game plugins](game-plugin.md) define its runtime meaning.
Reattaching a zone or camera preserves its world pose. Removing a parent reparents attached
zones/cameras, but activation references can remain broken and must stay visible as issues.

Game-fed sensor forms use catalog game-value labels and retain unavailable saved sources;
their source contract is in [game plugins](game-plugin.md). They have no viewport marker.
Cameras are independent of season ownership; the editor shows their mount/frustum and a copyable
robot-to-camera transform for robot code. Vision semantics belong to [bridges](bridges.md).

Named start positions are edited numerically because the editor shows no field.
Alliance copying uses the catalog's field symmetry and is disabled without it.
Placement/reset behavior belongs to [runtime](runtime.md), not viewport coordinate conversion.

## Run-view snapshots and commands

`RunPage` consumes [`IRuntimeHost`](../../app/contract/runtime_host.h) directly.
`scene()` supplies immutable config/field copies and game presentation descriptors;
`latestSnapshot()` supplies the latest immutable `ISimSnapshot`, which can be null before a tick.
The view may miss intermediate snapshots; do not treat its repaint rate as simulation timing.

`IRuntimeHost` exposes lifecycle, pause/step, speed, placement and Driver Station capabilities.
Current Run controls request reset/restart, speed, placement and Driver Station changes;
the shell starts simulation automatically on entry, with no start/stop/pause/step buttons.
The real host reaches `ISimController` through `RobotSession`, never mutable simulation
objects. Host methods and delivered signals belong to the UI thread; worker-originated
Qt signals require queued delivery (`HostLogLine` and `DsState` are registered once, by the
`IRuntimeHost` constructor).

`HostScene` carries the open project's path and the wizard's Driver Station choice. `RunPage`
applies that station to its Driver Station panel when the project (or the choice) changes and
never on a same-project config edit, so a later change made in the panel is kept.

The Driver Station's timed sequence advances from snapshot simulation time, not wall time.
Its manual game-message text is a fallback when the game provides FMS data.
Joystick input stays behind `IJoystickProvider` / `IJoystickSource`; gamepads use normalized
WPILib Xbox layout. Hiding the page or losing activation releases held keyboard inputs.
Bindings, shortcuts, restart choices, and Driver Station controls are in the
[User guide](../USER_GUIDE.md); optional gamepad dependencies are in [Building](../BUILDING.md).

## Sound effects

Practice plays the field's audio cues (manual Table 5-4). Two sources ask `SfxPlayer` for a cue id and neither
plays audio itself: `DriverStationModel::sfxRequested` (only in Practice: match start, AUTO end, TELEOP start, match
end, the Restart chime, and the foghorn for an E-Stop of a running match, never for Disable) and
`IRuntimeHost::soundCue`, the game plugin's requests. The real host drains them from `RobotSession::takeSoundCues()`
in `pump()`; cues are never queued while the page is hidden. The [game-plugin reference](game-plugin.md#sound-cues) owns
the id contract and the REBUILT cue table.

`SfxCatalog` maps ids to resources with two rules and no season knowledge: common cues play
`app/runview/assets/sfx/<asset>.wav` (embedded under `/wpiemu/runview/assets/sfx`), a game's cues play files its plugin
ships in `plugins/<id>/assets/sfx/`, which the `plugins/` aggregation seam hands to the run view's CMake as the global
property `WPIEMU_BUILTIN_PLUGIN_SOUNDS` (embedded under `/wpiemu/plugins/<id>/sfx`). A missing file or a cue id without
the plugin's prefix is reported once in the console. Alliance-selection sounds (`pick_clock*.wav`) are not embedded.

Playback is `ISfxOutput`: Qt Multimedia `QSoundEffect` in the product, a recording fake in tests, and silence on the
offscreen platform or without an audio device. The page is silent while hidden and when sound is off; the on/off switch
and 0-100 volume belong to **File > Settings** (`AppSettingsData`), and the shell applies them with
`RunPage::setSoundEffects`, so the run view never sees `app/common`.

## Updates

`UpdateService` (`app/common`, owned by `AppServices::updates()`) checks GitHub once per start and on
**Help > Check for updates…**; it asks only `github.com` (rationale in the
[decision index](../decisions/README.md)). It follows the redirect of `releases/latest` to the stable tag, compares it with the
running version, derives the installer name for this platform (`update_offer`) and reads the `.sha256` sidecar beside it.
Anything else (no release, no installer for this computer, no sidecar) is "up to date"; a malformed sidecar or a network
error is silent at start-up and reported only for a manual check. All requests go through `IUpdateTransport`
(Qt Network: https to `github.com` and `*.githubusercontent.com` only, five redirects, size caps, no cookies) and every
effect after the download through `IUpdateInstaller`, so tests script both.

`detectInstall` classifies the copy from `ISystemProbe`: Windows installer (`Uninstall.exe` beside `bin`), macOS bundle
outside `/Volumes`, AppImage (`APPIMAGE`), portable (packaged layout without an installer: the button opens the release
page) or development (version `0.0.0` or any other layout: never checks). `--demo` and **Settings > Check for updates when
the app starts** switch the start-up check off; the skipped version is kept under `updates/skippedVersion`.

States run Idle, Checking, Available, Downloading, Verifying, Ready, Installing, with Failed after a download that failed
or did not match its SHA-256 (the file is deleted; retrying downloads again). The installer lands in `%TEMP%\wpiemu-update`
(Windows), Downloads (macOS) or beside the AppImage. **Install** then means: Windows and Linux ask the window to close
(`installRequested`; `MainWindow` confirms only while a simulation runs, and `closeEvent` still flushes saves or refuses,
which returns to Ready) and the installer is started from `aboutToQuit`, through the shell on Windows because it
elevates; macOS opens the disk image and the app keeps running.

`presentUpdate` maps state to the `UpdateButton` text, tone and tooltip, and to the Help menu mirror (a native macOS menu
bar has no corner widget). The button is the menu bar's corner widget with `[updateButton="true"]` and `[tone]` QSS roles;
its context menu offers release notes, skip this version and cancel download. Only a manual check reports its outcome in
a dialog. The opt-in transport test (`WPIEMU_E2E_NETWORK=1`) talks to github.com.

## Emulator Java and build tools

Which Java runs a project is decided once, by [`ToolchainChecker::resolveJdk`](../../app/common/toolchain_checker.cpp): the wizard's Java
row reports it and `AppServices` passes the same folder to the launch (`RealRuntimeOptions::ToolchainOverrides`), so the two cannot
differ. The order and the rules are in the [runtime reference](runtime.md#launcher-boundary). A launch with no usable Java never
starts Gradle: `RealRuntimeHost` fails it with `no_usable_java` (or `java_setting_unusable` for a rejected Settings folder).

`ToolchainProvisioner` (`AppServices::provisioner()`) is the only code that downloads Java, and only when the Toolchain page's
**Get Java** button is confirmed: a row offers it (`FixAction::GetJava`) only when no JDK passed and `versions.json` maps the project's
season to a Java major with a pin for this computer. Steps: disk space, download through its own `IUpdateTransport` (github.com
only), SHA-256 against the pin, unpack with the system `tar` (Windows: `System32\tar.exe`), then on Windows replace the Visual C++
runtime DLLs with those installed beside the app **only if** the runtime that Java would load is older than 14.40
(`loadedVcRuntime`), run `java -version`, and move it to `<local data>/toolchains/<vendor>-<version>` with a `.wpiemu-ready` marker. Anything
that fails removes its partial files; Settings shows the install and removes it. Tests use `IUpdateTransport`, `IToolchainEffects`
and `FakeProbe`; `WPIEMU_E2E_NETWORK=1` also downloads the real Java once.

`BuildToolsInstaller` backs `FixAction::InstallBuildTools` on Windows C++ projects: after a confirmation it runs
`winget install … Microsoft.VisualStudio.2022.BuildTools` with `--passive` (Microsoft's window and the administrator prompt), or opens
the Build Tools download page when winget is missing, and the toolchain is re-checked afterwards. The C++ row counts Visual Studio
2022 17.9+ with the C++ tools and does not flag one it accepts.

Failures the launcher classifies (`diagnoseLaunchOutput`) reach the run view as `HostStatus::problem` and its fields. `presentStatus`
words them and `StateOverlay` offers **Fix in setup** (the shell opens the wizard on the Toolchain step) or **Open Settings** through
`ShellPages::onProblemAction`; the run view never includes `app/common`.

## Scene presentation and resources

The run renderer relies on these frame contracts:

* The snapshot chassis pose is the center of mass; primitives and mechanism mounts use that frame.
* Wheel hubs come from contact points plus radius when grounded; steering/spin come from wheel samples.
* Mechanism geometry follows its first published moving link; without a link it stays at the root.
* Robot-zone overlays resolve the attachment link × local pose. Snapshot zone ids drive activation;
  a missing state is inactive. Panel-descriptor overlays are separate static config-derived boxes.
* AprilTag layout comes from the game; camera snapshot visibility highlights the union of seen tags.
  The run scene does not draw the editor's camera bodies/frusta.
* Field visuals come from the game's `FieldDescription`; one with `opacity` below 1 is drawn alpha-blended.

`Robot state` is a separate window reading mechanisms, sensors, zones, and cameras from the
snapshot, with names/source labels from scene metadata. It refreshes while visible and follows
page hide/show. HUD widgets likewise combine game descriptors with snapshot values; no season
rules belong in their renderer.

Both viewports are thin QML renderers fed by C++ presentation objects. The owning CMake resource
lists compile QML under `/wpiemu/editor` and `/wpiemu/runview`; load resource URLs rather than
assuming a source checkout or working directory. Keep new assets in those resource lists.
Scene plumbing both viewports need lives in the SDK+Qt `wpiemu_app_ui` target ([`app/ui`](../../app/ui)):
SDK-to-Qt conversions, Qt Quick 3D's built-in mesh conventions, colour parsing and the `ZUpWorld` QML
component under `/wpiemu/ui`. Add scene code there only when the editor and run view both use it. The same target
holds the text and widget helpers any layer shares: `qs()` string conversion, the `Tr` translation-context helper
(`ETR` is its `"Editor"` instance), clock text and small label recipes.
The widget modules disable AUTOUIC because
`sdk/include/wpiemu/sdk/game/ui_descriptors.h` matches its generated-header pattern.

Keep SDK NWU Z-up data unchanged in config, physics, and C++ scene models. The one conversion to
Qt's Y-up scene is the -90-degree X rotation of [`ZUpWorld.qml`](../../app/ui/qml/ZUpWorld.qml), which
[`Viewport3D.qml`](../../app/editor/qml/Viewport3D.qml) and
[`RunScene.qml`](../../app/runview/qml/RunScene.qml) use as the parent of all SDK-frame content; lights
and other Y-up content stay outside it. Do not pre-convert persisted poses or convert again
in an adapter. The editor's display floor origin is below the hull center, not at its COM.

QML context objects must outlive the view using them. In particular, `RunPage` destroys its
viewport before its member presentation models; preserve this lifetime ordering when rewiring.
Theme-colored icons are rendered through `IconProvider`, but stylesheet arrows are fixed-color
SVG image resources: their loading also requires Qt's SVG image-format plugin. Deployment belongs
to [Releasing](../releasing.md). Translation extraction must recognize editor `ETR()` as a
`translate` alias; the shared display conventions are in [Contributing](../CONTRIBUTING.md).

## Tests and visual checks

[Shell/wizard tests](../../app/tests), [editor tests](../../tests/app_editor), and
[run-view tests](../../tests/app_runview) (which also cover the shared `app/ui` helpers) exercise models,
edit operations, adapters, and GUI smoke
behavior. Their CTest suites are `app.ui_tests`, `app_editor.tests`, and `runview.ui_tests`;
use [Building](../BUILDING.md) for invocation and environment setup.
Editor test sources use `.cc` to stay outside the Qt-free `.cpp` test glob.

Offscreen tests do **not** validate Qt Quick 3D rendering. Use a rendering-capable desktop
platform and the standalone demos for visual checks; `WPIEMU_BUILD_UI_DEMOS` controls their build.
The [editor demo](../../app/editor/demo/editor_demo.cpp), `wpiemu_editor_demo`, accepts
`--grab <png>` for capture, `--size <WxH>`, `--select <item>`, `--detect`, `--tank`, and `--light`.
The [run-view demo](../../app/runview/demo/main.cpp), `wpiemu_runview_demo`, accepts
`--scenario <name>` (including `running` and `practice`), `--screenshot <png>`, and `--size <WxH>`.
`--state` also opens Robot state and requests a companion `<png>.state.png` capture.
Use the same size, selection/scenario, and theme when comparing visuals; these demos use fakes,
not a real robot or a guarantee about launcher/IPC behavior.

**Next:** [Extend the simulator](../extending.md).
