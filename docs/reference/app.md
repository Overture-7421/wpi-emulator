# Application reference

**Read this when:** changing how the app composes the editor and run view, connects them to a
runtime host, presents snapshots, or offers updates.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for module and thread ownership;
[Contributing](../CONTRIBUTING.md) for UI conventions.

This page covers app-level contracts, not widget or service implementation details. Use the
[User guide](../USER_GUIDE.md) for controls and settings, [Building](../BUILDING.md) for
build and test commands, and [Extending](../extending.md) for feature recipes.
Configuration and robot placement belong to [Runtime](runtime.md); process communication
belongs to [Extension and IPC](extension-ipc.md).
For the app's Qt/UI ownership model, see [Qt in this project](../concepts/qt-in-this-project.md).

## Composition and runtime boundary

[`AppServices`](../../app/common/app_services.h) is the composition root. It creates the
shared registries and project services, then supplies one runtime host to the editor and run
view. Built-in games are reached through the plugin aggregator, not direct season dependencies.

The editor and run view share one [`IRuntimeHost`](../../app/contract/runtime_host.h).
That host exposes project presentation data, runtime status, snapshots and commands without
letting either page access mutable simulation state. Project selection is a separate
`IProjectBinding` responsibility owned by the shell and composed by `AppServices`.

The real host delegates process and simulation orchestration to Qt-free `RobotSession`;
Qt event-loop adaptation remains in the app. `--demo` selects a scripted host and does not
exercise robot launch or IPC. Keep these host boundaries intact when adding UI behavior.

Setup checks the extension selected for the live project season, listing available seasons
when it is missing (`no_extension_for_season`). A refused year handshake reports
`season_mismatch`. Native loader or initialization failures for `halsim_wpiemu` report
`extension_load_failed`, retaining the loader error in the console. These stable status keys
are translated into setup guidance by the run view; extension selection belongs to
[Runtime](runtime.md#launcher-boundary).

## Editor and run-view contracts

The editor works through `IEditorBackend` and an undoable document; it does not access core,
IPC or concrete game plugins. Edits are submitted to the backend, and saving is a separate
request. New edit surfaces must use the document's undo/redo path rather than mutating
runtime state or writing files directly. Before changing project identity or closing, the app
flushes pending edits to their original project; a failed save blocks the transition.
Metadata-driven property behavior and its recipe are in
[Editor properties](../guides/editor-properties.md).

The run view consumes `IRuntimeHost` and immutable scene descriptions. A latest snapshot can
be absent before the first tick, and the UI may skip intermediate snapshots. Repaint rate is
not simulation time. Host calls and delivered signals are UI-thread operations; hosts marshal
worker-thread results before delivering them to the views.

The Run-view camera is presentation state: Driver POV reads the latest snapshot's chassis pose
and smooths its gaze using elapsed UI-frame time, including while simulation is paused. Camera
presets never write simulation inputs or change the Driver Station alliance or station. The
viewport's Driver POV is distinct from simulated robot cameras, which publish vision results.

Launching robot code and starting simulation are separate operations. A live robot can stay
connected for device discovery; beginning and ending simulation enter and leave lock-step
without restarting the robot. Resetting simulation does not rerun robot initialization.
See [Runtime](runtime.md) for reset, configuration and start-placement behavior, and
[Extension and IPC](extension-ipc.md) for lock-step readiness and message ordering.

## Updates

Installed copies check for a newer stable release at startup when update checks are enabled
in Settings. The user can also request **Help > Check for updates**; only this manual check
shows a result message for success, unsupported installs or failure. Demo mode and development
builds do not offer updates. A portable install opens the release page instead of downloading
an installer.

For a supported installer, the app offers a download only when the release has an asset and
checksum for the current platform. It verifies the downloaded file against that checksum
before making installation available; a failed download or verification is discarded and can
be retried. The user can cancel a download or skip that release.

Installing a verified update asks the app to close where installation replaces the running
app. If a simulation is running, the user must confirm; unsaved edits are flushed before
closing, and a refused close leaves the update ready to install later. On macOS, installation
opens the verified disk image while the app stays open. The Help menu mirrors the button's
primary update action; the button's context menu offers release notes, skip and, during a
download, cancel. On macOS the Help menu is the primary action when the native menu bar has
no corner button.

**Next:** [Runtime and configuration](runtime.md).
