# Contributing

Read this before changing code or reviewing a change.
For build and test commands, start with [Build and test](BUILDING.md).

## Before you push

- Keep the change focused. Avoid behavior changes outside its purpose.
- Configure with warnings as errors and run focused tests, then the relevant full suite.
- Run applicable boundary and documentation checks from [Build and test](BUILDING.md#run-tests-and-repository-checks).
- Explain any check you could not run and why.
- Do not commit build output, per-project `.wpiemu` data, generated screenshots, or logs.
- Keep scratch experiments in the git-ignored `spikes/` directory; do not commit them.

If you are new to the code, start with [How WPILib simulation works](concepts/wpilib-simulation.md)
for the robot-process flow, [Simulation, physics, and units](concepts/simulation-and-physics.md)
for how commands become motion, and [Qt in this project](concepts/qt-in-this-project.md)
for the desktop app. [Architecture](ARCHITECTURE.md) explains module ownership and how
a simulation tick moves through the program. The [runtime](reference/runtime.md),
[mechanism](reference/mechanisms.md), [physics](reference/physics.md),
[extension and IPC](reference/extension-ipc.md), [bridge](reference/bridges.md), and
[application](reference/app.md) references own the detailed contracts.
The [first-change tutorial](tutorials/first-change.md) walks through a small motor-preset
change and its test.

## Coding conventions

- Write C++20 without compiler extensions. Format C++ with [`.clang-format`](../.clang-format),
  which uses Google-based rules, four spaces, and a 100-column limit.
- Put non-app public headers under `<module>/include/wpiemu/...`. Keep paths and namespaces
  consistent with the owning module, including when copying an extension skeleton.
- Do not use Qt macro names (`signals`, `slots`, `emit`, `foreach`) as identifiers in
  non-app public headers. Those headers must stay safe to include from code that uses Qt.
- Return `sdk::Result<T>` or `sdk::Status` for recoverable errors. Catch third-party
  exceptions at the boundary and convert them; exceptions must not cross SDK interfaces.
  Use `contractViolation` for programming-contract violations.
- Use `std::unique_ptr` for owned objects. Inject collaborators through constructors or
  `SimContext`; borrowed services must outlive their users. Registries are instances,
  not globals or singletons.
- Simulation uses SI units and right-handed Z-up NWU coordinates. Chassis +X points
  forward, field +X points toward the opposing wall, +Y points left, and +Z points up.
  Store quaternions as `(x,y,z,w)` and add unit suffixes to plain scalar names such as
  `massKg`, `radiusM`, and `timeUs`. See [Physics](reference/physics.md).
- Simulation time is integer microseconds from `ISimClock`. Use `SimContext::rng()` for
  randomness. Do not use wall clocks or independent random generators in simulation.
- Follow the [thread ownership and tick order](ARCHITECTURE.md#processes-and-threads).
  Do not add cross-thread mutable access or make HAL callbacks block.
- The app uses MVVM: widgets talk to view-models, not runtime or persistence services.
  Use `tr()` (`ETR()` in editor models), `ThemeTokens`, and `UnitFormatter`. Make editor
  changes through undoable config operations. Keep the Z-up to Qt Y-up scene conversion
  in QML, not in saved coordinates. See [Application](reference/app.md).
- Wrap every combo box, spin box and slider in `ui::quietWheel` (`ui/widget_support.h`) so the
  mouse wheel scrolls the surrounding form instead of changing values.
- Preserve unrelated mechanisms, mappings, and plugin sections when changing config.
  Core owns persistence. Decode, apply defaults, validate, then construct. Validation
  issue codes are stable SDK keys; numeric JSON stream fallbacks use
  `std::locale::classic()`. See [Runtime](reference/runtime.md).
- Preserve the [device identity and feedback contracts](reference/bridges.md). A declared
  CAN address does not always identify a distinct physical simulator device.
- Season and WPILib facts live in the season packs (`seasons/<year>/season.json`). Do not write a season
  year, `#if` on a year or ABI, or a per-year branch in core, the app, bridges, CMake or workflows: read
  the generated `SeasonRegistry`, add per-ABI sources under `abi/<abi>/`, or add a launch profile.
  `python tools\check_seasons.py` enforces this. See [Season packs](reference/seasons.md).
- Define shared behavior once in the SDK. Derive consumers from that source instead of
  copying tables, validation rules, or helpers. See the motor table in
  [`motor_model.h`](../sdk/include/wpiemu/sdk/dynamics/motor_model.h) and
  [`Alliance`](../sdk/include/wpiemu/sdk/game/alliance.h).

## Reuse without changing contracts

Similar code may differ in precision, threading, lifetime, or dependency direction.
For example, IPC and Photon codecs share scalar byte helpers, not packet layouts.
Demo and real hosts share an interface, not process behavior. Merge implementations
only when evidence shows behavior stays the same and no forbidden dependency is added.
Keep watchdog recovery and queue-overflow policies with their owning subsystem.

## Tests

Add a focused regression test near the tests for the changed module. Prefer public
interfaces and the header-only [SDK fakes](../sdk/include/wpiemu/sdk/testing) over
implementation access.

Use that folder's `waitUntil` to wait for another thread or process; do not write a sleep
loop. Use `device_fixtures.h` for device announcements and hub-warning checks.
Use `fake_extension.h` for the robot-process side of the protocol and
`fake_emulator.h` for the emulator side. A failing robot is configuration, not a new fake.
Helpers that need platform calls, such as the `ScopedEnv` environment guard, live in
[tests/support](../tests/support).

Test the important failure or reset edge as well as the normal path. Do not add tests
for constants or language behavior merely to increase coverage. Use real physics when
geometry or contact behavior cannot be judged with a fake. Keep deterministic tests on
a fixed step and seed. Robot-project integration tests are opt-in, not required for
every unit test.

See [Build and test](BUILDING.md#run-tests-and-repository-checks) for test discovery,
CTest selectors, Qt test setup, and what a passing check does and does not prove.

## SDK and configuration changes

Before release, change the current SDK, protocol, and schema in place. Do not add
compatibility shims, migrations, legacy fields, or version bumps. Update callers,
contract tests, and documentation in the same change. The current contracts live in
[Runtime](reference/runtime.md) and [Extension and IPC](reference/extension-ipc.md).

A plugin's config change must stay within the plugin's ownership; do not rewrite the
whole robot model. For additions, start with [Extending](extending.md). Record a new
boundary or SDK design decision in the [decision index](decisions/README.md), not as an
old implementation narrative.

## Writing documentation

Give each fact one authoritative home and link to it instead of copying it. Use plain,
direct prose and explain unfamiliar terms when readers first need them. Put procedures
in guides, subsystem behavior in references, user-visible behavior in the user guide,
and design rationale in decisions.

Verify names, APIs, commands, and behavior against the source or by running the
documented command. Keep links and anchors current; run
`python tools\check_docs.py` from the repository root. That check finds broken local
links and anchors, but it does not prove a factual claim is correct.

Next: [make your first change](tutorials/first-change.md).
