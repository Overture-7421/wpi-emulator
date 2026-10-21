# Contributing

Read this when preparing a change or reviewing someone else's.
Prerequisites: [Architecture](ARCHITECTURE.md) for ownership and [Building](BUILDING.md) for commands.

## Before you push

Keep each change focused and preserve behavior outside its scope. Configure with warnings as errors,
run the relevant tests, then the applicable full suite and boundary/documentation checks from
[Building](BUILDING.md#tests-and-checks). Explain anything you could not validate.

Do not commit build output, per-project `.wpiemu` data, generated screenshots or logs.
Scratch experiments go in the git-ignored `spikes/`; never commit them.

## Coding conventions

- Use C++20 without compiler extensions. [`.clang-format`](../.clang-format) defines Google-based
  formatting with four spaces and a 100-column limit. Comment only what needs clarification.
- Non-app module public headers live under `<module>/include/wpiemu/...`; keep paths and namespaces
  consistent with their owner when copying an extension skeleton.
- Non-app public headers must not use Qt macro names (`signals`, `slots`, `emit`, `foreach`)
  as identifiers, even when the header itself includes no Qt.
- Recoverable errors are `sdk::Result<T>` / `sdk::Status`. Catch third-party exceptions at the
  boundary and convert them: exceptions must not cross SDK interfaces. Programming-contract
  violations use `contractViolation`, not recoverable error handling.
- Use `std::unique_ptr` for owners and constructor/`SimContext` injection for collaborators.
  Borrowed services must outlive their consumers. Registries are instances, not globals/singletons.
- Internal units are SI in right-handed Z-up NWU coordinates: chassis +X forward, field +X toward
  the opposing wall, +Y left, +Z up. Quaternions are `(x,y,z,w)`; add unit suffixes to plain scalar
  names, such as `massKg`, `radiusM` and `timeUs`.
- Simulation time is integer microseconds from `ISimClock`; randomness comes from `SimContext::rng()`.
  Do not use wall clocks or independent randomness in simulation logic.
- Follow [thread ownership](ARCHITECTURE.md#processes-and-threads); do not add cross-thread mutable
  access or blocking HAL callbacks to make a local feature work.
- UI follows MVVM: widgets use view-models, not persistence/runtime services directly. Use `tr()`
  (`ETR()` in editor models), `ThemeTokens` and `UnitFormatter`. Editor edits go through undoable
  config operations. Keep the Z-up to Qt Y-up scene conversion in QML, not persisted coordinates.
- Preserve unrelated mechanisms, mappings and plugin sections when editing configuration.
  Persistence and load ordering are defined by the [runtime contract](reference/runtime.md).
  Validation issue codes are stable SDK keys; numeric JSON stream fallbacks use `std::locale::classic()`.
- For device changes, preserve the [identity and feedback contracts](reference/bridges.md);
  a declared CAN address is not necessarily a distinguishable physical simulator device.
- Define anything several modules need once in the SDK and derive from it, as the
  [motor table](../sdk/include/wpiemu/sdk/dynamics/motor_model.h) and
  [`Alliance`](../sdk/include/wpiemu/sdk/game/alliance.h) do; do not copy tables, validation rules or helpers between
  modules.

## Look-alikes kept on purpose

The simplification audit found these differ in behavior, precision, threading or dependency direction.
Merge one only with proof that it removes complexity without changing behavior or adding a module edge:

- Formatters and parsers: the run-view HUD, state and stats formatters; extension versus IPC environment parsing.
- Layers: IPC framing, framed transport, codec and channel; core versus extension link types, `IpcLink`'s outbound
  side, watchdogs and process readers; the vendor backend chain; the wizard's input ranges versus core validation;
  the config codec versus the SDK value codec; Photon's wire format versus the IPC codec (only bytes are shared).
- Math: motor, tire and mechanism dynamics (analytic gearbox versus numeric `DcMotorModel`); the vision bridges'
  double-precision vendor math versus SDK float poses.
- Hosts and shells: demo versus real runtime host, editor versus run-view lighting, POSIX versus Windows processes.
- Targets and tests: `wpiemu_halsim_core`, `wpiemu_core`, per-boundary test harnesses, the shared
  `PluginRegistrar` and init-script seams, the core-only runtime sample config.

## Tests

Add a focused regression test for changed behavior, next to the tests for that module. Prefer public
interfaces and the header-only [SDK fakes](../sdk/include/wpiemu/sdk/testing) over implementation access.
Wait on another thread or process with that folder's `waitUntil`, not a hand-written sleep loop, and build device
announcements and hub-warning checks with its `device_fixtures.h`. Play the robot-process side of the protocol with its
`fake_extension.h` (a failing robot is configuration, not a new fake) and the emulator side with `fake_emulator.h`.
Test helpers that need platform calls, such as the `ScopedEnv` environment guard, live in [tests/support](../tests/support).
Cover the important failure/reset edge as well as the happy path; do not test constants or language
behavior merely to increase coverage.

Use real physics when geometry/contact behavior cannot be judged by a fake. Keep deterministic tests
on a fixed step and seed. Robot-project integration is opt-in, not a prerequisite for every unit test.
Source discovery, naming restrictions, Qt suite handling and visual checks have one home in
[Building](BUILDING.md#tests-and-checks).

## SDK and configuration changes

Before release, change the single current SDK/protocol/schema in place. Do not add compatibility
shims, migrations, legacy fields or version bumps. Update callers, contract tests and documentation
in the same change. The current schema and wire contracts are in
[runtime](reference/runtime.md) and [extension/IPC](reference/extension-ipc.md).

A plugin's config changes must stay within its ownership, not rewrite the whole robot model.
For additions, use [Extending](extending.md). A new boundary or SDK design decision needs a short
decision record; use the [decision index](decisions/README.md), not an old implementation narrative.

## Documentation

One fact has one home; link instead of copying:

- [Architecture](ARCHITECTURE.md): module/dependency table, ownership overview and vocabulary.
- [Building](BUILDING.md): prerequisites, commands, options and running tests.
- [Extending](extending.md): every add-a-feature recipe; [reference](reference/runtime.md): subsystem contracts.
- [User guide](USER_GUIDE.md): user-visible behavior; [releasing](releasing.md): artifacts/deployment.
- [Decision index](decisions/README.md): design rationale, not subsystem manuals.

Keep page ceilings: Architecture 200 lines, Building 120, this page 110, Extending 220.
Reference pages stay within their topic budgets; decision records are at most 45 lines each.
Remove duplication rather than hiding it in oversized lines or creating forwarding stubs.

Use descriptive, unnumbered H2 headings with stable anchors. Open with “Read this when” and
prerequisites, end with an actionable Next link, and keep links, anchors and repository paths valid.
Pass `docs.check`; a successful link check does not prove an API or behavioral claim is correct.
Verify named APIs, flags and commands against source. Drop status/history, roadmap, product-version
and test-count inventories. Outside the decisions directory, link only to its index, never a
numbered record or citation.

Next: [extend a feature through the SDK](extending.md).
