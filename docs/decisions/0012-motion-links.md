# ADR 0012 - One-way modeled motion links

- Status: accepted (the legacy tank upgrade originally described here was removed by [ADR 0013](0013-pre-release-single-sdk-version.md))
- Scope: SDK, configuration, core runtime, mechanisms, drivetrain, and editor

## Context

A tank drivetrain can have one motor per side driving several wheels through chains.
Other mechanisms can share a drive with different ratios. Requiring a separate motor
mapping for every modeled wheel or mechanism cannot express these arrangements.
The product needs visible, sensor-consistent relationships, not accurate transmission dynamics.

## Decision

Add `RobotConfig::motionLinks`, whose records identify a source modeled axis, a target axis,
and a signed ratio in target SI units per source SI unit. Addresses use `SlotAddress` as an
owner/axis pair: drivetrain wheel outputs are `<wheel>.drive`; generic mechanisms expose
`axis`, or `deploy`/`roller` for a multi-axis intake. These are not device aliases, geometry
attachment frames (`LinkSpec`), or chassis-physics coupling constraints.

Each target has one source. Fan-out and chains are supported; cycles and self-links are errors.
Negative ratios reverse motion. Relative to configured initial positions,
`target = targetStart + ratio * (source - sourceStart)`. Followers clamp to their own
limits and zero outward velocity, but never stop or load the source.

The UI presents rev/rev, m/m, m/rev, or rev/m and converts to rad/m SI storage.
Followers skip their own motor/plant integration and report no motor battery draw.
Their final axis motion supplies visuals, encoders, limit switches, and game-facing state.
Independent motor mappings on a target are errors. Multi-motor source axes aggregate their motors as in
[ADR 0014](0014-multi-motor-gearboxes.md); links do not combine separately driven axes.

The session evaluates advertised axes in stable dependency order, then finalizes mechanism
state and side effects. Drivetrain contacts use prescribed follower spin without integrating
another motor. Their traction is approximate: there is no reflected inertia, reaction load,
power conservation, compliance, or shared-shaft solver.

The SDK additions are opt-in callbacks and data. `IMechanism::motionAxes()`
and `IDrivetrain::motionAxes()` default to empty; unsupported endpoints are rejected.
Implementers advertising axes must provide state/advance callbacks; mechanisms must also
override `finishMotionTick` to publish final state without advancing axes again.
Unlinked models use the normal tick/step path. No new wire signals/messages are needed.
Links are plain data: there is no load-time upgrade of tank layouts (ADR 0013); the wizard creates tank links with
`tankMotionLinks`; in the editor each wheel is linked from its own Drive card.

## Consequences

Simple source/target/ratio controls suffice. The editor exposes them as a per-axis "Driven by" field on the follower and shows followers nested, read-only and motor-less under their source in the project tree. All edits use existing undo
commands; no visual graph dependency is added. Renames rewrite endpoints, duplicates copy
incoming relationships, and deletion retains broken links for explicit correction/removal
and undo rather than silently retargeting them.

Simulation fidelity is intentionally limited. Clutches, physical differentials, arbitrary
HAL signal routing, and bidirectional mechanical networks need separate designs.
