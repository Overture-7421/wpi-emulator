# ADR 0012 - One-way modeled motion links

- Status: accepted
- Scope: SDK 1.5, configuration, core runtime, mechanisms, drivetrain, and editor

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
Independent motor mappings on a target are errors. Existing multi-motor source axes retain
their existing aggregation behavior; links do not combine separately driven axes.

The session evaluates advertised axes in stable dependency order, then finalizes mechanism
state and side effects. Drivetrain contacts use prescribed follower spin without integrating
another motor. Their traction is approximate: there is no reflected inertia, reaction load,
power conservation, compliance, or shared-shaft solver.

SDK additions are defaulted trailing data and opt-in callbacks. `IMechanism::motionAxes()`
and `IDrivetrain::motionAxes()` default to empty; unsupported endpoints are rejected.
Implementers advertising axes must provide state/advance callbacks; mechanisms must also
override `finishMotionTick` to publish final state without advancing axes again.
Unlinked models retain the old tick/step path. No new wire signals/messages are needed.

## Legacy tank upgrade

The global core schema remains 1. `motionLinksConfigured` distinguishes explicit independent
setups from legacy files. At the core pipeline upgrade stage, an unconfigured tank becomes
one source per chassis-local-Y side, preferring its sole mapped drive wheel. Other wheels
follow at 1:1; their motor placeholders no longer integrate or draw current.

No mappings are removed. Multiple mapped drive wheels on a side or ambiguous side geometry
produce a conversion issue requiring correction rather than guessing a source. An unmapped
side gets a deterministic unbound source and the normal missing-source mapping warning.
Upgrade reports are visible, and persistence uses the existing save path. Explicit independent
setups opt out. Direct SDK/config-codec calls are not automatically upgraded.

## Consequences

Simple source/target/ratio controls and tank presets suffice. All edits use existing undo
commands; no visual graph dependency is added. Renames rewrite endpoints, duplicates copy
incoming relationships, and deletion retains broken links for explicit correction/removal
and undo rather than silently retargeting them.

Simulation fidelity is intentionally limited. Clutches, physical differentials, arbitrary
HAL signal routing, and bidirectional mechanical networks need separate designs.
