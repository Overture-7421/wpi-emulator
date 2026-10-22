# Architecture decisions

Read this when you need the reasons behind a design constraint.
Prerequisite: [Architecture](../ARCHITECTURE.md).

These records explain adopted design constraints, not implementation inventories.
**Accepted** is a design status, not a delivery or test report.

## Index

| Decision | Rationale |
|---|---|
| [0001](0001-module-boundaries.md) | Module boundaries |
| [0002](0002-sdk-contract.md) | Pre-release SDK contract |
| [0003](0003-halsim-extension.md) | HALSim injection and vendor interception |
| [0004](0004-ipc-lockstep.md) | Local IPC with one outstanding tick |
| [0005](0005-physics-interface.md) | Abstracted physics and Box3D chassis |
| [0006](0006-single-robot-instance.md) | One long-lived robot and runtime composition |
| [0007](0007-generic-mechanisms-plugin-world.md) | Generic mechanisms and plugin-owned world |
| [0008](0008-in-process-vision.md) | In-process vision coprocessors |
| [0009](0009-start-placement-feedback.md) | Explicit start placement and calibrated feedback |
| [0010](0010-plugin-owned-season-data.md) | Plugin-owned season data |
| [0011](0011-one-host-contract.md) | One host contract and app layering |
| [0012](0012-shared-primitives-in-the-sdk.md) | Shared primitives live in the SDK |
| [0013](0013-game-sound-cues.md) | Game sound cues |

## Maintaining the set

Use Status, Context, Decision and Consequences sections; explain why and what is sacrificed.
Keep each record within 45 physical lines and this index within 35.
Contracts belong in [reference pages](../ARCHITECTURE.md#extension-points), not API inventories here.
For future decisions, choose the next unused number and add it here; keep assigned numbers stable, never reuse one.
Update rationale and links together when the current decision changes.

Next: [Contributing](../CONTRIBUTING.md).
