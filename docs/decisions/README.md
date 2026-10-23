# Architecture decisions

Read this when you need the reasons behind a design constraint.
Prerequisite: [Architecture](../ARCHITECTURE.md).

These records explain adopted design constraints, not implementation inventories. **Accepted** is a design status, not a delivery or test report.

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
| [0013](0013-sound-cues-as-events.md) | Sound cues travel as events, not snapshots |
| [0014](0014-updater-without-api-or-silent-install.md) | The updater needs no API and installs nothing silently |
| [0015](0015-managed-java-after-wpilib.md) | The emulator finds a working Java first and downloads one only when none works |
| [0016](0016-robot-setup-lives-in-the-project.md) | Robot setup lives in the project |

## Maintaining the set

Records explain rationale and trade-offs; current rules belong in the owning reference, not here.
Keep each record within 45 physical lines and this index within 35. Choose the next unused number;
never reuse or renumber one. Update the record and index together.

Next: [Contributing](../CONTRIBUTING.md).
