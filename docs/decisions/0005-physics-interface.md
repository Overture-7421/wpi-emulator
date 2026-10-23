# Abstracted physics and Box3D chassis

Read this when changing physical models or the rigid-body backend.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Field and chassis interactions need rigid-body collision behavior, while drivetrain traction needs
wheel-specific force models. Exposing engine types to plugins, mechanisms or tests would couple
otherwise reusable simulation code to one backend.

## Decision
Keep Box3D behind the SDK physics interface and model tire traction separately from rigid-body
support. Mechanism loads affect the chassis through SDK coupling, not direct engine access.

## Trade-offs
The adapter owns backend translation and quirks; fakes cannot establish collision fidelity. The
chosen wheel/contact model is intentionally simpler than suspension or a full tire solver, trading
physical detail for a bounded, testable model.

Current physics contracts and limits are in [Physics](../reference/physics.md).
