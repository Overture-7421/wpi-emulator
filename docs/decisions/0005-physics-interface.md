# Abstracted physics and Box3D chassis

Read this when changing physical models or the rigid-body backend.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Field and chassis interactions need general rigid-body collisions, but drivetrain
traction needs wheel-specific behavior. Exposing an engine directly would couple
plugins, mechanisms and tests to its types and quirks.

## Decision
Use Box3D behind the SDK physics interface, with opaque handles and engine-independent
geometry. Keep its headers and backend-specific corrections inside the adapter.
A pinned engine limits dependency churn without making its API the application contract.

Separate support from traction: one rigid chassis rests on frictionless wheel pads,
while tire models supply tangential forces. Quasi-static grounded-wheel loads avoid
feeding transient contact-impulse spikes into grip. A coupled wheel/chassis tire update
avoids the low-speed instability of an explicit stiff slip force.
This deliberately avoids building a suspension or a full tire-contact solver.

Use fixed steps, a single physics worker by default and controlled floating-point
settings to favor reproducible behavior. Engine-free fakes exercise consumers.
Mechanisms affect the chassis through SDK coupling for wrenches, constraints and
payload mass, rather than reaching into Box3D.

## Consequences
The backend can change without spreading engine dependencies through the product,
but the adapter must maintain translation, lifetime and engine-quirk handling.
Fakes cannot establish collision fidelity, and deterministic stepping is not a promise
of bit-identical results across every compiler and platform.
No suspension, approximate load transfer and simplified tires trade physical fidelity
for stable, understandable simulation; they are not a hardware-accuracy guarantee.
Model and coupling details belong in the [physics reference](../reference/physics.md).

Next: [Mechanisms reference](../reference/mechanisms.md).
