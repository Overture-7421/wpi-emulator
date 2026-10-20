# Module boundaries

Read this when changing a dependency or adding a module.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Supporting another season must not require editing simulation or UI internals.
Device support must not couple vendor libraries to physics. Directory layout alone
cannot prevent these dependencies from creeping back in.

## Decision
Make the standard-library-only, header-only SDK the shared boundary. Keep simulation
game-agnostic and Qt-free; contain Qt and HAL/vendor backends at their respective edges.
The dependency graph in Architecture is authoritative rather than a collection of
per-module exceptions.

Compose built-in plugins through the plugin aggregator and robot-process bridges
through the bridge aggregator. Consumers depend on those seams, not concrete seasons
or bridge modules. Individual plugins and bridge modules remain independent.

Enforce includes and CMake dependencies with the
[boundary checker](../../tools/check_boundaries.py). Internal build targets inherit
their module's rules rather than providing a way around them.

## Consequences
A season or device family can evolve without spreading its implementation across
the app and core, and SDK fakes allow independent development.
Shared behavior must be expressed deliberately in the SDK, which adds contract work.
Registration and aggregation wiring are explicit maintenance costs; adding a feature
does not justify bypassing the boundary.

Next: [Extension recipes](../extending.md).
