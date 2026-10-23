# Module boundaries

Read this when considering a dependency or a new module.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Season rules and robot-device adapters change for different reasons. If either spreads into core
or the UI, adding a season or device family requires unrelated edits and makes ownership unclear.

## Decision
Use the SDK as the narrow shared contract, keep backend and UI dependencies at their owning edges,
and compose built-in plugins and bridges through aggregators.

## Trade-offs
Explicit registration and dependency wiring take maintenance. In return, individual plugins and
bridges can evolve without making core or the app depend on their concrete implementations.

Current dependencies and owners are listed in [Architecture](../ARCHITECTURE.md).
