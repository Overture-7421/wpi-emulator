# Pre-release SDK contract

Read this when changing a shared interface or persisted representation.
Prerequisites: [Architecture](../ARCHITECTURE.md) and [Contributing](../CONTRIBUTING.md).

## Status
Accepted.

## Context
Modules need one definition of shared data, ownership and simulation services.
Private cross-module interfaces would drift. A compatibility promise during active
pre-release design would also preserve assumptions that no released user depends on.

## Decision
Use the SDK headers as the shared contract, including their lifetime, threading,
units and error semantics. Recoverable errors cross interfaces as values, not exceptions.
Do not work around a missing contract with a private cross-module backchannel.

Keep SDK **1.0**, configuration schema **1** and IPC protocol **1** as the single
pre-release baseline. These labels do not promise compatibility between development
revisions. Change interfaces, callers, tests and documentation together without
version bumps, legacy representations or migration machinery.

Decode the current configuration schema, apply defaults, validate, then construct.
Unsupported core and selected-plugin section versions are rejected; unknown core
fields are not a round-trip preservation mechanism. See the
[runtime contract](../reference/runtime.md) for loading and ownership details.

Built-in C++ plugins compile with the host's toolchain. The SDK is a source contract,
not a portable shared-library ABI.

## Consequences
The tree can remove obsolete concepts instead of maintaining parallel representations.
Saved projects from another development revision may fail validation or lose unknown
fields; matching version labels alone cannot make mixed revisions safe.
Header-only helpers and fakes keep consumers small, but shared changes affect every
consumer and must be integrated together.

Next: [Extension recipes](../extending.md).
