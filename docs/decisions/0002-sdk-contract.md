# Pre-release SDK contract

Read this when changing a shared interface or persisted representation.
Prerequisites: [Architecture](../ARCHITECTURE.md) and [Contributing](../CONTRIBUTING.md).

## Status
Accepted.

## Rationale
Private cross-module interfaces drift, while compatibility layers preserve assumptions that may
not match the evolving design. During active pre-release development, the project can coordinate
changes across its callers instead of carrying multiple representations.

## Decision
Treat SDK headers as the shared source contract and update their consumers together. Do not create
private cross-module backchannels or compatibility shims for unreleased development revisions.

## Trade-offs
The tree can remove obsolete concepts, but projects and modules built from different revisions
are not promised to interoperate. Current schema and wire behavior belong in their references.

See the [runtime](../reference/runtime.md) and [extension/IPC](../reference/extension-ipc.md)
references for current contracts.
