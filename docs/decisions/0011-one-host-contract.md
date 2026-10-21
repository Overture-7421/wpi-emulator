# One host contract

Read this when a page needs something from the runtime or a new app library is proposed.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
The run view declared its own backend interface, enums and structs that mirrored the shell's
`IRuntimeHost` field for field, and an adapter copied between them. Three enum sets described
the same robot-process state. A wizard setting reached the Driver Station only on a code path
that never ran.

## Decision
`IRuntimeHost` lives in the `app/contract` include boundary, compiled by the SDK+Qt `app/ui` target, and is the one
seam the run view, editor adapter and shell use. Robot-process, link, mode and log vocabulary is
one SDK header; Qt presentation structs convert from it in one place. Selecting the open project
is a separate interface, `IProjectBinding`, reached only through `AppServices`, so pages cannot
change the project. The scene carries the project path and the wizard's Driver Station choice,
which the run view applies once per project.

Inside `app/` the boundary checker has an owner per directory: the contract and shared UI support
see the SDK; the editor and run view use that support, and the run view also sees the host contract.
Two shared Qt-free headers are allowed. Common, host, wizard and shell compose these layers.

## Consequences
No second interface, adapter or enum set to keep in sync; fakes implement the same host.
The editor still builds without core. The cost is a small shared `app/ui` library
and per-directory include rules to maintain. Contracts belong in the
[app reference](../reference/app.md).

Next: [Shared primitives](0012-shared-primitives-in-the-sdk.md).
