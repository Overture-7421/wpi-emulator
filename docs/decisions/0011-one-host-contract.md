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
`IRuntimeHost` lives in `app/contract`, a target that links only the SDK and Qt, and is the one
seam the run view, editor adapter and shell use. Robot-process, link, mode and log vocabulary is
one SDK header; Qt presentation structs convert from it in one place. Selecting the open project
is a separate interface, `IProjectBinding`, reached only through `AppServices`, so pages cannot
change the project. The scene carries the project path and the wizard's Driver Station choice,
which the run view applies once per project.

Inside `app/` the boundary checker has an owner per directory: the contract, editor and run
view see the SDK, the contract and two shared Qt-free headers; a small `app/ui` library holds
Qt helpers both page libraries share; common, wizard and shell compose them.

## Consequences
No second interface, adapter or enum set to keep in sync; fakes implement the same host.
The editor still builds without core. The cost is two small libraries (`app/contract`,
`app/ui`) and per-directory rules to maintain. Contracts belong in the
[app reference](../reference/app.md).

Next: [Shared primitives](0012-shared-primitives-in-the-sdk.md).
