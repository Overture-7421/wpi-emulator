# ADR 0013 - Pre-release: the SDK stays at version 1, no compatibility is kept

- Status: accepted
- Supersedes in part: [ADR 0005](0005-sdk-freeze.md) (freeze and additive-only rules), [ADR 0006](0006-sdk-1-1-amendments.md),
  the migration, unknown-field and `.bak` upgrade parts of [ADR 0010](0010-runtime.md), the SDK 1.3/1.4 numbering of
  [ADR 0011](0011-single-robot-instance.md) and the legacy tank upgrade of [ADR 0012](0012-motion-links.md)
- Scope: SDK, IPC protocol, `robot.json`, plugin sections, game plugins, editor and wizard

## Context

Nothing has been released. The tree nevertheless carried compatibility machinery for revisions that no user ever had:
a core `schemaVersion` migration chain with `.bak` write-back on load, plugin migrators and a whole-config
`migrateRobotConfig` hook, the game_2026 "v1 draft" migration, the legacy `MechanismConfig::parent` string next to the
structured attachment, unknown-field round-tripping, an automatic legacy tank-to-motion-links upgrade, and a minor-version
history ("SDK 1.1 ... 1.5", wire v2/v3). It all cost code, tests and documentation and constrained changes.

## Decision

* Until the first release the SDK is **1.0** (`kSdkVersionMajor = 1`, `kSdkVersionMinor = 0`) and the IPC protocol is
  **1** (`kProtocolVersion = 1`, `kMinCompatibleProtocolVersion = 1`). The constants and their checks stay (the handshake
  refuses a lower peer, `GameMetadata::minSdkMajor/minSdkMinor` still gate plugins); there is no version history. Any
  change, including breaking ones, may be made without an ADR or a version bump. Compatibility rules start at the first release.
* **No project migration.** `robot.json` and each plugin section carry `schemaVersion: 1`; any other value is refused
  (`VersionMismatch`). A missing core version reads as 1. There are no `IConfigMigrator`s, no
  `IGameModule::configMigrators` / `migrateRobotConfig`, no `.bak` upgrade copies (the explicit
  `pre-import` backup of `importFromProject` stays) and no on-load rewriting. Config loading order is: decode, defaults,
  validation, construction.
* One attachment form: `MechanismConfig::attachment`. No legacy `parent` string.
* Unknown fields are ignored on load and not written back (`RobotConfig::unknownFields` is gone).
* Motion links are plain data. There is no `motionLinksConfigured` flag and no load-time tank upgrade; the wizard
  creates tank links with `tankMotionLinks`, and the editor offers the same preset.
* Saved projects from earlier revisions are not supported; recreate them with the wizard or edit them by hand.

## Consequences

Simpler SDK, codec, pipeline and plugin template; fewer tests to maintain. Anything persisted by an earlier revision of
this tree may fail to load or lose fields. When the first release is cut, a new ADR re-introduces versioned evolution
rules (and migrations) from that baseline.
