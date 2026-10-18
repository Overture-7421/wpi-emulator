# ADR 0016 - Zones replace game wrapper mechanisms

- Status: accepted
- Scope: `sdk/config/zone.h`, `sdk/game/zone_kind.h`, `sdk/game/zone_activation.h`, `RobotConfig::zones`, core codec and
  validator, game plugins, editor, run view

## Context

The 2026 plugin modelled the fuel intake and shooter as *mechanisms* (`game_2026.fuel_intake`, `game_2026.shooter`)
that wrapped the user's generic mechanisms by id. They were not machines: they owned no slots, no plant and no
motion, only a place on the robot (a pickup volume, a muzzle) and a rule for when it works (a roller spinning, a
flywheel at speed). Modelling that as a mechanism forced fake mechanism ids, fake slot specs and a plugin-specific
"deploy condition" that the editor could not show generically.

## Decision

* A **zone** is generic data on the robot model: `RobotConfig::zones` (`ZoneConfig`: `id`, `kind`, `label`,
  `attachment`, `local` pose, `sizeM`, `properties`, `activation`). It hangs on the chassis or a mechanism link with the same
  `MechanismAttachment` and link rules as a mechanism mount, so it moves with that mechanism.
* Core never interprets a zone. `kind` and `properties` are **plugin-owned**: a plugin owns the kinds starting with its
  id plus `.` (`"game_2026.pickup"`), declares them as `ZoneKindInfo` through `IGameModule::contributeZoneKinds()`
  (display name, `Volume` or `Muzzle` shape, `PropertySchema`, defaults, color) and builds the behaviour from the zones
  in its game state. Core validates ownership, ids, attachment, geometry, activation structure and the properties against
  the kind schema; the pipeline fills property defaults.
* **Activation** is a list of conditions on values mechanisms publish (`ZoneCondition`: mechanism id, `NamedValue` name,
  `above | below | within`, SI threshold/tolerance, absolute or signed) combined with `All` (AND) or `Any` (OR); an empty
  list is always active. `evaluateActivation` in `sdk/game/zone_activation.h` evaluates it against the
  `IMechanismDirectory`, converting published units to SI, and falls back to `MechanismState::active` when a value is
  not published. Conditions are data, so the editor offers them for every kind.
* Fuel intake and shooter stop being mechanisms; in game_2026 they become zone kinds. The plugin runtime, editor and
  run view are built on this API in later steps.
* **No migration** ([ADR 0013](0013-pre-release-single-sdk-version.md)): `zones` is read as empty when absent, SDK and
  `schemaVersion` stay 1, and projects that modelled intake/shooter as mechanisms are recreated by hand or with the wizard.

## Consequences

The plugin-specific wrappers, their fake slot specs and their private deploy conditions go away; every season gets
placeable, attachable, conditionally active zones from the same editor UI. Adding a season-specific zone is a
`ZoneKindInfo` plus game-state code and touches only the plugin. Zone ids share the namespace of mechanism ids for
validation clarity (a zone may not reuse a mechanism id). Zones have no slots or physics of their own: anything that
needs a device is still a mechanism, and the zone just observes its published values.
