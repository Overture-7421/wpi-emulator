# Plugin-owned season data

Read this when season, field or tag data seems to belong in core, a bridge or the app.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Season facts leaked outside the game plugin: the Limelight bridge defaulted to the 2026 field
size, fakes and demos carried `game_2026` ids and a 2026 field frame, and the wizard and
scene code repeated field and tag constants. A new season would have meant hunting these down.

## Decision
The game plugin owns every season fact: field description and size, AprilTag layout,
zone kinds, game values, ids and display name. Consumers receive them through SDK contracts.
The field size travels `AprilTagLayout` to `VisionSetup` to the vision bridge; no bridge or app
code carries a default. Without a size, the Limelight bridge publishes only the blue-origin pose
and warns once per configure rather than inventing a field.

Facts that hold for every FRC season are named SDK constants, not season data: the 6.5 in tag
size and the generic floor shown when a project has no plugin field. Fakes and demos share one
neutral demo game in the SDK testing headers instead of a season.

## Consequences
A new season adds a plugin and touches no core, bridge or UI code.
A plugin that omits the field size loses red- and center-origin Limelight poses; the plugin
guide says so. A neutral demo game means demos no longer look like the current game.
Plugin contracts belong in the [game-plugin reference](../reference/game-plugin.md).

Next: [One host contract](0011-one-host-contract.md).
