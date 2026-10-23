# Generic mechanisms and plugin-owned world

Read this when deciding whether behavior belongs in core or a game plugin.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Many robot parts differ in dimensions and loads rather than in their underlying mechanism.
Season-specific field rules and game state, by contrast, should not make core or UI understand a
particular game. Treating a pickup zone as a mechanism would also invent device behavior it does
not have.

## Decision
Use reusable core mechanisms and plants for physical behavior; let game plugins own fields,
entities, rules and season-specific metadata. Connect game values to generic sensors through SDK
contracts rather than private state access.

## Trade-offs
Users supply physical properties instead of relying on game-role presets. Generic game-fed sensors
represent declared values, not geometric range or beam intersections.

Current boundaries are in [Mechanisms](../reference/mechanisms.md) and
[Game plugins](../reference/game-plugin.md).
