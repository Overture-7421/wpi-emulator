# Generic mechanisms and plugin-owned world

Read this when deciding whether behavior belongs in core or a game plugin.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Many robot parts differ in physical properties, not in their game-facing names.
A pickup volume or muzzle is a location and rule, not another motor-driven machine.
Season-specific sensor downcasts would expose private game state to core.

## Decision
Use one generic core mechanism template with rotary, arm or linear motion.
Reusable plants own their small dynamics equations instead of linking WPILib natives,
keeping core independent of the robot toolchain.
Controllers on one axis contribute individual commands and currents so fighting
motors are not silently replaced by an assumed leader.

Plugins own their field, entities, rules, namespaced configuration, zone kinds and
game values. Attachable zones observe published mechanism values; they are not fake
mechanisms with invented devices. Core validates structure and metadata without
interpreting season behavior; the UI edits the same declarative model.

Core-owned distance and beam-break sensors turn plugin-published values into feedback
after the game tick, through SDK contracts rather than private state access.
Unavailable sources yield idle readings and warnings rather than invented truth.
The [game-plugin reference](../reference/game-plugin.md) defines the ownership boundary.

## Consequences
New seasons reuse mechanisms, zone editing and sensor responses without core/UI rules.
Generic parts require users to choose physical properties rather than a game-role preset.
One-way motion links prescribe follower motion without reflected load, power conservation
or backpressure when a follower hits a limit; they are not a transmission solver.
Game-fed sensors measure declared game values, not geometric beam or range intersections.
Plant, controller and link contracts belong in the
[mechanisms reference](../reference/mechanisms.md).

Next: [Extension recipes](../extending.md).
