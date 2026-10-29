# Weeks 2–3: cubes, see-saws, and the match clock

**In 30 seconds:** the plugin owns rules, but the emulator owns time. We will allocate a bounded
pool of cubes, define one complete reset, and let the Driver Station choose the active period.

If you followed the field workshop's direct link here, the field source is now written. If you
skipped it, return to [body ownership and geometry](02a-field-implementation.md) before appending
the game state. These workshops build one translation unit, not interchangeable code examples.

## 1. Give each piece a stable identity

A cube has three meaningful states: active in physics, held by the robot, or in portal reserve.
Pickup disables an existing body; release re-enables it. Neither operation creates another cube.
An array of sixty IDs makes conservation easy to inspect and keeps allocation out of the hot loop.

The first sixteen IDs per alliance represent its six fence cubes and ten pyramid cubes.
The remaining fourteen represent off-field supply. With a preload, one of those fourteen becomes
held inventory. Reserve cubes must remain excluded from collisions and pickup.

Reset restores the whole partition, poses, velocities, plate heights, scores, inventory and counters.
Do it in one place. A constructor is not a reset function: the runtime can reset an existing match
without destroying its game state or relaunching the robot.

## 2. Ask physics what happened

The game-state step follows physics. Plate sensors return possible cubes; the referee filters
their tags, positions and speed before counting them. This avoids awarding ownership merely
because a thrown cube passed above the plate. The approximation counts settled centers; it does
not calculate actual balance torque.

Count both plates before moving either one. With more cubes, a plate descends; its partner rises.
With a tie, both return to level and neither alliance scores. The motion is a level vertical heave,
not a hinged see-saw. Collider and snapshot must use the same current pose.

## 3. Let the Driver Station choose the period

The Practice profile has editable durations; its defaults differ from Power UP's 15-second
autonomous and 135-second teleop. Read mode, enabled state, emergency stop and remaining time from
the simulation context. Disabled, emergency stop and zero time stop interaction/scoring, but
physics still lets cubes settle. A time of -1 means an untimed run.

Ownership's nominal rate is 2 points/s in autonomous and 1 point/s in teleop. This prototype
integrates `dt` while ownership holds. The official game's discrete acquisition/whole-second
award rules require a different ledger; fractional prototype scores are intentional.

**Student asks:** “Why did our 15-second autonomous last 20 seconds?” Because Practice is a
configuration, not the game manual. Change the host profile; do not hide another timer in the plugin.

## 4. Publish game data before autonomous starts

`IGameState::gameSpecificMessage()` returns the reset assignment. The runtime forwards it through
Driver Station state to HALSim before autonomous initialization. The three characters mean own
switch, scale, far switch in alliance-view order. The legal strings are `LLL`, `LRL`, `RLR`, `RRR`;
both alliances receive the same string, but view the field from opposite ends.

In the pinned 2027 alpha-7 API, C++ uses `wpi::MatchState::GetGameData()` and Java uses
`org.wpilib.driverstation.MatchState.getGameData()`. Both return an optional string. Validate the
entire message, not just its first character. These are not the pre-2027 DriverStation APIs.
The [drive-day workshop](05a-driven-robot.md) supplies both complete, three-PWM robot programs and
their matching saved models; a one-motor API probe would not demonstrate gameplay.

**You now have:** the state model and the rule/clock contract. Now write the state allocation and
reset methods, then continue to the tick and snapshot implementation.

Next: [Workshop—allocate cubes and plates, then reset them](03a-reset-and-pieces.md).
