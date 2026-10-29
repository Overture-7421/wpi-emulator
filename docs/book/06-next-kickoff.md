# Next kickoff: build the real game

**In 30 seconds:** keep the architecture and replace the rulebook. A new season is a new plugin when
the game changes; it is a new season pack when WPILib or the HAL ABI changes. Sometimes it is both,
but the two decisions are still separate.

## The reusable playbook

1. Read the season manifest and confirm the WPILib pin, Java major, ABI, and launch profile.
2. Build the unmodified robot sample for that profile before adding game code.
3. Copy the plugin template, choose the plugin ID `game_<year>`, and keep the SDK-only boundary.
4. Translate the official field drawing into an explicit field frame, collision geometry, visuals,
   and symmetry.
5. Decide which game pieces need dynamics, which interactions need sensors, and which rules can be
   represented as state transitions instead of detailed mechanics.
6. Put match state in `IGameState`, field ownership in `IField`, and season descriptions in
   `IGameModule`.
7. Expose robot-readable facts with game values and generic sensors; expose UI facts with config,
   HUD, and sound descriptors.
8. Test the rule boundaries, reset, determinism, teardown, and real contacts before adding polish.
9. Wire the plugin through the aggregators and let the existing app load it by project season.
10. Compile the team's robot against that year's actual WPILib artifacts. Never carry APIs forward
    by memory.

## What the Power UP example taught us

The emulator owns the clock, but the Driver Station owns the mode and duration. Physics bodies are
not rendering instances. Sensors report geometry, while the plugin decides the rule. A game message
is a robot-facing interface, not a HUD string. A season plugin can describe a Qt UI without depending
on Qt. Those separations are more reusable than any one year's cube count.

For the next game, revisit every approximation. A roller game may need continuous inventory flow;
a hanging game may need a new physics capability; a vision-driven game may need season-specific
AprilTags. Do not stretch the Power UP design until it snaps. The correct extension seam depends on
the new game's behavior.

## The last mentor's note

You are not trying to simulate everything FIRST built. You are trying to make the robot code respond
to a coherent, testable world. Choose the simplest model that preserves the decisions your robot
actually makes, label what it does not model, and leave the architecture cleaner than you found it.

That is a season plugin worth handing to the next build team.
