# The Season Builder's Book

## FIRST POWER UP, played on the 2027 ABI

Your robot code thinks it is driving on a real field. It is wrong, and it never needs to know.
WPILib runs in the robot process; the HALSim extension talks to it as if real devices were attached.
The desktop app owns the world. A game plugin supplies the field, game pieces, rules, and the
referee's decisions.

This book walks you through building that plugin. You are an experienced FRC C++ mentor, so you
already know why a robot needs a drivetrain and why a cube should not pass through a wall. You do
not need to know this repository, Qt, or WPILib's internals yet. We'll introduce each when the work
needs it.

The worked game is **FIRST POWER UP (2018)**. The sample season is **2027**, whose current preview
pack uses the `systemcore` ABI and WPILib `2027.0.0-alpha-7`. That makes Power UP a teaching stand-in,
not a claim about the announced 2027 game. The season pack is currently core-only; these instructions
show the optional game plugin seam. Recheck the real game's manual and the selected season pin before
reusing any dimensions or rules.

The chapters follow a build season:

| Week | Your milestone |
|---|---|
| Kickoff | Learn who owns the robot, the world, and the referee; make a clean 2027 build |
| 1 · Pour the carpet | Create and register the plugin; set field coordinates and static geometry |
| 2 · Sixty yellow crates | Add cubes, safe resets, collision rules, and sensor volumes |
| 3 · The see-saws | Model switch and scale ownership; connect the match clock and game data |
| 4 · Keeping score | Add scoring, vault power-ups, ranking points, and HUD values |
| 5 · Robot meets field | Add robot interactions, configuration, editor descriptions, sounds, and a climber |
| 6 · Bag and tag | Build Java and C++ robots; run and verify a match |
| Next kickoff | Replace the example rules with the next game's rules without teaching core a new year |

Read [Kickoff](01-kickoff.md) first, then follow the chapter links at the end of each page. The
[Power UP fact sheet](appendix-power-up.md) is the source of the example's numbers; the
[file map](appendix-file-map.md), [glossary](appendix-glossary.md), and [pit
checklist](appendix-checklist.md) are for implementation and final inspection.

> **A note from the drive coach:** This book is about a playable simulation, not a referee-approved
> replica. It calls out approximations where the SDK or a single-robot simulator cannot reproduce
> the real hardware exactly. A limitation written down is a design constraint; a limitation hidden
> in code is a match-day surprise.

## Two processes, one robot

The robot project stays a normal WPILib project. The HALSim extension is loaded into that process
and exchanges sensor and actuator data with the desktop app. The game plugin lives in the app-side
simulation and depends only on the emulator SDK.

```text
Robot process                           Desktop app
WPILib robot code                       Qt shell and run view
       |                                         |
HAL simulation <- HALSim extension <-> local IPC <-> SessionRunner
                                                   |
                                       mechanisms -> physics
                                                   |
                                         game plugin -> referee
```

The separation is useful: a game plugin must not include Qt, Box3D, HAL, or vendor libraries. It
describes Qt-free metadata; the app renders that metadata. It calls the SDK physics interface; core
chooses the physics engine. WPILib details stay in the robot process and the ABI-specific bridge.

## What “verified” means here

Commands and robot samples in this book were checked in a clean source export using MSVC 2026,
warnings-as-errors, and the pinned WPILib 2027 alpha-7 Java and C++ toolchains. The game-plugin
template and season-extension tests passed. These checks verify the repository seams and the robot
sample builds; they do **not** make an approximate Power UP field an official field drawing.

The repository changes for this book are Markdown only. No plugin, robot code, generated build
output, or commit is part of the change.

Next: [Kickoff—meet the machine and set up your pit](01-kickoff.md).
