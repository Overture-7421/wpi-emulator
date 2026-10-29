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

**Scope:** the runnable outcome is a physical, single-robot prototype: correct starting cube groups,
six moving plates, pickup/release, FMS data, ownership scoring and reset. It is **not a complete
Power UP implementation**. Human feeds, vaults, power-ups, parking/climbing, ranking points,
penalties and opponents remain unimplemented. Ownership scoring uses continuous rates rather than
the manual's discrete award rules.

The chapters follow a build season. Read each design introduction, then its numbered workshop;
the workshops contain the actual implementation, in explained increments:

| Week | Your milestone |
|---|---|
| Kickoff | Learn who owns the robot, the world, and the referee; make a clean 2027 build |
| 1 · Pour the carpet | [Design](02-field-and-physics.md), then [own bodies and build the field](02a-field-implementation.md) |
| 2–3 · Cubes and see-saws | [Rules and clock](03-rules-and-robot.md), then [allocate and reset](03a-reset-and-pieces.md) |
| 4 · Keeping score | [Scope and extensions](04-scoring-and-features.md), then [ticks and snapshots](04a-match-and-snapshots.md) |
| 5 · Robot meets field | [Pickup/release](04b-zone-interactions.md), then [factories, metadata and registration](04c-module-and-registration.md) |
| 6 · Bag and tag | [Inspection plan](05-test-and-run.md), [mapped robots](05a-driven-robot.md), [physics tests](05b-physics-tests.md), [full matches](05c-full-match.md) |
| Next kickoff | Replace the example rules with the next game's rules without teaching core a new year |

Read [Kickoff](01-kickoff.md) first, then follow the chapter links at the end of each page. The
[Power UP fact sheet](appendix-power-up.md) is the source of the example's numbers; the
[file map](appendix-file-map.md), [glossary](appendix-glossary.md), and [pit
checklist](appendix-checklist.md) are for implementation and final inspection.

Each marked source block names its destination. Repeated blocks for the same file append in
reading order; they are explained fragments of a complete file, not individually compilable
programs. There are no omitted helpers or private downloads. [Replay the book](appendix-replay.md)
provides an optional assembler and a clean-export procedure. Your first complete plugin build
comes at the end of the registration workshop.

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

The [verification record](appendix-replay.md#verification-record) names the checks actually run
from the published listings. Compilation, real-physics tests, robot-process matches and visual
inspection prove different things. None makes nominal field geometry an official drawing or
turns the unimplemented features above into working rules.

The repository changes for this book are Markdown only. No plugin, robot code, generated build
output is added outside the Markdown listings. A reader opts in by assembling the listings in a
scratch export; the shipped 2027 pack stays core-only.

Next: [Kickoff—meet the machine and set up your pit](01-kickoff.md).
