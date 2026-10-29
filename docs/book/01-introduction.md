# 1. Introduction and scope

## 1.1 Objectives

After this chapter the reader can distinguish a game plugin from WPILib support,
identify the state a plugin owns, and judge what evidence is needed before
calling an example usable.

## 1.2 Two independent problems

A Java or C++ robot process calls WPILib. Its Hardware Abstraction Layer, or HAL,
normally communicates with robot hardware. In desktop simulation the HAL exposes
simulated device state. The robot process loads the emulator's HALSim extension,
which exchanges simulated device frames with the desktop over local IPC. The
robot still uses normal WPILib APIs; it does not call the game plugin [3].

The desktop process owns the physical world and game rules. A season pack chooses
the WPILib version and HAL ABI. A game module chooses field geometry, game pieces
and scoring. The case study assigns the 2018 game to the 2027 season pack so that
the robot projects exercise systemcore, the HAL ABI profile used by the pinned
WPILib preview. This is a deliberate teaching configuration, not a claim about
the official 2027 game.

## 1.3 Requirements

Table 1.1 states the requirements used throughout the book. Each has a
verification entry in [Appendix C](appendix-c-conformance.md).

Table 1.1. Requirements of the case study.

| ID | Requirement |
|---|---|
| R1 | Manifest, registry and plugin identity agree. |
| R2 | The plugin depends only on the C++ standard library and SDK. |
| R3 | Field visuals and colliders use the same poses. |
| R4 | Every one of the 60 cubes has exactly one location state. |
| R5 | The initial layout follows the manual and remains physically stable. |
| R6 | Resting cubes determine plate motion and ownership. |
| R7 | Driver Station state controls scoring windows. |
| R8 | The field assignment reaches robot code before autonomous. |
| R9 | Robot interaction uses ordinary outputs and declared zones. |
| R10 | Reset restores state; repeated input histories are repeatable. |
| R11 | Published snapshots and descriptors agree. |
| R12 | Implemented, approximate, scripted and omitted rules are identified. |
| R13 | The listings reproduce a compiling and tested plugin. |

R4 concerns conservation of identity, not conservation of the number of enabled
bodies. A held cube remains allocated but its collider is disabled. A cube in
the vault remains accounted for even though it no longer participates in
collision detection. This distinction prevents a common error: creating a new
cube on every release without consuming a held one.

## 1.4 Model boundary

The physical model covers one robot, cubes, field structures and movable plates.
The plates remain horizontal while their heights change. Human-player decisions,
virtual partners and opponent outcomes are deterministic scenario inputs.
Penalty adjudication, robot inspection, retroreflective vision and cube
orientation in the run view are outside the model. The exact classifications
and tests are recorded as F-01 through F-25 in Appendix C.

These choices separate a usable practice environment from an official referee
or a validated reconstruction of a competition field. In particular, a correct
score state machine does not establish that cube friction or plate motion
matches a real field.

## 1.5 Related approaches

WPILib simulation provides robot-side simulation hooks and supports replacing
hardware observations with simulated values [3]. The maple-sim project combines robot
simulation with a two-dimensional rigid-body world for Java projects [4].
The emulator used here instead places a three-dimensional world in a separate
desktop process and communicates through local IPC. These approaches answer
different integration questions; none makes a game model correct without
independent rule and physics tests.

## 1.6 Summary

The unit of implementation is a game plugin. HAL compatibility remains the
responsibility of the season pack and per-ABI code. The requirements separate
integration, game behavior and evidence so that each can be tested independently.

## 1.7 Exercises

1. Classify a new CAN motor controller, a new vault rule and a new carpet collider
   by their owning module.
2. Explain why a successful plugin build cannot establish R5.
3. Give an example of a test that would reject a violation of R4.

[Chapter 2](02-discrete-time.md)
