# Preface

An FRC robot program describes decisions and actuator commands. A simulator also
needs an account of what those commands do: how the chassis moves, how a cube
falls, when a plate changes ownership, and when that ownership earns points.
This book develops those accounts separately, then connects them through the
emulator's game-plugin interface.

The reader is assumed to understand ordinary C++ classes, references, containers
and ownership through `std::unique_ptr`. Experience with `TimedRobot`, motor
controllers and the Driver Station supplies the robot-side background. CMake,
rigid-body simulation, Qt and HAL simulation are introduced where they first
affect a design decision. The [repository glossary](../GLOSSARY.md) supplies
definitions of the surrounding tools.

## Reading and reproduction

Parts I and II explain the architecture and construct the physical core. Part III
adds the remaining match rules and presentation. Each implementation chapter
states its prerequisites, gives complete files, and identifies tests that can
reject an incorrect implementation. Appendix B collects the reproduction
commands; Appendix C distinguishes implemented rules from modeling assumptions.

The plugin is not installed by reading this book. Its source is held in the
listings, and the worked integration modifies a disposable copy of the repository.
The normal source tree remains a platform-only 2027 season. A developer who
adopts the example performs the same documented integration in a branch.
No custom robot library is required.

## Notation

Lengths are metres, times in rule state machines are integer microseconds, and
angles are radians unless a unit is stated. The field frame is right-handed,
with positive Z upward. A quaternion is written in `(x,y,z,w)` order. A *tick*
is one complete emulator update; a *physics step* is only its rigid-body portion.

The tags `[GM]`, `[FD]`, `[derived]` and `[approx]` identify the origin of a
constant: game manual, field drawing, calculation, or modeling choice.
Bracketed numbers such as [1] refer to the
[bibliography](appendix-f-references.md). Numbered definitions and invariants
name concepts that later chapters use; they do not imply that physical fidelity
has been proved by a unit test.

## Attribution

FIRST POWER UP is used as a case study. This book is an independent work and is
not affiliated with or endorsed by FIRST. The official manual and drawings
remain the authoritative game specification [1, 2]. The field model and diagrams
in this book are explanatory models, not manufacturing drawings.

[Chapter 1](01-introduction.md)
