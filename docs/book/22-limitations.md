# 22. Limitations and interpretation

## 22.1 Objectives

After this chapter the reader can state what the completed experiment establishes
and identify the additional evidence needed for a stronger physical or rules claim.

## 22.2 Physical fidelity

F-01 and F-03 describe nominal box geometry, rigid cubes and approximate
materials. No experiment here measures real cube compression, carpet friction,
fabric interaction or manufacturing variation. Five-second staging stability
establishes a useful initial condition, not agreement with a real-field drop test.

F-07 uses horizontal plates with prescribed vertical motion. It preserves a
visible and colliding surface but does not reproduce lever arms, plate angle,
load-dependent inertia or a hinged scale. A mechanically validated replacement
would need an appropriate joint interface and comparison data.

F-08 samples candidate overlap, height and speed. A fast piece can cross a
sensor between ticks. F-10 and F-18 use the chassis box as bumper geometry.
Parking is a pose-and-speed proxy; climbing additionally requires cable load,
but does not implement a transitive multi-robot support graph.

## 22.3 Rules and agents

F-09 identifies the fractional accounting policy rather than claiming exact
equivalence to FIRST's scoring engine. F-11 through F-16 script human choices
and button presses. The portal and return operations relocate conserved bodies
to approximate endpoints; no human or chute is physically simulated.

The pool's removed state reserves a category for later out-of-field handling.
The implementation does not assign that state or automatically recover escaped
cubes. Such a cube remains a field identity until an implemented transfer or reset.
It is not replenished from an unlimited supply.

F-19 supplies virtual referee outcomes, not five additional simulated robots.
The opponent cube is physical after placement, but its transport to the scale
is scripted. F-21 omits penalties and forced-climb awards caused by illegal
opponent contact. F-24 does not inspect robot legality.

These choices make a repeatable teaching and practice environment. They do
not make an official referee or a six-robot competition simulation.

## 22.4 Presentation and external validity

F-22 omits retroreflective targets. F-25 follows the SDK entity-group format,
which carries positions but not orientations. Cubes collide as rotating boxes
while their generic run-view visuals remain axis-aligned.

The measured tick cost belongs to one machine, Release configuration and
fixture history. Hash comparisons cover the tested backend and worker counts;
they do not establish bitwise equality across compilers or CPU architectures.
Real-HAL tests cover both published robot languages, but only the controlled
programs and mappings in the book.

The evaluation table in Appendix C distinguishes completed checks from
unavailable environments. A build or offscreen test is never substituted for
an actual rendered-scene inspection.

## 22.5 Further work

The conformance table gives a concrete order for extending fidelity: measure
materials, refine plate mechanics and bumper geometry, add explicit out-of-field
logistics, then model multi-robot support and contact adjudication. Each
extension needs its own observation and acceptance test. More detailed code
without those tests would not strengthen the result.

## 22.6 Summary

The complete listings produce an integrated game model with known boundaries.
Their value is reproducible behavior and an explained route from specification
to implementation. Appendix C is the record of what that behavior does and
does not establish.

## 22.7 Exercises

1. Propose a real-field experiment that could calibrate one material parameter.
2. Explain why adding five chassis pictures would not implement F-19 physically.
3. Choose one omitted feature and state a test that would justify changing
   its conformance classification.

[Appendix A](appendix-a-provenance.md)
