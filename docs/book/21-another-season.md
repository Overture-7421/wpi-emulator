# 21. A method for another season

## 21.1 Objectives

After this chapter the reader can separate reusable implementation patterns
from game-specific evidence and organize a new season into buildable checkpoints.
The completed case study is the prerequisite.

## 21.2 From a manual to a model

The reusable result is a method, not the POWER UP constants.
A new game starts with its manual, drawings and scoring definitions. Its
WPILib support is selected independently by the season pack.

1. Establish the coordinate frame, units and sources. Record dimensions as
   measured, derived or approximate before creating geometry.
2. Identify conserved identities and legal state transitions. A game with
   recyclable pieces needs a custody model even if its scoring is continuous.
3. Separate observations from rules. A sensor overlap is an observation;
   possession, a completed delivery or a referee award requires a definition.
4. Choose the physical detail needed for the intended experiment. A rule-only
   model cannot validate delivery, and a detailed collider is not automatically
   a validated mechanism.
5. Implement pure rules with explicit time and typed inputs. Test exact
   boundaries before connecting a world.
6. Add the physical adapters, body ownership and reset behavior. Test contact,
   conservation and teardown with the real backend.
7. Expose configuration and snapshots through existing SDK descriptions.
   Keep Qt, process control and persistence in their owning modules.
8. Integrate an ordinary robot process and test the complete output path.
   Keep a conformance table beside the measured results.

This order follows dependency: each stage supplies an oracle or an interface
needed by the next. It is not a requirement to finish every physical detail
before a walking skeleton can compile.

## 21.3 What carries over

The noncopyable body owner, fixed pool, Driver Station classifier and
attachment-aware zone calculation are useful patterns. A new plugin may use
different types, especially when it has multiple held pieces or mechanisms
that transfer pieces between each other. Extracting a common helper is justified
only when its contract is genuinely shared.

The existing 2026 plugin models many spherical pieces and continuous delivery.
POWER UP needs individually identified boxes, plate ownership and a finite
vault. Copying a sphere's collision model or scoring loop into the cube game
would preserve source structure while changing the intended experiment.

Assignment rules, endgame conditions, alliance achievements and official
geometry must be re-derived. Historical absence of AprilTags does not justify
an empty vision layout for a later game.

## 21.4 ABI and robot API

A later systemcore season can reuse the architectural boundary: the robot
calls its pinned WPILib APIs, the extension handles that HAL ABI, and the game
uses the SDK. It cannot assume that preview class names remain unchanged.

The 2027 alpha-7 programs in Chapter 13 are tested examples for that pin.
Updating a season's WPILib version calls for rebuilding its probes and real-HAL
tests, not adding year branches inside a game state or the host.
The [season-pack reference](../reference/seasons.md) owns that procedure.

## 21.5 Summary

A season implementation is a set of independently justified choices about
rules, physics and integration. Checkpoints make those choices inspectable.
Conformance and provenance remain necessary after the first successful match.

## 21.6 Exercises

1. A hypothetical game awards points when a disk remains inside a basket for
   0.5 s. Identify the physical observation, rule state and conservation state.
2. Explain which tests must change if that disk becomes a deformable object.
3. Classify a WPILib preview API rename separately from a new scoring rule.

[Chapter 22](22-limitations.md)
