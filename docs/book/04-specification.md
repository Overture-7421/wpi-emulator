# 4. The game as a specification

## 4.1 Objectives

After this chapter the reader can translate a manual condition into a state
transition, distinguish physical observations from referee decisions, and
state a conservation invariant for cubes.

## 4.2 Field coordinates

The field origin is at one alliance wall, with X increasing toward the other
wall, Y across the field and Z upward. The case study names the origin end
Blue. Naming the end does not change the official field's symmetry.
The opposing alliance's view is related by
$(x,y,\psi)\mapsto(L-x,W-y,\psi+\pi)$.

This is a rotation through the field centre, not a reflection across its
length. Rotating a layout preserves handedness. A release direction attached
to a chassis must rotate with it as well as translate.

The assignment is a three-character string: near switch, scale, far switch,
with L and R read from the alliance's own end. The legal strings are LLL, LRL,
RLR and RRR [1, section 4.1]. The first and third letters agree because the
two switches have the same alliance-relative arrangement. A world-space
positive-Y plate need not belong to the same alliance at every element.

## 4.3 State and transitions

**Definition 4.1 (cube location).** Each cube is in exactly one of the following
states: field, held, reserve, exchange, vault or removed. Field cubes have
enabled physics bodies. The other states retain identity without a live collider.

**Invariant 4.1 (conservation).** The counts of all cube locations sum to 60
after every transition and reset. Pickup and release change location; neither
allocates a new cube.

Definition 4.1 permits independent tests of custody and rigid-body motion.
For example, exchange receipt changes a cube from field to exchange. A later
human-player decision can put that same cube into a vault column. Awarding
five points without changing custody would permit the same cube to score again.

## 4.4 Observations and rules

The manual defines ownership using plate position and a one-second dwell,
with Force as an alternative source of ownership [1, section 4.2]. The
physical model first determines which cubes rest on each plate. The plate
model then changes its height. The scoring model applies dwell and point
accounting. These are three separate decisions, each with a separate test.

Table 4.1 summarizes the main rewards. It is a reading aid; the conditions
and interpretations used by the implementation are in Appendix A.

Table 4.1. Reward categories [1, Table 4-1].

| Action | Match points | Qualification ranking points |
|---|---|---|
| Auto-run | 5 per robot | Part of Auto Quest |
| Ownership | 2 + 2/s in auto; 1 + 1/s in teleop | Own switch at auto end contributes to Auto Quest |
| Vault cube | 5 | None |
| Park / climb | 5 / 30 | Three credited climbs earn Face the Boss |
| Match result | None | Win 2, tie 1 |

## 4.5 Choosing the model

A rule-only model could accept cube placements as inputs. It would test scoring
but not a robot's delivery. A complete mechanical model could simulate hinged
plates, deformable cubes and six robots. It would require capabilities and
validation beyond this case study.

The selected model uses real cube contacts and a kinematic plate rule. This
supports testing a physical delivery from an unmodified robot while keeping
the game rules inspectable. Approximate geometry and scripted agents retain
their explicit classifications in Appendix C rather than being presented as
physical measurements.

## 4.6 Summary

A manual rule becomes a combination of observations, transitions and rewards.
The coordinate convention and cube invariant constrain all later components.
Keeping those constraints explicit gives tests an oracle independent of the
implementation's control flow.

## 4.7 Exercises

1. Derive which world-space side belongs to Blue for each element under RLR.
2. Give the custody transitions for an intake, release, exchange and vault deposit.
3. Explain why counting cubes directly as points would contradict ownership scoring.

[Chapter 5](05-registration.md)
