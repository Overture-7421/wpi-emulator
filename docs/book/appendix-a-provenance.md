# Appendix A. Rules, dimensions and provenance

## A.1 Source convention

[1] is the official manual; [2] groups the official layout and component
drawings. Manual references use section and table numbers. FE-00041 revision 1,
sheets 2-4, is the overall layout reference. The drawing package supplies
component detail. No coordinates are recovered by measuring a manual illustration.

Tags mean game manual `[GM]`, field drawing `[FD]`, calculated `[derived]`,
and selected model parameter `[approx]`. A nominal game dimension does not make
the entire collision model exact.

## A.2 Geometry and setup

| Quantity | Value used | Basis |
|---|---|---|
| Field length and width | 16.4592 by 8.2296 m | [GM 3.1-3.2], nominal 54 by 27 ft |
| Auto line | 3.048 m from each wall | [GM 3.2], 10 ft |
| Home-switch centre | 4.2672 m from wall | [GM 3.4], 14 ft; box layout approximate |
| Cube dimensions | 0.3302 by 0.3302 by 0.2794 m | [GM 3.8], 13 by 13 by 11 in |
| Cube mass | 1.5 kg | [approx], nominal manual mass approximately 3.25 lb |
| Cube density | About 49.24 kg/m³ | [derived], mass divided by box volume |
| Loose field cubes | 32 | [GM 4.1.2], six fence plus ten pile per alliance |
| Portal reserves before preload | 28 | [GM 3.7, 4.1.2], seven at each of four portals |
| Pyramid tiers | 6, 3, 1 | [GM 4.1.2, Figure 4-2]; centres and pitch nominal |
| Preload in this model | Zero or one | [GM G01]; transferred from one seven-cube reserve |
| Lower exchange opening | 0.5334 by 0.41275 m | [GM 3.6], 21 by 16.25 in |
| Switch ownership surface | At most 0.1524 m | [GM 4.2], 6 in |
| Scale ownership surface | At most 1.4224 m | [GM 4.2], 56 in |
| Platform top | 0.09 m | [approx] collision box |
| Brick threshold | Platform top plus 0.3048 m | [GM 3.3], 12 in above platform |

Exactly 60 identities exist. With one preload, the initial states are 32 field,
1 held and 27 reserve. One portal then retains six cubes and its partner seven,
both above the setup minimum of five. Multiple physical preloads are not exposed.

## A.3 Model parameters

| Parameter | Selection | Interpretation |
|---|---|---|
| Plate half-extents | 0.6096, 0.4572, 0.025 m | Approximate colliding box |
| Switch / scale heave | 0.09 / 0.12 m | Sufficient to cross ownership thresholds |
| Heave speed | 0.04 m/s | Prescribed actuator speed |
| Resting candidate speed | Below 0.4 m/s | Operational filter, not a manual threshold |
| Resting height tolerance | 0.05 m | Operational filter |
| Staging drift acceptance | Below 0.04 m over 5 s | Test tolerance, not a field measurement |
| Human action period | 2 s default, 0.1-60 s configurable | Scripted choice, teleoperation time only |
| Hook capture radius | 0.5 m | Approximate capture |
| Cable shortening / minimum length | 0.20 m/s / 0.10 m | Approximate winch |
| Default cable load limit | 4,000 N | Configurable actuator limit |
| Upward support threshold | Above 1 N | Operational support observation |
| Parking tolerance / speed | 0.03 m / 0.1 m/s | Operational parking proxy |

Portal, exchange sensing and upper-return endpoints are explicitly approximate.
The vault is a custody/count model, not a physically packed set of columns.

## A.4 Scoring and boundary decisions

The manual supplies 15 s autonomous, 135 s teleoperation, the last 30 s as
endgame, the point values in Table 4-1, and the power effects in Table 4-2.
The Driver Station remains authoritative for practice durations.

The implementation makes the following choices where the manual does not
specify a scoring-engine algorithm:

1. Acquisition requires one uninterrupted enabled second. Loss of the height
   condition releases ownership immediately; recapture requalifies.
2. Ownership and fractional points carry across autonomous to teleoperation.
   A phase change alone does not award establishment again.
3. Rates integrate in integer micro-points; publication floors the cumulative
   alliance total. Fractions are retained across ownership changes.
4. Force is an immediate effective-ownership source. It does not duplicate
   establishment for a target already effectively owned by the same alliance.
5. A power level is latched at acceptance, including queueing. Stable input
   order resolves simultaneous presses: Blue before Red, then Force, Boost,
   Levitate in the adapter's iteration order.
6. Scheduled presses are attempted once. An insufficient-cube rejection is
   logged and not retried. Match-end scoring ignores any unfinished power.
7. Endgame uses the latest enabled physical observation. The final zero-time
   input awards results once; later settling cannot alter that result.

These choices are tested, but are not attributed to FIRST. The manual's
rule semantics and the simulator's sampling and rounding policy remain distinct.

[Appendix B](appendix-b-reproduction.md)
