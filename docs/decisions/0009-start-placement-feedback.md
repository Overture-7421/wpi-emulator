# Explicit start placement and calibrated feedback

Read this when changing chassis placement or robot-visible sensor state.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
A robot's odometry reset changes its estimate, not the physical chassis location. Likewise, feeding
a model-calibrated value as raw sensor state can cause robot firmware to apply the same calibration
twice.

## Decision
Keep physical placement separate from robot pose estimation, and convert feedback at the mapping or
bridge boundary so each reversal and calibration is applied exactly once.

## Trade-offs
Correct feedback depends on configured ratios, mappings and available calibration read-back.
When calibration is unavailable, guessing would be less trustworthy than reporting the limitation.

Current placement and feedback contracts are in [Runtime](../reference/runtime.md) and
[Bridges](../reference/bridges.md).
