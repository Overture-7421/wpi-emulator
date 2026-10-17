# Generic mechanisms, plants and motor models

Owner: `core/mechanisms/` (target `wpiemu_core`, namespace `wpiemu::core::mechanisms`). Game-agnostic:
nothing here knows a season. Decisions and rationale: [ADR 0009](decisions/0009-mechanisms-impl.md).

```cpp
#include "wpiemu/core/mechanisms/mechanism_templates.h"   // registerGenericMechanisms
#include "wpiemu/core/mechanisms/plants.h"                // registerBuiltinPlants, validatePlantSpec
#include "wpiemu/core/mechanisms/motor_catalog.h"         // MotorCatalog, CurrentLimitedMotorModel
#include "wpiemu/core/mechanisms/linear_system.h"         // LinearSystemId equivalents, expm, discretizeAB
#include "wpiemu/core/mechanisms/motor_drive.h"           // MotorOutput -> motor voltage
#include "wpiemu/core/mechanisms/sensor_models.h"         // encoder / rotor / limit / beam-break models

// composition root
sdk::MechanismFactory mechanisms;
registerGenericMechanisms(mechanisms);   // optional: MechanismTemplateServices{catalog, plantFactory}
```

## 1. Motor catalog (`MotorCatalog`)

Same data as WPILib `DCMotor` factories, **one motor** per row (`DCMotor(nominalV, stallTorque, stallCurrent,
freeCurrent, freeSpeed)`; `R = V/Istall`, `Kv = wfree/(V - R*Ifree)`, `Kt = Tstall/Istall`, identical to the
SDK `MotorSpec` derivations). Source: allwpilib 2026 `wpimath/.../system/plant/DCMotor.java`
(<https://github.com/wpilibsuite/allwpilib>, also in the WPILib 2026 install `documentation/java/src-html`).
`motorModel` property values = keys below. Values are checked row by row in `tests/mechanisms`.

| key | WPILib factory | Tstall N·m | Istall A | Ifree A | free rpm |
|---|---|---|---|---|---|
| `kraken_x60` | `getKrakenX60` | 7.09 | 366 | 2 | 6000 |
| `kraken_x60_foc` | `getKrakenX60Foc` | 9.37 | 483 | 2 | 5800 |
| `kraken_x44` | `getKrakenX44` | 4.11 | 279 | 2 | 7758 |
| `kraken_x44_foc` | `getKrakenX44Foc` | 5.01 | 329 | 2 | 7368 |
| `falcon_500` | `getFalcon500` | 4.69 | 257 | 1.5 | 6380 |
| `falcon_500_foc` | `getFalcon500Foc` | 5.84 | 304 | 1.5 | 6080 |
| `minion` | `getMinion` | 3.17 | 211 | 2 | 7704 |
| `neo` | `getNEO` | 2.6 | 105 | 1.8 | 5676 |
| `neo_550` | `getNeo550` | 0.97 | 100 | 1.4 | 11000 |
| `neo_vortex` | `getNeoVortex` | 3.60 | 211 | 3.6 | 6784 |
| `cim` | `getCIM` | 2.42 | 133 | 2.7 | 5310 |
| `mini_cim` | `getMiniCIM` | 1.41 | 89 | 3 | 5840 |
| `bag` | `getBag` | 0.43 | 53 | 1.8 | 13180 |
| `vex_775pro` | `getVex775Pro` | 0.71 | 134 | 0.7 | 18730 |
| `andymark_rs775_125` | `getAndymarkRs775_125` | 0.28 | 18 | 1.6 | 5800 |
| `banebots_rs775` | `getBanebotsRs775` | 0.72 | 97 | 2.7 | 13050 |
| `andymark_9015` | `getAndymark9015` | 0.36 | 71 | 3.7 | 14270 |
| `banebots_rs550` | `getBanebotsRs550` | 0.38 | 84 | 0.4 | 19000 |
| `romi_built_in` (4.5 V) | `getRomiBuiltIn` | 0.1765 | 1.25 | 0.13 | 150 |

`MotorCatalog::createModel(key)` gives an `sdk::IMotorModel` (`DcMotorModel`). `CurrentLimitedMotorModel`
decorates any `IMotorModel` with a stator-current limit (`I = clamp((V - w/Kv)/R, ±limit)`, `tau = Kt I`) and
`limitedVoltage()` (the voltage that produces exactly the limited current).
Catalogs are plain values (no global); extend with `add({key, label, spec})`.
> Note: `sdk::motors::cim()` in the frozen SDK lists 5330 rpm; WPILib 2026 says 5310. The catalog follows WPILib.

## 2. Plants (`IPlant`, `PlantFactory`)

Generic templates also expose opt-in motion axes (`axis`, or intake `deploy`/`roller`).
With a configured motion link, a follower uses ratio-scaled source motion instead of advancing
its plant/motors. Starting positions, travel limits, encoders, published geometry, and
game-facing values still apply; follower motor current is zero. A follower may not have an
independent motor mapping. Links never feed load back to the source. See
[ADR 0012](decisions/0012-motion-links.md); geometry attachment links are a separate feature.

`registerBuiltinPlants(PlantFactory&)` registers `PlantSpec::model` keys:

| key | kind | WPILib equivalent |
|---|---|---|
| `dc_motor` | `spec.kind` (angular: J, linear: mass + drum radius) | `LinearSystemId.createDCMotorSystem` / `DCMotorSim` |
| `turret` | angular, hard stops optional (`min == max` = continuous) | `DCMotorSim` |
| `flywheel` | angular, stops ignored, position integrates (for encoders) | `FlywheelSim` |
| `arm` | angular + gravity `m g L_cm cos(theta - offset) / J`, hard stops | `SingleJointedArmSim` |
| `elevator` | linear + constant gravity acceleration `-9.8`, hard stops | `ElevatorSim` |

State `x = [position, velocity]` of the mechanism **output** (rad, rad/s or m, m/s). `N = motorCount` motors
share the gearbox exactly like WPILib's `DCMotor` aggregation. Equations (`linear_system.h`, from
`LinearSystemId.java`):

```
elevator   A=[[0,1],[0,-G²Kt/(R r² m Kv)]]  B=[0, G Kt/(R r m)]
flywheel   A=[-G²Kt/(Kv R J)]               B=[G Kt/(R J)]
dc motor / single-jointed arm: A=[[0,1],[0,-G²Kt/(Kv R J)]]  B=[0, G Kt/(R J)]   (Kt, R, Kv of the gearbox)
```

Extensions on top of WPILib: `efficiency` (motoring ×η, back-driving ×1/η), viscous friction `b` (adds `b/M` to
the decay rate), Coulomb friction (explicit stiction: holds while the drive cannot overcome it, then slides),
external load (`PlantInput::externalLoad`, N or N·m), hard stops that clamp the position and zero the velocity
(`atLowerLimit`/`atUpperLimit`), `motorCurrentA()` = signed total current at the start of the step
(`N (V - Γ v / Kv)/R`, Γ = shaft rad per output unit), `motorShaftOmegaRadPerSec()`.

Integration: linear systems (`efficiency == 1`, no angle-dependent gravity) use the **exact matrix-exponential
discretization** (`discretizeAB` on the augmented `[A B_V B_d]`, cached per `dt`), so they are exact for any
step. Nonlinear systems use **RK4 sub-stepped** so `|A| h <= 0.1`, without a work cap that could violate
that bound. For stiff nonlinear plants (`|A| dt > 10`, including worst-case efficiency), a semi-analytic
exponential midpoint step integrates motor/viscous damping exactly and samples gravity at a predicted midpoint. Its substeps are
at most 100 microseconds (also limited by angular speed), controlling gravity accuracy independently
of electrical stiffness. Non-unity efficiency uses exact velocity events at the motoring/back-driving
boundaries (zero velocity and motor no-load velocity). If the two quadrant accelerations point toward
zero velocity, it remains at rest until the frozen load changes, instead of numerically chattering.
WPILib integrates with `rkdp`; analytic and fine-reference regressions cover the existing accuracy
and stiff arms across 1/2/4/5/10/20 ms ticks. Arm/elevator gravity uses WPILib's 9.8 m/s²
(`kPlantGravityMps2`), not the physics world's gravity.

`validatePlantSpec(spec)` reports the first bad field (the factory callback itself can only return null →
`Result` error "factory returned null").

## 3. Motor command → voltage (`MotorDrive`)

Per motor and tick, from `MotorOutput`: no data → neutral; torque-current request → `V = I R + w/Kv`; else
`voltageV` (or `dutyCycle × battery` when only duty is set). Then saturate at `±hub.batteryVoltage()` (battery
sag), then the stator current limit. **Neutral** (zero command/unmapped): *brake* = 0 V (shorted, back-EMF
braking), *coast* = `V = w/Kv` (no current). Battery draw per motor = `max(0, V·I)/Vbattery`
(`IMechanism::batteryCurrentA()`).

## 4. Generic templates (`registerGenericMechanisms`)

All templates share the **axis** model below, add template-specific properties, publish `MechanismState` and
write sensors through slots. Position/limits are SI: radians (rotary/arm) or metres (linear).

### Axis slots (prefix `deploy`/`roller` in the intake, none elsewhere)

| slot | type | required | meaning |
|---|---|---|---|
| `motor` | Motor | yes (intake `deployMotor`: no) | primary motor controller; receives rotor position/velocity/acceleration (rotor side, i.e. **before** the device's sensor-to-mechanism ratio: `output rot × gearRatio`) |
| `motor2`..`motor4` | Motor | no | extra motors; the plant sees the **mean** voltage over `motorCount` motors, unmapped ones mirror the primary (intake: `motor2` only) |
| `encoder` | PositionSensor | no | output-side position/velocity in rotations × `sensorRatio` (± inversion, + `sensorOffsetRot`); linear axes report **drum** rotations |
| `velocitySensor` | VelocitySensor | no | same data on a velocity-only slot |
| `reverseLimit` / `forwardLimit` | LimitSwitch | no | closed at the min / max stop (`limitMargin`, polarity `limitPressedValue`) |

### Axis properties (key; prefix applies in the intake)

| group | key | default | notes |
|---|---|---|---|
| Motor | `motorModel` | per template | catalog key |
| | `motorCount` | 1 | 1..8 |
| | `gearRatio` | per template | motor : output |
| | `efficiency` | 1 | 0.05..1 |
| | `motorInverted` | false | positive command moves the mechanism negative; rotor feedback keeps the controller's sign |
| | `neutralMode` | coast / brake (arms, turret, elevator) | |
| | `statorCurrentLimitA` | 0 | per motor, 0 = off |
| | `reportMotorLimits` | false | also set `MotorFeedback::forwardLimit/reverseLimit` |
| Dynamics | `motion` | (generic/intake deploy only) rotary \| arm \| linear | |
| | `momentOfInertiaKgM2` | per template | rotary J; arm: 0 = uniform rod `m L²/3` |
| | `massKg`, `lengthM`, `comFraction` | | arm mass, pivot→tip length, CoM fraction (WPILib: 0.5) |
| | `massKg`, `drumRadiusM` | | linear carriage (+payload) mass, drum radius |
| | `simulateGravity` | arm/linear templates: true | arm 0 rad = horizontal |
| | `viscousFrictionNmS`, `coulombFrictionNm` | rollers 0.001 / 0 | output side |
| Limits | `minPosition`, `maxPosition`, `startPosition` | | equal min/max = no stops; start must lie inside |
| | `limitMargin`, `limitPressedValue` | 1e-6, true | |
| Sensors | `sensorRatio`, `sensorInverted`, `sensorOffsetRot` | 1, false, 0 | |

### Template list

| type | display / category | axes | extra properties | links | published `values` (`active`) |
|---|---|---|---|---|---|
| `roller` | Generic roller / Generic | 1 rotary (flywheel plant) | `radiusM` 0.03, `spinAxis` y, `activeSurfaceSpeedMps` 0.5 | `roller` | position, velocity, **surfaceSpeed**, voltage, current (`active`: \|surface speed\| ≥ threshold) |
| `generic_1dof` | Generic 1-DOF / Generic | rotary \| arm \| linear, stops, limit switches | `axis` z, `beamBreakSource` external\|window, `beamBreakMin/Max`, `beamBreakBlockedValue`; slot `beamBreak` | `mover` | position, velocity, voltage, current, atLowerLimit, atUpperLimit (`active`: moving > 0.01) |
| `intake` | Intake / Intake | `deploy…` (arm or linear, stops, limit switches) + `roller…` | `deployEnabled`, `deployAxis` y, `deployedAtMax`, `deployedTolerance` 0.05, `rollerOffset` (0.3,0,0), `rollerRadiusM`, `rollerSpinAxis`, `activeSurfaceSpeedMps` | `deploy`, `roller` (child of deploy) | deployPosition, deployVelocity, deployed, rollerVelocity, **rollerSurfaceSpeed**, voltages, currents (`active`: roller above threshold **and** deployed) |
| `shooter` | Shooter / flywheel / Shooter | 1 rotary flywheel | as roller (radius 0.05, threshold 1 m/s) | `flywheel` | as roller; **surfaceSpeed** = ω·radius for exit-speed models |
| `pivot` | Hood / pivot / Arm | 1 arm (gravity), stops [0, 1.5], limit switches | | `arm` (angle from horizontal, + raises +X: rotation about Y by −θ) | as generic |
| `turret` | Turret / Turret | 1 rotary, stops ±3.3 rad, limit switches | | `turret` (yaw about Z) | as generic |
| `elevator` | Elevator / climber / Elevator | 1 linear (gravity), stops [0, 0.8], limit switches | `axis` z | `carriage` (translation) | as generic |
| `indexer` | Indexer / conveyor / Indexer | 1 rotary belt | as roller + slots `entryBeamBreak`, `exitBeamBreak` | `belt` | as roller |

Intake deploy rotation is about `deployAxis` by **+θ without sign flip** (about +Y positive lowers a forward
intake; so positive = deploying). `rollerOffset` is in the deploy frame. With deploy gravity enabled,
the arm COM is assumed along local +X: gravity increases +Y deploy angle near horizontal, while
rotation about X or Z has no gravitational torque. Linear deploy gravity acts only on a Z axis
(toward negative position); horizontal X/Y slides have no gravity load. This is a local-frame
approximation, not mount/chassis-tilt-dependent gravity.

### Sensors and the game-state hook

* Encoders, rotor feedback and limit switches are ideal (noise-free) and written **every tick** (absolute truth).
* Limit switches derive from the stops. `generic_1dof` can also drive its `beamBreak` slot from a position
  window (`beamBreakSource = window`).
* **Beam breaks driven by game logic**: with source `external` (default) and for the indexer's
  `entryBeamBreak`/`exitBeamBreak`, the template **never writes** the slot. A game plugin writes it through
  `ctx.devices().writeDigital({mechanismId, "entryBeamBreak"}, blocked)`; nothing else touches it.
* Game plugins read `IMechanism::state()` (`active`, named values such as `rollerSurfaceSpeed`/`surfaceSpeed`,
  link poses) through `IMechanismDirectory`; they never need a core header.

### Errors

`MechanismFactory::create` returns an error (and the reason goes to `ctx.sim.logger()` category `mechanisms`)
for: unknown `motorModel`, out-of-range / mistyped properties, `maxPosition < minPosition`, `startPosition`
outside the stops, non-physical plant parameters, bad `axis`/`spinAxis`. Unknown keys only warn.
