# 2. The emulator as a discrete-time system

## 2.1 Objectives

After this chapter the reader can trace one motor command through a tick,
distinguish simulated time from elapsed desktop time, and explain why reset
requires an epoch identifier.

## 2.2 Time and state

**Definition 2.1 (tick).** A tick advances the emulator from state $S_k$ to
$S_{k+1}$ using a fixed interval $\Delta t$ and the currently accepted robot
outputs. For the default interval of 5 ms, 200 ticks represent one simulated
second. A slow desktop may take more than one real second to compute them.
Changing wall-clock pacing does not change the interval used by physics.

The state transition has the form $S_{k+1}=F(S_k,u_k,\Delta t)$, where $u_k$
contains actuator outputs and Driver Station inputs. The game rules belong
inside this transition. A desktop timer, an independent random generator or a
callback on the UI thread would introduce another input that the recorded
history does not contain.

## 2.3 The update order

Figure 2.1 follows the repository's [tick contract](../ARCHITECTURE.md#one-simulation-tick).
The game state observes the result of physics, rather than predicting it from
motor commands. A release late in tick k therefore begins moving in physics
at tick k+1.

Figure 2.1. One acknowledged tick; arrows denote data flow.

```text
accepted outputs -> mechanisms -> drivetrain -> physics
                                                  |
                                       game state -> game hook
                                                  |
                                       sensors -> snapshot
                                                  |
robot outputs <- HAL step <- extension <- sensor frame
      |
matching acknowledgement permits the next tick
```

Mechanisms model local loads such as a motor-driven roller. The drivetrain
converts wheel states to forces on the chassis. Physics integrates the bodies.
The game then recognizes resting cubes, applies rule transitions and publishes
values. The robot sees the resulting sensor frame during the extension's HAL
time step.

## 2.4 Ownership and process boundaries

The Qt UI reads immutable snapshots. A snapshot is a copied observation, not a
reference to a live physics body. User actions enter the simulation through
queued commands. This arrangement permits the UI to miss a rendered frame
without changing a game result.

`SimulationSession` owns synchronous simulation; `SessionRunner` adds pacing,
queues and IPC; `RobotSession` manages the robot process. The plugin does not
implement these services. It receives borrowed services through `SimContext`
and keeps its mutable data in the field and game-state objects.

**Invariant 2.1 (tick identity).** Only an acknowledgement for the current
session and run epoch can complete an outstanding tick. At most one tick is
outstanding. Reset changes the epoch even when the robot process stays alive.

Invariant 2.1 prevents a delayed pre-reset output from being applied to the
new world. It is enforced by runtime and extension code, rather than repeated
inside the plugin.

## 2.5 Repeatability

A fixed interval removes one source of variation but does not prove
determinism. Body creation order, contact processing, random inputs and worker
execution can also affect a rigid-body computation. The SDK provides
`stateHash()` for comparing repeated physical histories. Chapter 14 compares
hash sequences rather than relying only on final scores.

Fiedler discusses fixed time steps and deterministic network simulation [5, 6].
The emulator's acknowledged timing should not be confused with an input-only
network lockstep algorithm: this emulator exchanges sensor and actuator frames.
The relevant lesson is the need to identify every input to the transition.

## 2.6 Summary

Simulation time is an input to the model, not a measurement of desktop speed.
The plugin runs after physics on the simulation thread. Session and epoch
checks remain the responsibility of the existing runtime.

## 2.7 Exercises

1. Trace the first physics step affected by a cube released in the game-state
   portion of tick 100.
2. Explain why a snapshot counter is unsuitable for delivering one-shot sounds.
3. Describe an experiment that distinguishes equal final scores from equal
   physical histories.

[Chapter 3](03-rigid-bodies.md)
