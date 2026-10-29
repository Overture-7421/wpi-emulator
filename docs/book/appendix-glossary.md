# Appendix C: the pit-wall glossary

| Term | In this book |
|---|---|
| **ABI** | The binary-facing WPILib/HAL generation. 2027's `systemcore` ABI changes robot-side headers and toolchain needs; it does not move season rules into core. |
| **HAL** | WPILib's hardware abstraction layer. Robot code talks to it instead of directly toggling real hardware. |
| **HALSim** | WPILib's simulation hooks. The emulator extension attaches here so robot code uses its ordinary device APIs. |
| **SDK** | The emulator's Qt-free public interfaces. A game plugin uses SDK types, not Box3D, Qt, or HAL internals. |
| **Season pack** | The year-specific manifest that selects WPILib versions, ABI, launch profile, and optional game plugin. |
| **Game module** | The stateless `IGameModule` factory and descriptor provider registered for one game. |
| **Game state** | The per-session `IGameState` that owns match rules, timers, score, and game entities. |
| **Field** | The `IField` owner for static geometry, visuals, and driver viewpoints. It is destroyed before the physics world. |
| **Simulation tick** | One fixed step owned by the emulator. Physics steps first; game rules consume the resulting events afterward. |
| **SI / NWU** | Metres, kilograms, seconds; right-handed North-West-Up coordinates with Z up. |
| **Sensor shape** | A physics volume that reports overlaps without pushing the overlapping body away. Both shapes must enable sensor events. |
| **Game-specific message** | The short FMS assignment sent to robot code before autonomous; for Power UP it is one of four three-character L/R strings. |
| **Practice mode** | A user-configurable Driver Station mode. Its durations need not match the official game manual. |
| **Quick 3D / QML** | Qt's scene renderer and declarative UI layer. The app maps SDK snapshot data into the scene; plugins stay Qt-free. |
| **2027 alpha-7** | The pinned WPILib preview used by these samples, not a promise that later 2027 releases have identical APIs. |

For the full thread, ownership, tick, and reset contracts, see [Architecture](../ARCHITECTURE.md).
For units and body types, see the [physics reference](../reference/physics.md).
