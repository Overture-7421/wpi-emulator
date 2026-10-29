# Appendix D. Glossary and notation

| Term | Meaning in this book |
|---|---|
| ABI | Binary interface expected by a compiled library; selected by the season's HAL profile |
| Aggregator | Composition target that registers built-in plugins without exposing them to app code |
| Body | One rigid pose, velocity and collection of collision shapes |
| Collider | Shape participating in physical contact |
| Custody | The unique logical location of a conserved cube identity |
| Descriptor | SDK data interpreted by a generic application component |
| Epoch | Run identity changed by reset, distinct from the process lifetime |
| HAL | Hardware Abstraction Layer used by WPILib |
| HALSim | HAL's desktop simulation interface |
| IPC | Local interprocess exchange between robot extension and emulator |
| Kinematic | Body whose velocity is prescribed rather than solved from applied loads |
| Module | Stateless, registry-owned factory and descriptor provider |
| NWU | North-West-Up, the right-handed field convention used by the SDK |
| Oracle | Independent statement of the expected result of a test |
| Physics step | Integration and contact portion of a tick |
| Pool | Fixed allocation of cube identities and bodies |
| Result / Status | SDK success-or-error return types; errors do not cross as exceptions |
| Season pack | Data selecting WPILib version, HAL ABI and optional game |
| Sensor | Overlap shape that produces no contact response |
| Snapshot | Immutable observation read outside mutable simulation state |
| Systemcore | The HAL ABI profile used by the pinned 2027 preview |
| Tick | Complete emulator transition, including game logic and publication |
| Virtual outcome | Scripted referee input for a robot without a simulated body |
| Zone | Configured, attachment-relative region or release/hook endpoint |

L and W are field length and width. A pose transforms a local point into world
coordinates. Quaternion components are ordered x, y, z, w. Half-extents describe
a collision box; full dimensions describe its visible size. One rule second
is 1,000,000 microseconds.

L0-L5 denote test levels from Chapter 14, not game levels. Force and Boost levels
1-3 select targets; they do not multiply duration. F-01 through F-25 identify
conformance rows, while R1 through R13 identify implementation requirements.

[Appendix E](appendix-e-solutions.md)
