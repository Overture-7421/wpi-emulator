# Implementing a Game Season in the WPI Emulator

## FIRST POWER UP on the 2027 Systemcore ABI

This book develops a game plugin from a specification, a physical model and a set of
tests. The case study is FIRST POWER UP, played in 2018. Its robot programs use the
2027 WPILib preview and the systemcore HAL ABI. Game rules and WPILib compatibility
are separate subjects: choosing an ABI does not choose a game.

The intended reader is an FRC C++ mentor. Knowledge of Qt, the emulator, and WPILib's
simulation internals is not assumed. [The preface](00-preface.md) describes the
prerequisites and how the source listings become a working project.

| Part | Chapters |
|---|---|
| I. Foundations | [1. Scope](01-introduction.md), [2. Discrete time](02-discrete-time.md), [3. Rigid bodies](03-rigid-bodies.md), [4. Specification](04-specification.md) |
| II. Core plugin | [5. Registration](05-registration.md), [6. Field](06-field.md), [7. Cubes](07-cubes.md), [8. Plates](08-plates.md), [9. Match time](09-match-time.md), [10. Interaction](10-interaction.md), [11. Configuration and snapshots](11-observability.md), [12. Assembly](12-assembly.md), [13. Robot programs](13-robot-programs.md), [14. System verification](14-verification.md) |
| III. Game features | [15. Score accounting](15-scoring.md), [16. Human players](16-human-players.md), [17. Vault and power-ups](17-power-ups.md), [18. Parking and climbing](18-climbing.md), [19. Match scenarios](19-scenarios.md), [20. Presentation](20-presentation.md) |
| IV. Synthesis | [21. Another season](21-another-season.md), [22. Limitations](22-limitations.md) |
| Appendices | [A. Provenance](appendix-a-provenance.md), [B. Reproduction](appendix-b-reproduction.md), [C. Conformance](appendix-c-conformance.md), [D. Glossary](appendix-d-glossary.md), [E. Exercises](appendix-e-solutions.md), [F. References](appendix-f-references.md) |

The chapters are cumulative. A later listing marked as a revision replaces the
earlier file in full. No function begins in one chapter and ends in another.
Appendix B describes the optional extraction tool and the build checkpoints.

[Documentation home](../README.md)
