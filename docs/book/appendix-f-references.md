# Appendix F. References

The official documents supply game facts; the remaining references supply
simulation background. Repository-specific contracts are linked at the point
of use. Online sources below were opened or directly retrieved during the
source audit. No manual diagram or extended manual passage is reproduced.

1. FIRST Robotics Competition. *2018 Game & Season Manual*, 2018. Official PDF
   metadata identifies the subject as FIRST POWER UP. Sections 3-4 supply field
   and scoring definitions; section 7 supplies setup and human-player rules;
   the glossary supplies parking and climbing definitions.
   [Official manual](https://firstfrc.blob.core.windows.net/frc2018/Manual/2018FRCGameSeasonManual.pdf).

2. FIRST Robotics Competition. *2018 Field Layout and Marking*, drawing
   FE-00041, revision 1, especially sheets 2-4.
   [Layout and marking](https://firstfrc.blob.core.windows.net/frc2018/Drawings/LayoutandMarkingDiagram.pdf).
   Component detail: *FIRST POWER UP Season-Specific Field Drawings*, 199-page
   package, beginning with GE-18004 revision B.
   [Component drawings](https://firstfrc.blob.core.windows.net/frc2018/Drawings/FieldDrawings-FIRSTPOWERUPspecific.pdf).

3. WPILib. “Introduction to Robot Simulation.” *WPILib Documentation*.
   [Simulation introduction](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html).
   This is background on desktop simulation, not documentation of the preview
   robot APIs used in Chapter 13.

4. Shenzhen Robotics Alliance. *maple-sim*. Project README.
   [Repository](https://github.com/Shenzhen-Robotics-Alliance/maple-sim).
   The project uses dyn4j for two-dimensional rigid-body simulation in Java.

5. Fiedler, Glenn. “Fix Your Timestep!” *Gaffer On Games*.
   [Article](https://gafferongames.com/post/fix_your_timestep/).

6. Fiedler, Glenn. “Deterministic Lockstep.” *Gaffer On Games*.
   [Article](https://gafferongames.com/post/deterministic_lockstep/).
   The network architecture is background; the emulator's own IPC contract
   remains the authority for its timing behavior.

7. Catto, Erin. *Box3D*, initial alpha release, tag v0.1.0.
   [Pinned release](https://github.com/erincatto/box3d/releases/tag/v0.1.0).
   A floating default branch is not the engine version used by this experiment.

[Contents](README.md)
