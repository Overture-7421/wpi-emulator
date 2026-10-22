# Documentation

Read this when finding your way around WPI Emulator or choosing where to make a change.
Prerequisites: none; [the project overview](../README.md) explains what the emulator does.

The emulator runs ordinary WPILib robot code in a separate process. You configure the robot's
physical model in the desktop app; you do not need to add emulator-specific hooks to robot code.
Extending the simulator itself is a different task from configuring a robot.

## Choose your path

| I want to... | Start here | Then |
|---|---|---|
| Run my team's robot | [User guide](USER_GUIDE.md) | [Swerve sample](../samples/swerve_robot/README.md) or [vision samples](../samples/vision_robot/README.md) |
| Build or contribute | [Building](BUILDING.md) | [Architecture](ARCHITECTURE.md), then [Contributing](CONTRIBUTING.md) |
| Add simulator support | [Extension decision guide](extending.md) | A task guide below, then its owning reference |
| Diagnose a failure | [Maintenance guide](guides/maintenance.md) | The relevant subsystem reference below |
| Produce a release | [Releasing](releasing.md) | [User installation instructions](USER_GUIDE.md#install-and-launch) |
| Change or publish documentation | [Documentation maintenance](guides/documentation.md) | [Contribution policy](CONTRIBUTING.md#documentation) |
| Understand a design trade-off | [Architecture decisions](decisions/README.md) | The current reference, not historical implementation assumptions |

## Extension guides

Choose the smallest existing seam before adding code. A new motor's physical constants are not
a new controller bridge, and a new intake is often a configuration of the generic mechanism rather
than a new mechanism class.

| Task | Guide |
|---|---|
| Add a HAL device family, SimDevice binding, or vendor adapter | [Devices and bridges](guides/devices.md) |
| Add motor data or understand model/controller responsibilities | [Motors](guides/motors.md) |
| Configure or implement a mechanism and plant | [Mechanisms](guides/mechanisms.md) |
| Bring up another game year | [Game years](guides/game-years.md) |
| Connect pickup/shooter zones, game-fed sensors and sound cues | [Game interactions](guides/game-interactions.md) |
| Add a camera vendor or test robot-facing results | [Vision](guides/vision.md) |
| Add configurable editor fields | [Editor properties](guides/editor-properties.md) |
| Add drivetrain, tire, physics, or protocol strategies | [Simulation strategies](guides/simulation-strategies.md) |

The guides own procedures and examples. The reference pages own behavioral contracts; link to
them instead of copying their defaults, equations, or lifecycle rules into another manual.

## Subsystem references

| Reference | Questions it answers |
|---|---|
| [Runtime and configuration](reference/runtime.md) | How is `robot.json` decoded, validated, saved, and turned into a session? Who owns the process? |
| [Mechanisms](reference/mechanisms.md) | What do slots, motor commands, sensors, attachments, and motion links mean? |
| [Physics](reference/physics.md) | How do chassis contact, tire forces, coupling, and deterministic stepping work? |
| [Extension and IPC](reference/extension-ipc.md) | How does the robot attach, acknowledge ticks, reset, reconnect, and recover from stalls? |
| [Bridges](reference/bridges.md) | Which device and camera semantics must an adapter preserve? |
| [Game plugins](reference/game-plugin.md) | Who owns season data, field objects, game state, zones, and HUD values? |
| [Application](reference/app.md) | How do the shell, editor, run view, host, and immutable snapshots fit together? |

## Working examples and evidence

The [season skeleton](../plugins/_template/README.md) is the starting point for a plugin;
the [new-season integration test](../tests/plugins/test_new_year_plugin.cpp) demonstrates wiring.
The [swerve](../samples/swerve_robot/README.md) and [vision](../samples/vision_robot/README.md)
projects are robot programs, not templates for simulator internals.

Follow each example's prerequisites and expected results. A successful build does not prove robot
integration; a skipped opt-in test is not a pass. [Building](BUILDING.md#opt-in-robot-project-tests)
explains how to run those checks. Windows results do not establish Linux/macOS robot loading or
interactive rendering. Known limits remain explicit in the [user guide](USER_GUIDE.md#limits-and-platforms).

## Repository and Wiki

Repository Markdown is authoritative. The [GitHub Wiki](https://github.com/Overture-7421/wpi-emulator/wiki)
is a generated reading surface published from reviewed default-branch content, once publishing is
configured. Edit repository files through a pull request, not generated Wiki pages.
See [documentation maintenance](guides/documentation.md) for preview, setup, and publication failures.

Next: [build the project](BUILDING.md), or [choose an extension seam](extending.md).
