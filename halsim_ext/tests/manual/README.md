# Manual end-to-end tests for halsim_wpiemu

Not run by ctest (they need a WPILib installation, Gradle and a robot project).

* `fake_emulator.cpp` → target `wpiemu_halsim_fake_emulator`: a headless emulator that listens on the real local
  transport, completes the handshake, waits for `program_started`, enables the robot and single-steps lock-step ticks,
  printing every announced device and every changed output.
* `wpiemu-init.gradle`: the Gradle init script of ADR 0001 (makes `simulateJava`/`simulateNative` load the extension).
* `run_robot_project.ps1`: copies a robot project to `%TEMP%`, runs the two together and cleans up. Exit 0 /
  "E2E OK" when all ticks were acknowledged by the robot.

See `docs/halsim-extension.md` ("Tests") for the command line.
