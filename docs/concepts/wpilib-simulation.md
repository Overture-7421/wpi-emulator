# How WPILib simulation works

Read this when you want to understand how an ordinary WPILib robot program runs without a roboRIO.

For example, a robot project can still call its normal motor and
[Driver Station](../GLOSSARY.md) APIs. Instead of
talking to physical hardware, the program runs on the desktop against WPILib's simulation HAL, and
the emulator's extension exchanges simulated device data with the desktop app. The robot code
does not need emulator-specific hooks.

## From robot code to simulated devices

Robot code calls WPILib APIs such as motor-controller and joystick APIs. Those APIs use the
[HAL](../GLOSSARY.md), WPILib's hardware abstraction layer. In a real robot, HAL reaches hardware
through the roboRIO. In desktop simulation, the HAL implementation exposes simulated state instead.

**[HALSim](../GLOSSARY.md)** is the simulation side of HAL. It provides APIs to read and write
simulated hardware state and can load extra simulator libraries. A
**[HALSim extension](../GLOSSARY.md)** is one such library. WPILib
loads libraries named by `HALSIM_EXTENSIONS` and calls each extension's
`HALSIM_InitExtension` entry point after loading it; see the installed
[WPILib simulator-extension
reference](https://github.wpilib.org/allwpilib/docs/release/cpp/group__hal__extensions.html).
The emulator implements that entry point in
[its extension composition root](../../halsim_ext/src/extension.cpp).

The emulator does not rewrite the robot project. Its launcher passes a generated Gradle init
script with `-I`. The script disables GradleRIO's default HAL extensions for the simulation task
and sets `HALSIM_EXTENSIONS` to the emulator library. The launcher selects `simulateJava` or
`simulateNative` for the project's language. See the
[launcher](../../core/runtime/src/launcher/robot_launcher.cpp) and
[init-script generator](../../core/runtime/src/launcher/init_script.cpp).

The extension translates between HAL state in the robot process and emulator messages. Its
[HAL platform adapter](../../halsim_ext/src/hal/hal_sim_platform.cpp) applies Driver Station
state, joystick data, battery feedback, and simulated time. Device bridges handle supported HAL
families and vendor simulation APIs; they do not make every hardware device or vendor library
available automatically. The [bridge reference](../reference/bridges.md) lists the implemented
semantics.

## SimDevice and vendor simulation APIs

A vendor library can expose simulated device values through WPILib's
**[SimDevice](../GLOSSARY.md)** facility.
These values live in the robot process, where the emulator's vendor bridges can discover and
exchange them. A bridge maps a supported set of those values to emulator signals; it is an
adapter, not a general CAN controller or a firmware emulator. See the
[bridge reference](../reference/bridges.md).

**Example:** a supported simulated motor controller exposes feedback values through its vendor
simulation interface. The extension bridge reads or writes those values while the desktop model
computes the motor and mechanism response. The robot still runs the vendor library's normal robot
code, but unsupported controllers and firmware behaviors are not thereby reproduced.

## Clock and Driver Station

The emulator owns [simulation time](../GLOSSARY.md). In lock-step mode, the app sends one sensor
update; the extension applies it, advances HAL timing, collects robot outputs, and acknowledges
the tick.
The Driver Station state supplied by the app includes robot mode and joystick data. The exact tick
ordering, pause behavior, and recovery rules belong to
[Architecture](../ARCHITECTURE.md) and the [extension and IPC reference](../reference/extension-ipc.md).

HAL callbacks run on robot threads. They record events rather than waiting for IPC or doing
socket work; see the [extension and IPC reference](../reference/extension-ipc.md).

This setup replaces hardware-facing interactions with simulated ones. It does not run on a
roboRIO, connect to real robot hardware, or reproduce every physical, electrical, firmware, or
vendor-library detail. The supported device and camera behavior is defined by the
[bridge reference](../reference/bridges.md).

## Where to look in this repository

- Start the robot: [`RobotSession`](../../app/host/robot_session.cpp) and
  [`robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp).
- Inject the extension: [`init_script.cpp`](../../core/runtime/src/launcher/init_script.cpp).
- Extension entry point: [`HALSIM_InitExtension`](../../halsim_ext/src/extension.cpp).
- Apply HAL timing and Driver Station state:
  [`hal_sim_platform.cpp`](../../halsim_ext/src/hal/hal_sim_platform.cpp).
- Synchronize frames and lifecycle: [extension and IPC](../reference/extension-ipc.md).
- Map simulated devices: [bridges](../reference/bridges.md).
