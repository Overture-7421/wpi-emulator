# HAL bridges (`bridges/hal/`)

Device bridges for the WPILib HAL families, running inside the robot process (loaded by `halsim_wpiemu`).
Each family is one `sdk::ChannelFamilyBridge` (or `IDeviceBridge` for CAN). They depend only on the SDK and
on a thin HAL seam, `IHalApi` (`wpiemu/bridges/hal/hal_api.h`); the real `HALSIM_*` binding is optional.
Written against WPILib / GradleRIO **2026.2.1** (ADR 0001, µs sim time).

## Wiring

```cpp
#include "wpiemu/bridges/hal/hal_sim_binding.h"      // only with WPIEMU_WPILIB_ROOT (target wpiemu::bridges_hal_halsim)
sdk::DeviceBridgeRegistry registry;
wpiemu::bridges::hal::registerHalBridges(registry);   // real HAL
// or, anywhere (tests, no WPILib):
wpiemu::bridges::hal::registerHalBridges(registry, std::make_shared<testing::FakeHalApi>(), HalBridgeOptions{});
auto bridges = registry.createAll();                   // one IDeviceBridge per family, registration order
```

* `wpiemu::bridges_hal` (always built): bridges + `IHalApi` + `testing/fake_hal_api.h`.
* `wpiemu::bridges_hal_halsim` (only when CMake option `WPIEMU_WPILIB_ROOT` points at the extracted
  `hal-cpp`/`wpiutil-cpp` headers zips and the platform `wpiHal` library): `makeHalSimApi()` and the one-argument
  `registerHalBridges(registry)`. `halsim_ext` links it when the target exists. Layouts searched are listed in
  `bridges/hal/CMakeLists.txt` (e.g. `<root>/hal-headers/hal/Ports.h`, `<root>/wpiutil-headers/wpi/string.h`,
  `<root>/hal-bin/windows/x86-64/shared/wpiHal.lib`). The static lib needs the robot's `wpiHal` at load time, as
  the extension already does (ADR 0001 §3).
* `HalBridgeOptions` injects the few values the HAL does not expose (see below).

## How discovery works

`ChannelFamilyBridge` polls the per-index *Initialized* getters on the clock thread and announces / retires on
transitions (the `live` flag is the per-index `initialNotify` transition tracking of ADR 0001). No HAL callback
runs on robot threads except CAN (below), so nothing here can block robot init. Every `HALSIM_*` index is
range-checked in the binding (HALSIM itself does not), and counts always come from `HAL_GetNum*()`.

## Families

Directions: **O** = robot -> sim (observed), **I** = sim -> robot (written). Canonical names/units are from
`sdk/devices/signals.h`; extras are bridge-specific. `DeviceKey` = `hal/<type>[id]@rio#channel`.

| Bridge | type / key | Capabilities | Signals |
|---|---|---|---|
| `hal.pwm` | `pwm[ch]` (controller model unknown => generic PWM) | MotorOutput, PwmOutput | O `dutyCycle` and `pwmSpeed` (HAL speed, clamped to [-1,1], NaN->0), O `pwmPosition` [0,1] (servos), O `pulseMicroseconds` |
| `hal.dio` | `dio[ch]` | **inputs**: DigitalInput; **outputs**: DigitalOutput (direction fixed at init) | input pin: I `digitalIn`; output pin: O `digitalOut`. The sim never writes an output pin |
| `hal.analog_in` | `analog_in[ch]` | AnalogInput | I `analogVoltage` (V) |
| `hal.analog_out` | `analog_out[ch]` | none | O `analogVoltage` |
| `hal.encoder` | `encoder[idx]` (display name `Encoder 0 (DIO a/b)`) | PositionSensor, VelocitySensor | I `position` (rot), I `velocity` (rps), I `count` (raw), O `digitalChannelA/B`, O `distancePerPulse`, O `reverseDirection` |
| `hal.duty_cycle` | `duty_cycle[idx]` | PositionSensor | I `position` (rot, wrapped into a [0,1) duty ratio), I `frequency` (Hz), O `digitalChannel` |
| `hal.relay` | `relay[header]#0` forward, `#1` reverse (separate devices) | DigitalOutput | O `digitalOut` |
| `hal.ctre_pcm` / `hal.rev_ph` | `ctre_pcm[module]` / `rev_ph[module]` (vendor ctre / rev) | none | O `compressorOn`, O `compressorClosedLoop` (PH: config type != disabled), I `pressureSwitch`, I `compressorCurrent` |
| `hal.ctre_pcm.solenoid` / `hal.rev_ph.solenoid` | `ctre_pcm_solenoid[module]#ch` / `rev_ph_solenoid[module]#ch` | Solenoid, DigitalOutput | O `solenoid` |
| `hal.power_distribution` | `power_distribution[module]` (PDP/PDH are indistinguishable in sim) | SupplyVoltage | I `supplyVoltage`, I `temperature`, I `channelCurrent0..N-1` (A) |
| `hal.addressable_led` | `addressable_led[idx]` | Led | O `running`, `length`, `outputPort`, `avgRed/Green/Blue` (0-255), `firstPixelRgb` |
| `hal.roborio` | `roborio[0]` (always present) | SupplyVoltage | I `supplyVoltage` (battery -> roboRIO VIn; the hub writes `SensorFrame.batteryVoltageV` to every SupplyVoltage device), I `vInCurrent`, O `brownoutVoltage` |
| `hal.accelerometer` | `accelerometer[0]` (live once the robot activates it) | Imu | I `accelX/Y/Z` in m/s^2 (HAL takes g) |
| `hal.can` | `can_<mfr>_<devtype>[devId]`, e.g. `can_rev_motor_controller[12]` | none | O `frameCount`, `lastApiId`, `lastPeriodMs`, `lastDataLow/High` (little-endian 32-bit halves) |

### Behaviour notes

* **PWM / motor output.** `HALSIM_GetPWMSpeed` is already scaled by the robot's PWM config, so it is the normalised
  `[-1, 1]` command.
* **Encoders.** HAL sim state is `count` (distance / distancePerPulse) and signed `period`. `position`/`velocity` are
  converted with `HalBridgeOptions::encoderCountsPerRevolution` (default 2048 counts per mechanism revolution; the
  HAL does not know the gearing). The robot's `reverseDirection` is honoured (the sim HAL ignores it): a reversed
  encoder reads negative, as on hardware. `velocity != 0` also sets the direction flag; at 0 the period is `+inf`.
  The DIO pins an encoder uses *also* appear as `dio` inputs: use `digitalChannelA/B` to pair them (HAL
  Counters have no sim data in 2026.2.1, so there is no counter bridge).
* **DutyCycle.** A new DutyCycle reports 0 Hz, which makes `DutyCycleEncoder.isConnected()` false. When the channel is
  announced with `frequency == 0` the bridge writes `dutyCycleDefaultFrequencyHz` (975, REV Through Bore) and
  restores 0 in `stop()` (unless the sim overrode it).
* **Solenoids.** The sim HAL has no per-channel "initialised" flag, so a solenoid channel is announced the first
  time the robot commands it ON and stays announced (latched) until its module is freed. Set
  `announceAllSolenoidChannels` to announce every channel of every live module instead. Pressure *sensor* readings
  (analog) arrive through `analog_in`; the sim only owns the pressure *switch* and compressor current.
* **PDP/PDH.** One family; channel signals cover the PDH maximum (24). Per-channel currents are sim inputs.
* **CAN (observe-only).** `HALSIM_RegisterCanSendMessageCallback` (the only HAL callback used) is invoked on
  whatever robot thread sends; the bridge records frames under a mutex and announces from `poll()`. Frames are grouped
  by `(manufacturer, device type, device id)` decoded from the 29-bit arbitration id; broadcast frames are ignored;
  at most 512 devices are tracked. 2026.2.1 callbacks carry no bus id, so every device is on `rio`. Nothing is ever
  answered on the bus. **Phoenix 6 does not use HAL CAN** (ADR 0001/0002) - those devices come from the vendor bridge's
  SimDevice rules; this bridge only sees classic HAL/CANAPI users.
* **Loopback.** All HAL families here are output-only or sim-driven inputs with nothing to echo, so
  `applyLoopback` is a no-op (the base-class default).

### Not covered

HAL Counters, analog triggers, accumulators, SPI/I2C devices, and SimDevice-based devices (NavX, ADIS, Phoenix, REV
Spark...: vendor bridges). DS data and sim time are owned by `halsim_ext`.

## Tests

`tests/bridges_hal/` (doctest, `FakeHalApi`): registration order/duplicates/null api, per-family announce/retire,
signal read/write, out-of-range channels and signal indices, reversed encoders, solenoid latch, CAN decode, lifecycle.
A manual smoke test against the real `wpiHal` (extracted 2026.2.1 zips) announced PWM/DIO/AnalogIn/PCM/solenoid/roboRIO
devices, returned PWM/solenoid/CAN outputs and wrote a DIO value through the real `HALSIM_*` calls.
