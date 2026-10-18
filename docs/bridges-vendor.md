# Vendor device bridges (`bridges/vendor`)

Implements [ADR 0002](decisions/0002-phoenix6-interception.md) layer **L1** (HAL SimDevices) plus the optional **L2**
(read-only CTRE config read-back) and the **L3 extension seam**. The module depends only on `wpiemu_sdk` (ADR 0004);
there is no link-time dependency on WPILib or CTRE (both are resolved at run time from modules the robot process
already loaded).

```
 registerVendorBridges(DeviceBridgeRegistry&)              "ctre.phoenix6"  "ctre.phoenix5"
        │                                                        │
 VendorSimDeviceBridge (IDeviceBridge)  ──────────────────────────┘
   ├─ SimDeviceBridge (SDK)            rule table: name regex → DeviceDescriptor + bindings (data, ctre_rules.cpp)
   ├─ IncrementalSimDeviceBackend      late-value decorator: poll-safe while the robot builds devices
   ├─ GuardedSimDeviceBackend          write policy decorator (continuous single write, read-back, re-assert)
   │     └─ ISimDeviceBackend (SDK)    seam: fake in tests, HalSimDeviceBackend (HALSIM_*) in production
   ├─ ITimeSyncAdvisor                 wall-clock advice for the CTRE firmware sim
   ├─ CtreSimNative (optional, L2)     config read-back through dynamically resolved CTRE exports
   └─ ICtreObservationSink (L3)        fed by an opt-in hook provider (none shipped); gives the CAN bus label
```

## Using it (composition root / `halsim_ext`)

```cpp
#include "wpiemu/bridges/vendor/vendor_bridges.h"
namespace vendor = wpiemu::bridges::vendor;

vendor::registerVendorBridges(registry);                 // defaults: real HAL binding, config read-back on
// or: vendor::VendorBridgeOptions o; o.backend = myBackend; o.onConfig = ...; vendor::registerVendorBridges(registry, o);
auto bridges = registry.createAll();                     // "ctre.phoenix6", "ctre.phoenix5"
```

Optional capabilities are reached through `dynamic_cast` on the created `IDeviceBridge*` (they are plain interfaces,
the SDK stays untouched):

| Interface | Purpose |
|---|---|
| `IVendorMirrorSync::reassertAfterVendorMirror()` | call from a HAL `SimPeriodicBefore` callback that runs *after* the CTRE wrappers' own mirror (see "Pigeon yaw") |
| `ITimeSyncAdvisor::timeSyncAdvice()` | wall-clock pacing advice (see "Time base") |
| `VendorSimDeviceBridge::setInputWritesEnabled(id, bool)` | "emulator owns the device" switch (see "Robots that simulate themselves") |
| `IDeviceBridge::resetInputHistory()` (default no-op) | generic clock-thread lifecycle hook; vendor override discards input/unwrap history without changing firmware offsets |
| `VendorSimDeviceBridge::configOf(id)` / `VendorBridgeOptions::onConfig` | CTRE config snapshot (gear ratio, invert, remote sensor); filled from `collectOutputs()` / `readPendingConfigs()` only |
| Reversed motors | commands reach the hub **after** the controller's own inversion; the hub negates them (and the rotor feedback written back) for a mapping with `reversed` ([ADR 0014](decisions/0014-multi-motor-gearboxes.md)). Reading `invert` from `configOf()` to pre-fill the flag is not done yet |
| `VendorSimDeviceBridge::readPendingConfigs(n)` | explicit config read-back for a host that knows the robot is quiescent |
| `ICtreObservationSink` | the sink an opt-in hook provider feeds (bus label) |

Unlike the vendor-specific optional interfaces, input history reset is reached through the SDK bridge
interface (no cast or extension-to-vendor dependency). The extension's `BridgeSet` invokes it before
feedback on each new connection/reconnect, applied Reset/BeginLockstep, or adoption of a newer frame
epoch. Duplicate commands do not invoke it again. EndLockstep itself keeps history until the next
Begin. The existing concrete helper remains callable for other hosts and overrides the SDK hook.
For example, wrapped physical yaw `0 -> 170 -> -170` gives raw `190` degrees; after resetting history,
new physical heading `0` writes raw `0`, not `360`. Robot `setYaw` offsets remain firmware-owned.

Build option: `-DWPIEMU_WPILIB_ROOT=<dir>` where `<dir>` (or `<dir>/include`) contains `hal/simulation/SimDeviceData.h`
compiles the real binding (`WPIEMU_VENDOR_HAS_WPILIB`). Without it the bridges register but see no devices and log one
warning at start. The binding attaches to `wpiHal` with `GetModuleHandle` / `dlopen(RTLD_NOLOAD)`; it never loads a copy.

## Identity and bus (ADR 0006)

Every rule has `busIsStatic = false`: SimDevice names carry no bus, so `DeviceDescriptor::bus == kBusUnknown` and the
bridge never fabricates one. `DeviceKey::physical()` (family/type/id) is the simulator identity: a TalonFX 4 on `rio`
and one on a CANivore are the same physical device (the HAL SimDevice name `CANMotor:Talon FX (v6)[4]` is the same).
The CANivore label can only come from an L3 observation: an opt-in `ICtreHookProvider` calls
`onDeviceObserved({type, id, "canivore"})` (any thread); the bridge applies it in `poll()` (retrying until the device is
announced) through `SimDeviceBridge::observeBus` → `IBridgeHost::observeBus`. The hub then handles aliases/conflicts.
No inline-hook code ships here (CTRE license, ADR 0002 R7): only the interface (`ctre_native.h`).

## Capability tables

Names and directions were re-verified against the Phoenix 6 **26.3.0** wrapper sources (`com.ctre.phoenix6.hardware.*`,
`sim.*SimState`) and the CTRE tools headers. Directions are as seen by the simulator: **In** = we write (HAL input),
**Out** = we read (robot→sim). Canonical names/units: `sdk/devices/signals.h`. ✔ verified in 26.3.0 sources, ◐ from ADR 0002 only.

### Phoenix 6 (`family=phoenix6`, `vendor=ctre`)

**TalonFX** (`type=talonfx`: Kraken X60/X44, Falcon 500 – one device type; the motor model is a robot-config choice)
and **TalonFXS** (`type=talonfxs`, identical tree with model `Talon FXS`) ✔

| SimDevice (`%d` = CAN id) | value | dir | signal | unit |
|---|---|---|---|---|
| `CANMotor:Talon FX (v6)[%d]` | `supplyVoltage` | In | `supplyVoltage` | V |
| | `dutyCycle` | Out | `dutyCycle` | -1..1 |
| | `motorVoltage` | Out | `motorVoltage` (loopback → `rotorVelocity`) | V |
| | `torqueCurrent` | Out | `torqueCurrent` | A |
| | `supplyCurrent` | Out | `supplyCurrent` | A |
| `CANEncoder:…/Rotor Sensor` | `rawPositionInput` | In | `rotorPosition` | rot (rotor, before ratios) |
| | `velocity` | In | `rotorVelocity` | rps |
| | `acceleration` | In | `rotorAcceleration` | rps² |
| | `position` | Out | `rotorPositionReported` | rot (includes robot `setPosition`) |
| `CANDIO:…/Fwd Limit`, `/Rev Limit` | `value` (HAL: bidir) | In (overridden) | `forwardLimit` / `reverseLimit` | bool, closed = true |

Capabilities: MotorOutput, RotorFeedback, PositionSensor, VelocitySensor, LimitInputs, SupplyVoltage. `init`/`input`
helper values are not bridged. We drive the **rotor**; Phoenix applies `SensorToMechanismRatio`/offsets (ADR 0002).
Not available through SimDevices: temperature, faults, `MotorType` (X60/X44; L2 physics key only). Limit polarity: the
sim value is the contact state ("closed"); with `NormallyOpen` (default) closed = limit active, with `NormallyClosed`
the hub must invert (`CtreConfigSnapshot::forwardLimitType()/reverseLimitType()`, 0 = NormallyOpen).

**CANcoder** (`cancoder`) ✔: `CANEncoder:CANcoder (v6)[%d]`: `supplyVoltage` In → `supplyVoltage`; `rawPositionInput` In →
`position` (rot); `velocity` In → `velocity` (rps); `position` Out → `positionReported`. Capabilities: PositionSensor,
VelocitySensor, SupplyVoltage. Absolute position is the same integrated value (magnet offset/direction are firmware
config; read them with L2 `magnetOffsetRot()`/`clockwisePositive()`). `MagnetHealth` is an L2 physics key only.

**Pigeon 2** (`pigeon2`) ✔: `CANGyro:Pigeon 2 (v6)[%d]`: `supplyVoltage` In; `rawYawInput` In → `yaw`; `pitch`, `roll` In;
`angularVelX/Y/Z` In → `angularVelX/Y/Z`; `yaw` Out → `yawReported`. **Degrees and deg/s on the wire**, canonical rad and
rad/s (`scale = π/180`, inverse applied on write). Capabilities: Imu, SupplyVoltage. CTRE sim has **no accelerometer/gravity
inputs** (no setters in `Pigeon2SimState`): `accelX/Y/Z` are never announced and the hub must synthesise them.

**CANdle** (`candle`) ✔: `CAN:CANdle (v6)[%d]`: `supplyVoltage`, `fiveVRail` (V), `outputCurrent` (A), `temperature` (°C) In;
`vbatModulation` Out. Capabilities: Led, SupplyVoltage (observed only). LED colours/animations are not in SimDevices
(would need L3 `RequestControl*` hooks).

**CANrange** (`canrange`) ✔: `CAN:CANrange (v6)[%d]`: `supplyVoltage`, `distance` (m) In → `distance`; `isDetected` Out →
`detectedReported` (computed by the firmware sim from `distance` and the robot's proximity config, so there is no
detected *input*). Capabilities: Distance, SupplyVoltage.

**CANdi** (`candi`) ✔: root `CAN:CANdi (v6)[%d]` (`supplyVoltage`, `requestedOutputCurrent` In; `outputCurrent` Out),
`CANDutyCycle:…/PWM1|PWM2` (`position` rot, `velocity` rps, `connected`, `riseToRise` s, `riseToFall` s, all In →
`pwm1…`/`pwm2…`), `CANEncoder:…/Quadrature` (`rawPositionInput` → `position`, `velocity` → `velocity`, `position` Out →
`positionReported`), `CANDIO:…/S1|S2` (`value`, labelled *Output* by the wrapper but it is the sim's input: direction
overridden to In; S1 → `digitalIn`, S2 → `digitalIn2`; true = closed). Capabilities: DigitalInput, PositionSensor,
VelocitySensor, SupplyVoltage. (Live `CANdi`/`TalonFXS` devices are still unproven on a running robot, ADR R8.)

### Phoenix 5 (`family=phoenix5`) ◐

Names/values come from ADR 0002 (live enumeration + bytecode); the Phoenix 5 vendor library is not in this repo's
caches, so they are **not re-verified**. Unknown values are not bound (data-driven; fix the table, not the code).

| device | SimDevice | bound signals |
|---|---|---|
| `talonsrx` | `CANMotor:Talon SRX[%d]` | `busVoltage`→`supplyVoltage`, `percentOutput`→`dutyCycle`, `motorOutputLeadVoltage`→`motorVoltage`, `supplyCurrent`/`motorCurrent`→`supplyCurrentInput`/`motorCurrentInput` (directions as the HAL reports them) |
| | `CANEncoder:…/Quad Encoder` | `rawPositionInput`→`position` (÷ CPR), `velocity`→`velocity` (counts/100 ms → rps: ×10÷CPR), `position`→`positionReported` |
| | `CANDIO:…/Fwd Limit`, `/Rev Limit` | `value` → `forwardLimit`/`reverseLimit` |
| | `CANEncoder:…/Analog In`, `/Pulse Width Input` | auto-bound as `Analog In/<value>`, … (raw sensor units, no canonical mapping) |
| `victorspx` | `CANMotor:Victor SPX[%d]` | as the SRX root |
| `pigeonimu` | `CANGyro:Pigeon IMU[%d]` | **discover-only**: no capabilities, native value names (heading semantics unverified) |

`Phoenix5Options::quadratureCountsPerRev` (default 4096 = CTRE Mag Encoder) converts raw counts. Plain (non-WPI) P5
classes and `Core*` classes have no SimDevices (ADR R1): not covered by L1.

## Pigeon yaw / integrated raw inputs (ADR 0002 R9, mitigated by design)

CTRE documents (`Pigeon2SimState.setRawYaw`, `CANcoderSimState.setRawPosition`): inputs must be **continuous**; the
firmware sim **integrates changes** of `rawYawInput`/`rawPositionInput` into the reported yaw/position, so a simulator can
move the sensor *without* overriding the robot's `setYaw`/`setPosition` (those only move a firmware offset). The spike's
"alternation" is most plausibly a consequence of non-idempotent writes (re-writing an absolute value from another thread
while the wrapper's own `SimPeriodicBefore` mirror writes the native value back into the SimDevice; not reproduced here).
`GuardedSimDeviceBackend` therefore:

1. **writes once per tick**: guarded values (`rawYawInput`, `rawPositionInput` of every `(v6)` device) are held back during
   `applyInputs` and flushed in `endWrites()`; the last sample of a tick wins;
2. keeps the value **continuous**: yaw is unwrapped (shortest-arc delta, degrees) so the hub's wrapped heading never injects a
   ±360° step into the integrator; unchanged values are not rewritten;
3. **never fights `setYaw`/`setPosition`**: it only ever writes absolute physical truth (offsets stay in the firmware);
4. **verifies by read-back** at the next tick; a changed value (mirror quirk, another writer) is counted and re-written;
5. `reassertAfterVendorMirror()` restores disturbed guarded values *between the vendor mirror and the robot code*. HAL runs
   `SimPeriodicBefore` callbacks in registration order and the CTRE wrappers register theirs at device construction, so the
   host's callback must be registered **after** them (see "Loop hook the host needs" below). Optional; without it a
   disturbance can be visible for one loop at most.

Delta writes (`AddYaw`/`AddPosition`, L2) would be the fallback if integration semantics ever change; the guard keeps the
absolute-write contract because it is idempotent and race-free. **Open item:** a robot-level characterisation run
(spike R9) should confirm `reasserts == 0` in steady state; `inputDisturbances()` exposes the counters.

## Time base (ADR 0002 R3)

The CTRE firmware sim uses the wall clock (ramps, PID derivative, Motion Magic), not HAL sim time, so uncapped, paused or
stepped runs make Phoenix controllers diverge from the robot loop. Mitigation (a hint, never forced):

* `ITimeSyncAdvisor::timeSyncAdvice()` returns `{paceToWallClock, maxRealtimeFactor = 1.0, pauseUnsafe}` while any
  Phoenix device is live (`TimeSyncPolicy::Ignore` turns it off, e.g. headless runs without closed-loop Phoenix code).
* Pause/step: a long `HALSIM_PauseTiming` lets the firmware sim run ahead; apply the advice (real-time pacing) and avoid
  long pauses while Phoenix closed-loop code is active. This was *not* re-measured here (M1 test with 0.5×/2× pending).

## Robots that simulate themselves (ADR 0002 R4)

If robot code (maple-sim, `SimState` writers) also writes the same inputs, last-writer-wins. Detection: every written
input is read back next tick; a persistent mismatch (≥ `GuardOptions::foreignWriteThreshold`, default 5) is reported once
through the logger (`bridge.vendor.guard`) and in `inputDisturbances()` (`foreignWriter = true`). Policy: guarded inputs
are re-asserted every tick; `setInputWritesEnabled(id, false)` hands a device back to the robot-side simulation (we still
read its outputs; loopback is skipped). A `supplyVoltage` below 4 V is clamped by CTRE and shows up as a (harmless) mismatch.

## Concurrent device construction (live polling)

**Symptom (first end-to-end run, FRC-Shelby-2026OS, Java, 29 devices):** with `WPIEMU_LIVE_DISCOVERY=1` (polling while the
robot constructor still creates devices) the JVM died with `0xC0000005` right after the 2nd/3rd TalonFX.

**Root cause (reproduced and bisected with the real robot):** the CTRE config read-back (L2) ran inside `poll()`. It enters
`CTRE_PhoenixTools_Sim` (`encode_device` + `get_configs_no_malloc`) from the extension thread while robot threads are inside
the same native library creating/configuring the next devices (`SimCreate`, `set_configs`); that library is not safe for
that. Evidence: identical run with config read-back disabled → E2E OK, 29 devices; with it → crash, deterministically at
the first device whose construction overlapped a read; with the fix (below) live polling is stable (3 runs). The HAL-only
part of polling was never the crash, but the first design had two further flaws that are fixed too:

1. *Late values were lost.* The SDK `SimDeviceBridge` lists a device's values once; a SimDevice seen half-built (device
   created, values still being created) would never get its remaining values. My first answer was a "settle for N polls"
   heuristic in the HAL binding. It was wrong (it hid every device from the single program-start poll: "1 device(s)
   announced" instead of 29) and is gone.
2. *Stale handles / lock order.* HAL calls our callbacks on robot threads while holding its own lock.

**Fix (all in `bridges/vendor`):**

* **No CTRE native call from `poll()`.** `poll()` is HAL-SimDevice enumeration only. Config read-back runs from
  `collectOutputs()` (the SDK contract: only while the robot is quiescent, i.e. between `StepTiming` calls; one device per
  call) or from `readPendingConfigs(n)` for a host that knows it is quiescent. In free-running modes (no `collectOutputs`)
  no config is read unless the host calls `readPendingConfigs`.
* **`IncrementalSimDeviceBackend`**: a device is reported the instant it exists; values created later come out as a pseudo
  device with the *same name* and a synthetic handle that lists only the new values. The rule matches it like the original,
  the bridge appends the signals (stable indices) and re-announces. No timing, no extra latency; the single program-start poll
  is complete. Pseudo devices disappear with their device.
* **HAL binding rules** (`hal_sim_device_backend.cpp`): callbacks only bump an atomic generation counter; every other call
  runs on the poll thread and never calls HAL while holding the binding's mutex (lock order is always HAL → mutex); per-device
  `HALSIM_RegisterSimValueCreatedCallback` (registered before the consumer lists the values, so nothing falls in the gap)
  wakes the poll up for late values; handles are only dereferenced by HAL (stale handles are no-ops for
  `HAL_Get/SetSimValue` and the enumerators); no SEH/exception tricks.
* Regression tests: `test_live_poll.cpp` (thread creating devices/values late while the main thread polls: nothing lost, no
  native call while "busy") and the `vendor HAL binding: polling while another thread constructs devices…` case against a
  fake HALSIM API with real-HAL locking.

Consequence for hosts: it is now safe to poll at any time; the extension may re-enable `WPIEMU_LIVE_DISCOVERY` by default
from the vendor bridge's point of view. A device announced early may be re-announced with more signals
(`DeviceAnnounce` is idempotent by key; children such as the TalonFX `Fwd Limit` arrive as separate SimDevices anyway).

## Loop hook the host needs (`reassertAfterVendorMirror`)

`halsim_ext` cannot call `IVendorMirrorSync` today: it must not include `bridges/vendor` (ADR 0004) and the SDK has no
neutral interface. Minimal API:

1. **SDK (e.g. `sdk/devices/loop_hook.h`)**
   ```cpp
   // Implemented by a bridge that wants to run right before robot code each loop iteration.
   class IRobotLoopHook {
       WPIEMU_INTERFACE(IRobotLoopHook)
   public:
       // ROBOT thread, from a HAL SimPeriodicBefore callback registered after every vendor mirror callback.
       // Must be fast, lock-free of the clock thread's bridge state except its own mutexes, and never call HAL timing.
       virtual void onRobotLoopBegin() = 0;
   };
   ```
   `VendorSimDeviceBridge` would implement it by forwarding to `reassertAfterVendorMirror()` (one line; not added here
   because the SDK is owned elsewhere).
2. **`halsim_ext`**: after each `BridgeSet::poll()` that announced a device, (re-)register ONE
   `HALSIM_RegisterSimPeriodicBeforeCallback` (`HALSIM_CancelSimPeriodicBeforeCallback` the previous uid first). HAL runs
   those callbacks in registration order and the CTRE wrappers register theirs when a device is constructed, so
   re-registering after new devices keeps the host's callback last. The callback `dynamic_cast`s each bridge to
   `IRobotLoopHook` (cached at registration) and calls `onRobotLoopBegin()`. Only needed in lock-step runs (in
   free-running modes the robot loop is not synchronised with writes anyway).

Until then the bridge works without it: disturbed guarded inputs are restored at the next tick's write.

## Optional CTRE native supplement (L2) – OFF except config read-back

`CtreSimNative` binds `c_ctre_phoenix6_encode_device`, `c_ctre_phoenix6_get_configs_no_malloc` and
`c_ctre_phoenix6_platform_sim_{get_physics_value,set_physics_input}` through `ISymbolResolver`
(`ModuleSymbolResolver`: `GetModuleHandle`/`dlopen(RTLD_NOLOAD)`, attach-only). Missing module/symbols are tolerated.
Only `readConfigs` is used by the bridge, once per Phoenix 6 device after it is announced, **only while the robot is
quiescent** (`collectOutputs()` / `readPendingConfigs()`, one device per call; never from `poll()`, see above; bounded
retries, fixed back-off while the module is absent); the result is `CtreConfigSnapshot` (`s_<n>` float bits /
ints, `f_<decimal>`; typed: `clockwisePositive()`, `sensorToMechanismRatio()`, `rotorToSensorRatio()`,
`feedbackSensorSource()`, `remoteSensorId()`, `magnetOffsetRot()`, limit types). Physics get/set exist for opt-in features
(fresh reads, `Add*`, probe grid for `Core*` classes) but are **not** used by the shipped bridges. There is no SDK channel
yet to forward the snapshot to the emulator (UI auto-fill); use `VendorBridgeOptions::onConfig`.

## Extension recipe: adding a vendor (e.g. REV, NavX)

1. **Discover names**: run a robot with the vendor's wrapper in simulation and dump `HALSIM_EnumerateSimDevices("")` +
   `HALSIM_EnumerateSimValues` (names, directions, types), or read the wrapper sources. Note units.
2. **Write the table** (`src/<vendor>_rules.cpp`, declared in `<vendor>_rules.h`): one `SimDeviceRule` per device family:
   `family`, `type`, `vendor`, `capabilities`, `nameRegex` (full match; group 1 = id, optional child group), bindings to
   canonical signals (`signals.h`) with `scale` (unit conversion) and `direction` overrides where the wrapper labels a
   sim *input* as output. CAN vendors: keep `busIsStatic = false`. Give all rules of one device identical type/capabilities.
3. **Policies** (only if needed): add `GuardedValueSpec`s for values the vendor firmware *integrates*; reuse the guard, do not
   fork it.
4. **Register**: in `vendor_bridges.cpp` add a `SharedSetup::makeRev()` creating
   `VendorSimDeviceBridge("rev.spark", rev::rules(), backend, options)` and one `registerBridge("rev.spark", {…})` call
   (open/closed: nothing existing changes; the extension still makes the single `registerVendorBridges` call).
5. **Test** with `tests/bridges_vendor/fakes.h` (`FakeSimBackend`, `FakeHost`, wrapper-faithful fixtures like `addTalonFx`):
   discovery/grouping, directions, scaling, missing values, retire/re-announce, guard behaviour.
6. **Document** the table in this file and run `python tools/check_boundaries.py`.

```cpp
// sketch: a REV-like rule (names illustrative, verify first!)
SimDeviceRule r;
r.name = "rev.sparkmax"; r.family = "rev"; r.type = "sparkmax"; r.vendor = "rev";
r.busIsStatic = false; r.bus = sdk::kBusUnknown;
r.nameRegex = R"(SPARK MAX \[(\d+)\])"; r.idGroup = 1;
r.capabilities = Capability::MotorOutput | Capability::RotorFeedback;
r.bindings = {{"", "Applied Output", signal::kDutyCycle, 1.0, 0.0, std::nullopt, "", ""},
              {"", "Position", signal::kRotorPosition, 1.0, 0.0, std::nullopt, "rot", ""}};
```

## Tests

`test_live_poll.cpp` covers polling while another thread constructs devices (see "Concurrent device construction").
`tests/bridges_vendor/` (doctest, no WPILib/CTRE needed except `test_hal_backend.cpp`, which builds only when the HAL
headers were found and drives the real binding against a fake HALSIM C API): rule tables for every device (happy path, bus
not fabricated, missing values/children, retire), guard behaviour (unwrap, coalesce, read-back, re-assert, foreign writer,
firmware-integration model with `setYaw`), registration, "emulator owns device", time-sync advice/pacer, L3 sink gating,
config parsing and read-back, absent-symbol tolerance. The HAL binding was additionally smoke-tested once against the real
`wpiHal.dll` (2026.2.1): devices created after registration were discovered, directions/types mapped and values written back.
Build/test recipe: `docs/BUILDING.md`.
