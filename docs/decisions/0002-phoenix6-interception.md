# ADR 0002 – Intercepting CTRE Phoenix 6 (and Phoenix 5) from an unmodified robot

Status: **Accepted (proven on Windows, Phoenix 6 26.3.0 / GradleRIO 2026.2.1, Java robot)** · Spike: `spike-phoenix`

## TL;DR

Phoenix does **not** put CAN frames on the HAL CAN simulation, so "CAN-frame emulation" is not a
workable fallback. Phoenix simulation is a **self-contained software model of each device's firmware
inside `CTRE_PhoenixTools_Sim`** (one shared library per OS) and it is reachable from our in-process
HALSim extension through two independent, proven channels:

| Layer | Channel | Role |
|---|---|---|
| **L1 – primary** | **HAL `SimDevice`** values (`HALSIM_*SimDevice*` / `HAL_GetSimValue` / `HAL_SetSimValue`). Phoenix's WPILib wrapper classes (`com.ctre.phoenix6.hardware.TalonFX`, `CANcoder`, `Pigeon2`, `CANdle`, `CANrange`, `CANdi`, `TalonFXS`, P5 `WPI_TalonSRX/VictorSPX/PigeonIMU`) publish every device as SimDevices (the same data the WPILib Sim GUI shows). | Detect, read commanded outputs, write sensor/supply state. Pure WPILib API: no CTRE binary coupling, race-free, portable. |
| **L2 – supplement** | CTRE's own public C sim ABI exported by `CTRE_PhoenixTools_Sim` (`c_ctre_phoenix6_platform_sim_*`, the exact functions `*SimState` call), resolved with `GetProcAddress`/`dlsym` on the already-loaded module. | Devices without SimDevices (`Core*` classes, plain P5 `TalonSRX`), fresh (no one-loop latency) output reads, `Add*` delta writes, motor type, **config read-back** (gear ratios, remote sensor ids, inverts, limits). |
| **L3 – optional** | Inline hooks (MinHook / funchook) on `c_ctre_phoenix6_encode_device`, `ctre::phoenix::platform::SimCreate`, `c_ctre_phoenix6_RequestControl*`. | CAN bus name per device (CANivore name), instant detection for non-SimDevice classes, control-request intent for UI labels. **Off by default.** |

The robot's code is never touched: we never call, and the robot never needs, any `*SimState` API.

**Proof (real project copy, FRC-Shelby-2026OS, 88 SimDevices, 20 TalonFX / 5 CANcoder / 1 Pigeon2 / 2 CANrange):**
from the extension we detected every device, read `motorVoltage` / `torqueCurrent` of a TalonFX the robot was
driving closed-loop, and wrote rotor velocity/position, CANcoder position, Pigeon2 yaw and supply voltage.
The robot's own telemetry (`SmartDashboard/Subsystems/Shooter/Velocity` in its wpilog) went from `0` to
exactly `-30.000` (= −50 rps rotor ÷ SensorToMechanismRatio 1.6667, with `Inverted=true`) – with both L1 and L2.
HAL CAN send callbacks observed: **0**.

## Verified facts (evidence)

1. **Phoenix sim is internal.** `CTRE_PhoenixTools_Sim.dll` imports only `WS2_32`, `WINMM`, CRT/KERNEL32 – **no
   `wpiHal`**, no HAL CAN. A `HALSIM_RegisterCanSendMessageCallback` observer saw 0 messages over a 15 s enabled run.
   Console shows a single virtual bus: `[phoenix] CANbus Connected: sim`. It also starts the Tuner X diagnostics
   server (`[phoenix-diagnostics] Server … running on port: 1250`).
2. **Firmware per device is simulated.** `PhoenixTools_Sim` statically depends on one plugin DLL per device family
   (`CTRE_SimProTalonFX`, `…ProCANcoder`, `…ProPigeon2`, `…ProCANdle`, `…ProTalonFXS`, `…ProCANrange`, `…ProCANdi`,
   `CTRE_SimTalonSRX`, `CTRE_SimVictorSPX`, `CTRE_SimPigeonIMU`), each exporting a single
   `SIMULATION_PRO_<Device>_InitLibrary(_SimLibInterface_t&, _SimApiInterface_t&)`. Control laws (duty, voltage,
   velocity/position PID, Motion Magic, current/limit handling, following, fused CANcoder) execute inside it:
   we only provide rotor state and read the resulting `MotorVoltage`. Output updates synchronously with
   input writes (measured ≈0.03 ms from `RotorVelocity` write to new `MotorVoltage`; 1288/1296 writes changed it).
3. **Public sim control ABI** – shipped in CTRE's *public* header
   `ctre/phoenix6/platform/platform_c_style.h` (tools-sim headers zip) and exported by `CTRE_PhoenixTools_Sim`
   (also Linux `.so`; the same names appear in `libCTRE_PhoenixTools_Sim.so` for x86-64 and arm64):
   ```c
   int32_t c_ctre_phoenix6_platform_sim_create(int deviceType, int id);
   int32_t c_ctre_phoenix6_platform_sim_destroy(int deviceType, int id);
   int32_t c_ctre_phoenix6_platform_sim_set_physics_input(int deviceType, int id, char const *physicsType, double value);
   int32_t c_ctre_phoenix6_platform_sim_get_physics_value(int deviceType, int id, char const *physicsType, double *value);
   int32_t c_ctre_phoenix6_platform_sim_get_last_error(int deviceType, int id);
   ```
   `DeviceType`: `TalonSRX=0, VictorSPX=1, PigeonIMU=2, RibbonPigeonIMU=3, P6_TalonFX=4, P6_CANcoder=5,
   P6_Pigeon2=6, P6_TalonFXS=7, P6_CANrange=8, P6_CANdi=9, P6_CANdle=10`. Device id domain 0–62.
   Return `1200` = `SimDeviceNotFound` (device does not exist), `1201` = `SimPhysicsTypeNotSupported`
   (device exists, key unknown), `0` = OK. Java `PlatformJNI.JNI_Sim*`, P5 `CTRE_PhoenixCCISim` (`c_Sim*`) and the C++
   `ctre::phoenix::platform::Sim*` are thin wrappers over the same registry (P5 CCI imports the same `Sim*` functions
   from `PhoenixTools_Sim`), so **Phoenix 5 and 6 share one device registry**.
4. **Device construction → `SimCreate`.** Every `Core*` constructor calls `PlatformJNI.JNI_SimCreate(type,id)`
   (`CoreTalonFX.java:118`, `CoreCANcoder:112`, `CorePigeon2:119`, `CoreCANdle:110`, …) which reaches the C++
   export `?SimCreate@platform@phoenix@ctre@@YAHW4DeviceType@123@H@Z` (the C export `…_sim_create` is *not* on the
   Java path). `DeviceIdentifier` calls `c_ctre_phoenix6_encode_device(id, model, canbusName, &hash)`; this is the only
   place the **CAN bus name** appears (SimCreate and the physics ABI are bus-blind). Hooks on both fired for every
   device (`talon fx 4 @OverCANivore`, `talon fx 24 @rio`, `pigeon 2 13 @OverCANivore`, …).
5. **SimDevices.** The WPI wrapper classes create SimDevices on construction and mirror native state through
   `HAL.registerSimPeriodicBeforeCallback` (outputs, once per robot loop) and value-changed callbacks (inputs →
   `JNI_SimSetPhysicsInput`, synchronous). Names/values (all `double` unless noted; direction as seen by the *simulator*):

   | SimDevice name (`%d` = CAN id) | Input (we write) | Output (we read) |
   |---|---|---|
   | `CANMotor:Talon FX (v6)[%d]` | `supplyVoltage` | `dutyCycle`, `motorVoltage`, `torqueCurrent`, `supplyCurrent` |
   | `CANEncoder:Talon FX (v6)[%d]/Rotor Sensor` | `rawPositionInput`, `velocity`, `acceleration` | `position` |
   | `CANDIO:Talon FX (v6)[%d]/Fwd Limit` / `/Rev Limit` | `value` (bool, bidir) | – |
   | `CANMotor:Talon FXS (v6)[%d]` (+ same sub-devices) | same as FX | same as FX |
   | `CANEncoder:CANcoder (v6)[%d]` | `supplyVoltage`, `rawPositionInput`, `velocity` | `position` |
   | `CANGyro:Pigeon 2 (v6)[%d]` | `supplyVoltage`, `rawYawInput`, `pitch`, `roll`, `angularVelX/Y/Z` | `yaw` |
   | `CAN:CANdle (v6)[%d]` | `supplyVoltage`, `fiveVRail`, `outputCurrent`, `temperature` | `vbatModulation` |
   | `CAN:CANrange (v6)[%d]` | `supplyVoltage`, `distance` | `isDetected` (bool) |
   | `CAN:CANdi (v6)[%d]` (+ `CANDutyCycle:`/`CANEncoder:`/`CANDIO:` children) | per `CANdi.java` | per `CANdi.java` |
   | P5 `CANMotor:Talon SRX[%d]` / `Victor SPX[%d]` | `busVoltage`, `supplyCurrent`, `motorCurrent` (directions not enumerated) | `percentOutput`, `motorOutputLeadVoltage` (directions not enumerated; names from `WPI_TalonSRX` bytecode, device names ✔ enumerated live) |
   | P5 `CANEncoder:Talon SRX[%d]/Quad Encoder`, `…/Analog In`, `…/Pulse Width Input`, `CANDIO:…/Fwd Limit` | `rawPositionInput`, `velocity`, … | `position` |
   | P5 `CANGyro:Pigeon IMU[%d]` | heading inputs | – |

   `HALSIM_EnumerateSimDevices("CAN", …)` on the Shelby robot returned 88 devices at ~10 s.
6. **Config is observable.** `c_ctre_phoenix6_get_configs(ctx=0, "rio", hash, 1.0, &str, false)` returns the device's
   whole serialized config (`"<spn>,s_<value>\n"`; floats are IEEE-754 bit patterns, e.g. `1918,s_1070945649` =
   `SensorToMechanismRatio 1.6667`). Verified SPNs (from `ctre/phoenix6/spns/SpnValue.hpp`): `1901` Inverted,
   `1904` NeutralMode, `1906/1908` Stator/Supply current limit, `1907/1909` limit enable, `1910/1911` peak duty,
   `1912/1913` peak voltage, `1918` SensorToMechanismRatio, `1919` RotorToSensorRatio, `1920` FeedbackSensorSource,
   `1921` FeedbackRemoteSensorID, `987` CANcoder MagnetOffset, `828` CANcoder SensorDirection. The same strings are
   visible to an L3 hook on `c_ctre_phoenix6_set_configs` (decimal `f_` form).
7. **Control requests are visible** to an L3 hook: `c_ctre_phoenix6_RequestControl{VoltageOut,DutyCycleOut,
   PositionVoltage,VelocityVoltage,MotionMagicVoltage,TorqueCurrentFOC,…}(canbus, ecuEncoding, freq, …params…)`
   (signatures in `Control_Interface.h`; `ecuEncoding` = encode_device hash, e.g. id 4 → `0x84`). Not needed for
   physics, only for UI ("Velocity target 50 rps").
8. **Time source.** `PhoenixTools_Sim` links `WINMM` (wall clock); the WPI wrappers use `Timer`/FPGA time only for
   Utils conversion. Open risk R3 below.

## Decision

1. **Primary (M1/M3): L1 SimDevice bridge** in `halsim_wpiemu`. A generic, table-driven `SimDeviceBridge`
   (regex over SimDevice names → `DeviceDescriptor{vendor="ctre", type, id, capabilities}` + value-name bindings).
   Phoenix 6 and Phoenix 5 become data tables on top of it (and REV/NavX could follow with no new interception code).
2. **L2 native supplement** (`bridges/vendor/ctre_native`): lazily binds `CTRE_PhoenixTools_Sim` through
   `GetModuleHandle` / `dlsym(RTLD_DEFAULT)` (never `LoadLibrary` a copy: it must be the module the robot loaded).
   Used for (a) config read-back on discovery, (b) non-SimDevice devices (probe grid, below), (c) optional
   fresh-output fast path, (d) `MotorType`, `Add*` writes.
3. **L3 hooks** only behind an opt-in flag, for bus names / requests. Never required for correctness.
4. **Dropped:** HAL CAN-frame emulation (Phoenix never emits to HAL CAN; confirmed), shim/proxy DLL and
   Tuner-X-protocol emulation are documented fallbacks only (below).

## Exact procedure

### Discovery (L1)
```cpp
// in HALSIM_InitExtension (runs before any robot device exists; callbacks then see every creation)
HALSIM_RegisterSimDeviceCreatedCallback("CAN", ctx, onDevice, /*initialNotify=*/1);
// onDevice(name, ctx, handle): just record {name, handle}; do NOT enumerate values yet (not created at this point)
// later (any thread, e.g. first sim tick after creation): enumerate values once and cache handles
HALSIM_EnumerateSimValues(handle, &dev, [](const char* n, void* p, HAL_SimValueHandle h, int32_t dir, const HAL_Value* v){
    // dir: 0 input (simulator writes), 1 output (robot->sim, we read), 2 bidir
    /* cache by (device, n) */ });
```
Parse names: P6 `^(CAN\w*):(.+) \(v6\)\[(\d+)\](?:/(.+))?$`, P5 `^(CAN\w*):(Talon SRX|Victor SPX|Pigeon IMU)\[(\d+)\](?:/(.+))?$`.
Group `CANMotor` + its `CANEncoder …/Rotor Sensor` + `CANDIO …/Fwd|Rev Limit` children under one `DeviceDescriptor`.

### Per tick (lock-step with the robot loop)
* `HALSIM_RegisterSimPeriodicAfterCallback` (robot thread, after the robot's periodic code): read output values,
  send `ActuatorFrame`; apply the `SensorFrame` returned by the emulator (physics already stepped).
* Write inputs with `HAL_SetSimValue(handle, &HAL_Value{ .data.v_double = x, .type = HAL_DOUBLE })`.
  Phoenix's own value-changed callback forwards it to the native sim immediately (works from any thread; verified
  from our extension thread).
* **Write absolute physical truth** each tick (`rawPositionInput` = rotor rotations, `velocity` = rotor rps,
  `rawYawInput` = true heading). Robot calls such as `setPosition(0)` / `setYaw(0)` are offsets inside the Phoenix
  firmware sim exactly like hardware; do not "fight" them. (Native `AddRotorPosition/AddPosition/AddYaw` available in L2.)
* Outputs in L1 refresh once per robot loop (Phoenix's `onPeriodic` in `HAL_SimPeriodicBefore`), i.e. up to one loop
  (20 ms) older than a native read. Use L2 `get_physics_value` for a zero-latency read when tighter fidelity is wanted.
* Battery model: write `supplyVoltage` on every device from the emulator's battery sag (accepted, 10.5 V test).
* Motor physics: apply `motorVoltage` (all modes; the Phoenix firmware already ran PID/MotionMagic against our
  rotor feedback). `torqueCurrent` is also available for TorqueCurrentFOC users. Gear ratios live in robot config
  (`SensorToMechanismRatio`, `RotorToSensorRatio`), i.e. we drive the **rotor** and the robot code sees mechanism units.

### L2 native (resolve once; Windows names, same names on Linux/macOS with `lib…so/.dylib`)
```cpp
auto tools = GetModuleHandleA("CTRE_PhoenixTools_Sim.dll");           // dlopen(.., RTLD_NOLOAD) on POSIX
auto getp  = (int32_t(*)(int,int,const char*,double*))GetProcAddress(tools,"c_ctre_phoenix6_platform_sim_get_physics_value");
auto setp  = (int32_t(*)(int,int,const char*,double))  GetProcAddress(tools,"c_ctre_phoenix6_platform_sim_set_physics_input");
double v; int r = getp(/*P6_TalonFX*/4, 25, "MotorVoltage", &v);     // r==0 ok, 1200 no such device, 1201 key unsupported
setp(4, 25, "RotorVelocity", 50.0); setp(4, 25, "AddRotorPosition", 50.0*0.02);
```
Physics keys verified/seen per device (`SimState` sources + DLL strings): TalonFX `MotorVoltage TorqueCurrent SupplyCurrent
SupplyVoltage RawRotorPosition AddRotorPosition RotorVelocity RotorAcceleration ForwardLimit ReverseLimit MotorType(0=X60,1=X44)`;
CANcoder `SupplyVoltage RawPosition AddPosition Velocity MagnetHealth`; Pigeon2 `SupplyVoltage RawYaw AddYaw Pitch Roll
AngularVelocityX/Y/Z`; CANdle `SupplyVoltage FiveVRail OutputCurrent Temperature` (+get `VBatModulation`); P5 SRX/Victor get
`MotorOutputLeadVoltage`, set `BusVoltage QuadEncRawPos QuadEncAddPos QuadEncVel LimitFwd LimitRev AnalogPos PulseWidth*`;
P5 Pigeon `HeadingRaw HeadingAdd`. Existence probe grid (no hooks): `getp(type,id,"SupplyVoltage"/"MotorOutputLeadVoltage")`
over `type 0..10 × id 0..62` every ~250 ms; `ret != 1200` ⇒ exists. Config: `c_ctre_phoenix6_get_configs(0,"rio",hash,1.0,&s,false)`
with `hash` from `c_ctre_phoenix6_encode_device(id, model, bus, &hash)` (`"talon fx"`, `"cancoder"`, `"pigeon 2"`, `"candle"`, `"canrange"`).

### L3 hooks (optional)
MinHook (BSD-2) inline hooks installed on `CTRE_PhoenixTools_Sim` exports (works because the JNI/C++ entry points
funnel through them): `c_ctre_phoenix6_encode_device(id, model, bus, &hash)` → bus name,
`?SimCreate@platform@phoenix@ctre@@YAHW4DeviceType@123@H@Z` (Itanium `_ZN4ctre7phoenix8platform9SimCreateENS1_10DeviceTypeEi`)
→ creation event for every type incl. plain P5 classes, `c_ctre_phoenix6_RequestControl*` → intent.
**Must be installed before the first device is constructed**: pre-load `CTRE_PhoenixTools_Sim` from the launcher-supplied
absolute path (the JVM's later `System.loadLibrary` resolves to the same module) or use `LdrRegisterDllNotification`.
Polling for the module at 2 ms worked in the Java test but is racy (a Phoenix 5 run lost the race and saw no `SimCreate`).

## Per-device capability table

Legend: ✔ proven in this spike · ◐ supported by CTRE API, not exercised · ✖ not exposed by CTRE sim

| Device | Detect | Read outputs | Write sim state | Config / bus |
|---|---|---|---|---|
| **TalonFX** (X60/X44) | ✔ L1 `CANMotor:Talon FX (v6)`; ✔ L2 probe | ✔ motorVoltage, dutyCycle, torqueCurrent, supplyCurrent (closed-loop already resolved by firmware sim) | ✔ rotor pos/vel/accel, supply V, limit switches (✔ SimDevice), `MotorType` ◐ L2, `Add*` ◐ L2 | ✔ L2 `get_configs` (ratios, invert, neutral, current limits, remote sensor id → FusedCANcoder link); bus ✔ L3 |
| **TalonFXS** | ◐ same pattern (`Talon FXS (v6)`) | ◐ | ◐ | ◐ L2 |
| **CANcoder** | ✔ L1/L2 | ✖ (input device) | ✔ position/velocity, supply V; `MagnetHealth` ◐ L2 | ✔ L2 (offset, direction) |
| **Pigeon2** | ✔ L1/L2 | ✖ | ✔ yaw (raw), pitch, roll, angular rates X/Y/Z, supply V | ◐ mount pose etc. L2 |
| **CANdle** | ◐ L1 `CAN:CANdle (v6)` | ◐ `vbatModulation`; LED colours/animations only via L3 `RequestControl{SolidColor,*Animation}` hooks | ◐ supplyV, 5 V rail, current, temp | ✖ |
| **CANrange / CANdi** | ◐ (CANrange present in Shelby, detected ✔) | – | ◐ distance / detected, DIO/quadrature | – |
| **CANivore bus** | n/a – Phoenix sim has one virtual bus (`sim`); **name only via L3** (`encode_device`). `CANBus.getStatus()` returns simulated status | – | – | any bus string accepted (`"rio"`, `"OverCANivore"`) |
| **Talon SRX / Victor SPX (P5)** | ✔ L1 for `WPI_TalonSRX/VictorSPX` (names above); ✔ L2 for plain `TalonSRX/VictorSPX` (SimCreate hooked, `MotorOutputLeadVoltage` read) | ✔ percentOutput / leadVoltage | ✔ quad encoder pos/vel (robot saw `1234`), bus V, limits, analog, pulse-width | ✖ (no get_configs equivalent proven) |
| **Pigeon IMU (P5)** | ✔ L1 `WPI_PigeonIMU` (`CANGyro:Pigeon IMU`); SimCreate seen | – | ◐ `HeadingRaw` | – |
| **Fault flags, Pigeon2 acceleration** | – | – | ✖ CTRE sim API has no setters (not in any `*SimState`) | – |

## Cross-platform notes

* Simulation natives exist for `windowsx86-64`, `linuxx86-64`, `linuxarm64`, `osxuniversal` (vendordep
  `simMode: swsim` entries). Hardware-sim (`linuxathena`) is not relevant on desktop.
* Files (verified from the 26.3.0 maven zips): `CTRE_PhoenixTools_Sim.dll`; `libCTRE_PhoenixTools_Sim.so`
  (`linux/x86-64/shared`, `linux/arm64/shared`); `libCTRE_PhoenixTools_Sim.dylib` (`osx/universal/shared`).
  Companion libs: `CTRE_Phoenix6_Sim` (C++ API), `CTRE_Phoenix6_WPISim` (C++ WPI classes – imports
  `HAL_CreateSimDevice/HAL_CreateSimValue`, i.e. SimDevices also exist for C++ robots), `CTRE_PhoenixCCISim` (P5),
  `CTRE_Sim*` device plugins. Linux exports confirmed for the six `Sim*` C++ symbols and the C ABI; the macOS
  export trie was not inspected (not verified).
* **L1 is OS-independent** (only `wpiHal`). L2/L3 need per-OS symbol/loader code (`GetModuleHandle`↔`dlopen(RTLD_NOLOAD)`;
  inline hooks: MinHook on Windows x64, funchook/Dobby on Linux & macOS arm64 – hardened-runtime JVMs may refuse code
  patching → L3 may be unavailable on macOS; acceptable because it is optional).
* Java robots load natives from `java.library.path`; Windows needs the same directory on `PATH` for P5 CCI
  dependency resolution (observed). The launcher reproduces this from `simulateExternalJavaRelease`'s
  `libraryDir` (ADR 0001).
* Phoenix writes per-device persistent state to `<cwd>/ctre_sim/*.dat` (the `ctre_sim` folder in the user's repo) and
  listens on TCP 1250 (Tuner X). The robot must run with cwd = project (deploy dir), so the folder lands in the project;
  document it and add it to `.gitignore` guidance; a second concurrent sim would hit the port-1250 conflict.

## Fallbacks (ranked)

1. L2 alone (if SimDevices disappear/rename in a future Phoenix): same procedure, discovery via probe grid.
2. L3 hooks for discovery/bus names.
3. Java-level: `-javaagent`/reflection calling `com.ctre.phoenix6.jni.PlatformJNI.JNI_Sim*` (public statics) – Java robots only.
4. **Shim/proxy DLL** named `CTRE_PhoenixTools_Sim` forwarding all exports (≈640 symbols) – heavy, breaks on every CTRE
   release, EULA-sensitive; last resort.
5. Re-implement the Tuner X diagnostics protocol (port 1250) – undocumented, not recommended.
6. HAL CAN-frame emulation – **not viable** (Phoenix does not use HAL CAN in sim).

## Risks / open items

* **R1 Non-wrapper classes.** Robots using `CoreTalonFX`/plain P5 `TalonSRX` have no SimDevices → covered only by L2
  (probe grid ≤250 ms latency, no bus name unless L3). M3 acceptance test needed.
* **R2 One-loop output latency in L1** (see above). Mitigation: L2 fast-path read.
* **R3 Time base.** Firmware-sim integrators (ramp rates, Motion Magic, PID D-term) appear to use CTRE's own clock
  (wall-clock `WINMM`), not HAL sim time. If the emulator pauses or steps `HALSIM` time faster/slower than real time,
  Phoenix timing may diverge from the robot's. **Test in M1** (`HALSIM_StepTiming` at 0.5×/2×); default to real-time
  stepping for Phoenix robots until proven.
* **R4 Robots that already run their own sim** (the Shelby project bundles maple-sim / OvertureLib; devices showed
  `supplyVoltage` 13.5 V on the CANivore group vs 12 V on rio, i.e. someone else is writing inputs): two writers →
  last-writer-wins. Detect via `HALSIM_RegisterSimValueChangedCallback` on inputs (foreign writes) and warn;
  provide a "emulator owns device" toggle.
* **R5 Id collisions across buses.** Phoenix sim is keyed by `(type,id)` only; same id on `rio` and a CANivore is one sim
  device (also in Phoenix's own sim). Emulator keys devices by `(type,id)` (+ optional bus label from L3); warn on duplicates seen by L3.
* **R6 Version drift.** SimDevice names/values and the C ABI are stable across the 2024–2026 releases we inspected but
  are not formally frozen; keep names in data tables, add an integration test per CTRE release.
* **R7 License (brief, not legal advice).** `CTRE_LICENSE.txt` forbids redistributing the Software, reverse engineering
  or modifying it. Mitigations: never bundle CTRE binaries (use the project's own GradleRIO-resolved libs); L1/L2 call only
  documented/public header entry points and WPILib's public SimDevice API; L3 (runtime inline patching) is the greyest
  area → opt-in, disabled by default, flagged in docs.
* **R9 Pigeon2 yaw interplay.** With absolute `RawYaw` writes every 20 ms the robot-visible yaw alternated between the written heading and ~0 over a 4 s window (the robot also calls yaw resets; SimDevice run showed yaw ≈ 0 while `rawYawInput` = 45). Semantics of `setYaw` vs raw writes (offset vs state overwrite) must be characterised in M3 before wiring the IMU; `AddYaw` (delta) is the fallback. Same check for `setPosition(0)` on TalonFX/CANcoder.
* **R8 Not verified:** C++ (`simulateNative`) robot run, Linux/macOS runs, `CANdi`/`TalonFXS` live devices, P5 plain-class
  SimDevice absence (inferred), macOS symbol export names.

## Impact on the plan

* **Spike B closed – gating risk retired.** No re-plan of M3 scope beyond below.
* **M0.5 SDK:** `IDeviceBridge` gets a generic `SimDeviceBridge` base (name-regex → descriptor, value-name bindings, direction-aware
  I/O) shared by Phoenix 6, Phoenix 5 and future vendors; `DeviceDescriptor` gains optional `busLabel`, `config` (key→value) and
  `drivenBy` (emulator | robot-sim) fields.
* **M1:** extension registers `HALSIM_RegisterSimDeviceCreatedCallback` at init; exchange point = `HALSIM_RegisterSimPeriodicAfterCallback`
  (lock-step with the robot loop); battery model writes `supplyVoltage`; add the R3 time-base test.
* **M3:** Phoenix 6 work = data tables + L2 module (config read-back, probe grid, optional fast path) + P5 tables. CANivore support
  = bus *label* only (L3, optional) – no CAN emulation. Fixtures = JSON dumps of SimDevice trees (no CAN traces). Remove "CAN frame
  emulation" from the plan. Per-device unit tests can be made hardware-free by faking the HALSIM SimDevice API.
* **M5 UI:** auto-fill mechanism gear ratio / invert / fused-CANcoder mapping from L2 config; show control-mode labels when L3 is on.
* **Effort:** M3 Phoenix scope shrinks (≈ −1 sprint of CAN/proxy work) at the cost of new tests for R1/R3/R4.

