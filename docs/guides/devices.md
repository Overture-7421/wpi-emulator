# Add a device adapter

Read this when robot code creates a device that the emulator does not discover or cannot map.
Prerequisites: [Architecture](../ARCHITECTURE.md), [a development build](../BUILDING.md), and the
[bridge contract](../reference/bridges.md). That reference owns identity, units and feedback semantics.

A bridge runs inside the **robot process**. It translates a backend's device names and values into
SDK descriptors and canonical signals; it does not simulate a mechanism or choose its motor physics.
For physical motor data, use [the motor guide](motors.md).

## Choose the smallest change

| What the backend exposes | Start here | Working implementation |
|---|---|---|
| Another model with existing SimDevice semantics | A rule in the existing vendor module | [CTRE rules](../../bridges/vendor/src/ctre_rules.cpp) |
| A new indexed HAL family | `ChannelFamilyBridge`; the HAL module supplies `HalFamilyBridge` and `IndexedHalBridge` plumbing | [HAL sources](../../bridges/hal/src) |
| A genuinely different API/lifecycle | Implement `IDeviceBridge`, in an existing module where possible | [SDK interface](../../sdk/include/wpiemu/sdk/devices/device_bridge.h) |
| A camera publishing coprocessor | `IVisionBridge`, not a motor/device rule | [Vision guide](vision.md) |

Before writing a rule, inspect the vendor wrapper's simulation source or enumerate the actual
SimDevices. Record exact names, types, directions, units, children, and which values appear late.
A hardware API name is not evidence of a SimDevice name.

## Trace one device end to end

Use the existing TalonFX fixture before adapting a new wrapper. This is an exact excerpt from
[`test_vendor_rules.cpp`](../../tests/bridges_vendor/test_vendor_rules.cpp), inside
`vendor rules: TalonFX: tree grouped into one device with canonical signals and directions`.
It uses that file's `Rig`, `ctre` alias and `addTalonFx` fixture; it is not a standalone program.

```cpp
Rig rig(ctre::phoenix6Rules());
const auto t = addTalonFx(rig.backend, 4);
rig.start();
rig.bridge->poll();
```

Expected result: one announcement groups the motor, rotor and limit children. The same test checks
the descriptor, capabilities, each direction, collected voltages and applied sensor values.
Treat those assertions as a checklist for your adapter, not just a discovery smoke test.

1. Give the rule a full-match regex, stable family/type/vendor, and the correct ID/child groups.
   `SimDeviceRule` and `SimValueBinding` are in
   [`sim_device_bridge.h`](../../sdk/include/wpiemu/sdk/devices/sim_device_bridge.h).
2. Bind only real values to [canonical signals](../../sdk/include/wpiemu/sdk/devices/signals.h).
   “Output” means robot-to-emulator; “Input” means emulator-to-robot. Supply explicit conversion
   scale/offset where native units differ. Do not write a firmware-reported output as feedback.
3. Announce capabilities that the bindings actually support. Mapping compatibility is based on
   capabilities, not a friendly model name.
4. If names do not expose a physical CAN bus, set `busIsStatic = false`. The CTRE `base` helper
   demonstrates this. A configured CAN address cannot disambiguate two devices the backend aliases.
5. Exercise late values, retirement and rediscovery. Shutdown must be idempotent; input history
   must clear at reset without erasing robot-owned calibration.

## Register a vendor rule set

The existing vendor module owns backend policy in
[`vendor_bridges.cpp`](../../bridges/vendor/src/vendor_bridges.cpp). Add the new table to its shared
setup and register a factory through `DeviceBridgeRegistry::add`; propagate registration failures.
Add any new source explicitly in
[`bridges/vendor/CMakeLists.txt`](../../bridges/vendor/CMakeLists.txt).

Reuse `VendorSimDeviceBridge` and the backend decorators. `GuardedValueSpec` is for verified
firmware-integrated input semantics, not a general way to suppress feedback changes. CTRE-native
config read-back and hooks are CTRE-specific; do not enable them for an unrelated vendor.

## Wire a new module only when necessary

1. Put public headers below the module's `include/wpiemu/bridges` tree; keep backend headers private.
2. Declare its sources/include root in that module's CMake file, with SDK as the public dependency.
3. In [`bridges/CMakeLists.txt`](../../bridges/CMakeLists.txt), add its subdirectory and link its
   library into `wpiemu_bridges_builtin`.
4. Include its public registration header and call the registration function in
   [`bridges/builtin_bridges.cpp`](../../bridges/builtin_bridges.cpp), preserving the first error.
   Do not change the extension to include or link the module directly.
5. Update `ALLOWED`, `MODULE_OWNERS` and `TARGET_OWNERS` in
   [`check_boundaries.py`](../../tools/check_boundaries.py) for the module and allowed aggregator edge.
6. Add tests under `tests` and explicitly link the new library in
   [`tests/CMakeLists.txt`](../../tests/CMakeLists.txt). Source discovery does not add library links.
   Reconfigure, build, then run the new tests; see [Building](../BUILDING.md).

## Validate and diagnose

Run from the repository root with CTest on PATH:

```powershell
ctest --preset windows-ninja-release -R "^(vendor rules:|vendor bridge:|ChannelFamilyBridge)"
```

The excerpt above is exercised by this selection. Extend
[`vendor fakes`](../../tests/bridges_vendor/fakes.h) with wrapper-faithful fixtures and assert:
discovery, grouping, signal direction/scale, missing values, retirement, unmapped loopback,
reset and duplicate registration.

If discovery works but mapping fails, compare advertised capabilities to the requested slot.
If a motor moves but its encoder drifts, trace raw input versus firmware-reported output and
gearbox units before adding compensation. Preserve the single mapping-inversion owner.
Fake tests verify translation; they do not prove a real vendor wrapper exports those values.
Use a real robot project when introducing wrapper support.

Next: [configure or extend a mechanism](mechanisms.md).
