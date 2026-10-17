# WPI Emulator user guide

WPI Emulator runs your **unmodified** WPILib robot project (Java or C++) against a simulated robot and field.
Your code is started through Gradle exactly as `./gradlew simulateJava` would, with a small HALSim extension added.
The extension hands the motors, encoders and sensors your code creates to the emulator app, which owns the physics
(Box3D), the mechanism models and the 3D view.

This guide describes what is implemented in this repository. Where something is not verified or not supported it
says so (see [Limitations](#8-limitations)).

* [1. Install](#1-install)
* [2. First run: the setup wizard](#2-first-run-the-setup-wizard)
* [3. The editor](#3-the-editor)
* [4. 2026 game configuration (fuel, intake, shooter)](#4-2026-game-configuration-fuel-intake-shooter)
* [5. The run view and the driver station](#5-the-run-view-and-the-driver-station)
* [6. Settings and where data is stored](#6-settings-and-where-data-is-stored)
* [7. Troubleshooting](#7-troubleshooting)
* [8. Limitations](#8-limitations)

## 1. Install

**What you need**

| Need | Notes |
|---|---|
| A WPILib robot project | Must apply the GradleRIO plugin and contain a Gradle wrapper (`gradlew` / `gradlew.bat`). Open it once in VS Code with the WPILib extension if the wrapper is missing. |
| WPILib 2026 | Installed in `C:\Users\Public\wpilib\2026` (Windows) or `~/wpilib/2026`. It provides the JDK the emulator uses to run Gradle. Any JDK 17+ found through `JAVA_HOME` also works. |
| C++ projects only | A C++ compiler: Visual Studio 2022 (or Build Tools) with the "Desktop development with C++" workload on Windows, Xcode command line tools on macOS, `g++`/`clang++` on Linux. |
| A GPU/driver with OpenGL or Direct3D support | The 3D view uses Qt Quick 3D. |

**Windows (the only platform verified end to end)**: run `wpi-emulator-<version>-windows-x64.exe` (NSIS installer;
installs to `C:\Program Files\WPI Emulator`) or unzip the portable `...-windows-x64.zip` and start `wpiemu_app.exe`.
**macOS** (`.dmg`) and **Linux** (`.AppImage`) packages are produced by scripts and CI but have **not been run
locally**; see [packaging.md](packaging.md). To build from source see [BUILDING.md](BUILDING.md).

Check the install from a terminal: `wpiemu_app --version` prints the SDK version, IPC protocol version and the number of
game plugins (this does not open a window).

**Command line**

```
wpiemu_app [--demo] [--project <folder>] [--version]
```

* `--project <folder>` opens a project on start (otherwise the last project is reopened, unless disabled in Settings).
* `--demo` uses simulated devices and scripted chassis motion instead of launching any robot code (a "Demo mode" chip
  is shown in the toolbar). Handy to look around the UI without a project.

## 2. First run: the setup wizard

Start the app and either drop a project folder on the welcome page or click **Open WPILib project...**
(`Ctrl+O`). A project that has no saved robot setup opens the **Set up your robot** wizard. You can rerun it any time
with **File > Set up robot...**; the sidebar says *"Nothing is changed until you open the editor."*

### Step 1 - Project
Drop the project folder on the page, paste or **Browse...** for its path, or pick a recent project. The app reads
`build.gradle` (and `.wpilib/wpilib_preferences.json`) and shows:

| Row | Meaning |
|---|---|
| Season | WPILib year and GradleRIO version, e.g. `2026 (GradleRIO 2026.2.1)` |
| Language | Java or C++ |
| Team number | from `.wpilib/wpilib_preferences.json` |
| Vendor libraries | from `vendordeps/` |
| Robot setup | *New* or *Already configured - finishing will update it and keep your mechanisms and device mappings* |

Errors (red banner): the folder does not exist, no `build.gradle`, or GradleRIO is not applied (not a WPILib project).
Warnings (yellow): GradleRIO version unreadable (season unknown), Java/C++ unclear, no Gradle wrapper.

### Step 2 - Toolchain
The app checks your machine; each row shows what it found and a "How to fix" hint when something is missing.

| Check | Looks for |
|---|---|
| Java (JDK 17+) | the JDK folder from Settings, `JAVA_HOME`, then WPILib's bundled `jdk` |
| Gradle wrapper | `gradlew.bat` / `gradlew` in the project |
| WPILib `<year>` installation | `C:\Users\Public\wpilib\<year>` / `~/wpilib/<year>` (optional if a JDK is found elsewhere) |
| C++ toolchain | only for C++ projects |
| Game module | whether the project's season has a built-in game plugin (only **2026** in v1) |

Use **Re-check** after fixing something. A missing item does not block the wizard (*"You can finish the setup now and
fix it before running"*). For a season without a game module a **Core-only mode** banner appears: the robot,
drivetrain and mechanisms still simulate, but on an empty field with no game pieces or scoring.

### Step 3 - Robot
Describe the drivetrain; everything can be refined in the editor later.

| Card | Fields |
|---|---|
| Drivetrain | Robot name; **Type**: Swerve / Tank / Mecanum |
| Size and weight | Frame length (front to back), frame width, robot mass, bumper thickness, wheel diameter, wheels per side (tank), total wheel count |
| Motors and gearing | Drive motor, motors per wheel (per powered side for tank), drive gear ratio; steer motor and ratio (swerve) |
| Alliance | Bumper colour Red / Blue; your driver station 1-3 |

A schematic of the robot updates as you type. Invalid values are highlighted and block **Next**.

### Step 4 - Summary
Shows project, game module, toolchain status, robot and where it is saved. The tick box **Detect devices now** runs
your robot code once so the editor can list its motors and sensors (disabled until the toolchain check passes;
you can also detect later from the editor). **Open editor** saves the setup and switches to the editor.

Saved to: the app data folder, plus a shareable copy in the project at `.wpiemu/robot.json` (see
[section 6](#6-settings-and-where-data-is-stored)). Re-running the wizard keeps your mechanisms, device mappings and
game settings.

## 3. The editor

### Linking wheels and mechanisms

The inspector's **Motion links** section connects a **Source** axis to a **Follower**.
Set the signed **Ratio**, then **Add / update link**. Ratio 2 turns a rotary follower twice
per source revolution; -1 reverses it. Mixed rotary/linear links show explicit **m / rev**
or **rev / m** units. Select a link to update/remove it, or choose **New link** to add another.
Changes support Undo/Redo.

New tank setups link all wheels on a side to one powered wheel. **Link tank wheels by side**
prefers the wheel already mapped to that side's controller. Map only the two source wheels:
one controller can then move all three wheels on its side. Existing ordinary tank files upgrade
automatically, preserving mappings; save the upgraded configuration. **Use independent wheels**
opts out. Ambiguous layouts or several mapped wheels per side need correction before conversion.

Links also work between supported mechanism axes: `axis` for ordinary mechanisms, and
`deploy`/`roller` for intakes. Followers need no motor mapping; encoders and limit sensors
can still be mapped. Remove a follower's independent motor mapping before linking it.
One source can drive several followers, including chains, but cycles are not allowed.

This is deliberately simple motion following. Followers retain their starting positions,
stop at their own travel limits, and update visuals/sensors, but do not load or stop the source.
It is not an accurate gearbox, torque-sharing, or energy-conservation simulation.

### Editor layout

The **Editor** and **Run** tabs centred in the menu bar row (`Ctrl+1` / `Ctrl+2`) switch between the editing page and the run page.

```
+---------------------+-----------------------------------+------------------+
| Project tree        |  3D viewport                      | Inspector        |
| Mechanism palette   |  (Front / Side / Top / 3D)        | (properties,     |
| Detected devices    |-----------------------------------|  device slots)   |
|                     |  Console | Validation | Telemetry |                  |
+---------------------+-----------------------------------+------------------+
```

### Viewport
Left-click selects (click a mechanism again to drill into one of its shapes). Right-drag (or Alt+left-drag) orbits, left-drag on empty space or middle-drag pans, the
wheel zooms. `W` / `E` / `R` pick move / rotate / scale for the gizmo, `F` frames the selection, `1`-`4` pick Front /
Side / Top / 3D views, hold `Ctrl` to toggle snapping while dragging, `Del` deletes, `Ctrl+D` duplicates (when the
editor, not the shell, has focus), `Esc` clears the selection. Everything is undoable (`Ctrl+Z` / `Ctrl+Y`);
a whole gizmo drag is one undo step. The toolbar can switch the display units between **in / lb** and **cm / kg**
(the saved model is always SI).

### Saving and switching projects
Changes autosave after a short delay; **File > Save**, the editor's **Save** and `Ctrl+S` save
immediately. Closing the window/project, opening another project or rerunning setup first saves
pending edits to the **old** project. If the authoritative app-data save fails, the transition is
cancelled and your document stays open and unsaved. Check disk space and access to the app-data
location, then retry **Save** before continuing. A failed shareable project export is only a
warning when the app-data copy succeeded.

Undo/redo and reloading update focused inspector fields as well as the model, including every
component of a pose. Ordinary typing keeps its cursor position.

### Chassis
The project tree has a **Chassis** node (hull, centre of mass, drivetrain, wheels and collision shapes). Select it to
get the **Robot overview** in the inspector:

* *Frame, mass, centre of mass*: dimensions, mass, COM position.
* *Type and traction model*: drivetrain kind and its parameters.
* **Wheels (n)**: each wheel has a position, radius and a **drive** (and for swerve **steer**) motor slot; **+ Add
  wheel** / **Delete wheel**.
* **Shapes (n)**: boxes and cylinders (**+ Box**, **+ Cylinder**) that model the visible, colliding parts.

### Mechanisms
The **mechanism palette** lists the templates. Double-click one to add it (to the robot, or to the selected mechanism);
or drag it onto a tree row to attach it there. Core templates (game independent):

| Template | Use |
|---|---|
| Generic roller | spinning roller / conveyor wheel |
| Generic 1-DOF | any single axis (rotary, arm or linear) with stops, limit switches and a beam break |
| Intake | deploy arm (optional) + roller |
| Shooter (flywheel) | flywheel with encoder |
| Hood / pivot / Arm | gravity-loaded arm with stops |
| Turret | yaw axis |
| Elevator / climber | linear axis with gravity |
| Indexer / conveyor | belt with entry/exit beam breaks |

Each template exposes **device slots** (motor, encoder, limit switches, beam breaks...) and properties (motor model
and count, gearing, mass/inertia, stops, friction...) generated from its metadata, so the inspector changes with the
template. **Add attached mechanism** hangs one on another mechanism's named link (for example a roller on an arm);
use **Attach to** / **Attachment link** and **Mount pose** to place it. Mechanisms move with the part they are
attached to. Plant models are documented in [mechanisms.md](mechanisms.md).

### Device mapping
Your robot code creates devices (motor controllers, encoders, gyros, solenoids...) at run time. The simulator needs to
know which mechanism slot each one drives:

1. Entering the **Editor / Run** tab launches your robot project once (the first launch builds it and can take a
   while). The same instance detects devices and later runs the simulation; the *Detected devices* panel shows
   *"Starting robot program..."* until it connects. The robot code is always running, so devices appear
   as your code creates them.
2. The **Detected devices** table lists **Device**, **Type**, **ID**, **Bus** and **Status**
   (`unmapped (loopback)`, `gone`, alias warnings).
3. **Drag a detected device** from the table onto a slot row in the inspector (mechanism slots or a wheel's
   drive/steer slots). Slot rows highlight when the device is compatible; incompatible drops are refused
   (*"Cannot map the device"*). Slot rows can also pick a device from a list, and **Unmap** clears a slot.
4. Slot rows show their state: required and unmapped, optional, mapped, *not detected* (mapped to a device the last
   discovery did not see), *gone*, or a type mismatch.

Restarting robot code invalidates the old live inventory. Devices absent from subsequent complete
discovery snapshots remain listed as `gone`; their saved slot mappings are preserved. Numeric
runtime device IDs being reused by a different device do not transfer physical identity or mappings.

Devices that are detected but not mapped run in **loopback**: outputs are echoed back as inputs, so the code does not
break, but nothing physical happens. Required slots that stay unmapped read neutral values.

The **Validation** tab lists problems (unmapped required slots, duplicate mappings, alias conflicts...) and the
**Console** tab shows discovery and run output. Changes are saved automatically and with `Ctrl+S`.

## 4. 2026 game configuration (fuel, intake, shooter)

For a 2026 project the shell shows a green game chip and the `game_2026` plugin (REBUILT) is active: the field
(hub, bumps, trenches, tower, depots, outposts), fuel physics, hub shift rules and fuel scoring. Details, sources and
simplifications: [game_2026.md](game_2026.md).

**Game rules** (project tree, *game rules* node): three panels.

| Panel | Settings |
|---|---|
| Fuel meter | meter capacity (default 30), fuel preloaded in the robot (default 8) |
| Field staging | neutral-zone fuel (default 360), fuel per depot (24), outpost chute stock (24), human player off / timed and its period |
| Match rules | enforce hub shifts, which hub is inactive first (`auto`/`red`/`blue`), hub processing delay, recycle scored fuel |

**Your own intake and shooter** are built from the normal templates (roller, intake, shooter, indexer, hood, turret)
with their devices mapped as above. Two extra palette entries bind them to the game:

* **Fuel Intake** (category Intake): add it, set the **Pickup zone** (a box shown in green in the viewport; drag its
  handles or edit the numbers), and say which roller/deploy mechanism must be active (`activation`: roller, deployed,
  both, either, always). Fuel inside the zone is absorbed while active and the meter is not full.
* **Fuel Shooter** (category Shooter, link `muzzle`): set the **Muzzle** offset (orange marker), the flywheel mechanism
  (exit speed = angular speed x radius x efficiency), optional hood and turret mechanisms (or attach it to the turret
  link), and the feed mechanism and fire rate. Shots are real physics bodies.

A reference to a mechanism that does not exist logs one warning and leaves that intake/shooter inactive. The
property tables are in [game_2026.md section 5](game_2026.md#5-robot-side-meter-intake-shooter).

## 5. The run view and the driver station

Your robot launches automatically when you enter the Editor/Run tab and stays alive until you close the project or
use **Restart robot code**. Entering the **Run** tab starts the simulation at sim time zero automatically and it keeps running;
there are no start, stop, pause or step controls. The shell shows *Starting robot program...*, *Robot ready*,
*Simulating*, or *Robot stopped / crashed / failed to start*.

The **Driver Station** has four modes: **Auto**, **Teleop**, **Test** (Enable runs until you disable) and
**Practice** (Enable runs autonomous, a short disabled gap, then teleop, on the match clock; durations are in the
*Setup* tab). Enter disables, Space is the emergency stop (as on the real DS). Match timing, hub shifts and the score
and phase widgets only apply in Practice; the other modes are free play.

### Run view

| Area | What it does |
|---|---|
| View controls | Icon buttons at the bottom-right of the 3D view: **Restart** (`Ctrl+R`, puts the robot and game pieces back to the start; the robot code keeps running). Its dropdown adds **Restart robot code** (`Ctrl+Shift+R`, reboots the code and resets everything). **Speed** (relative to real time) |
| Camera | Same camera controls as the editor viewport (right-drag or Alt+left-drag to rotate; left-drag, middle-drag or Shift+drag to pan; wheel to zoom) and **Home view** (⌂ button, last in the bottom-right controls, `Ctrl+0`, or double-click the view) |
| Overlays | **View** menu: game HUD, mechanism values, zones (intake / shooter), individually switchable (Run tab only) |
| Telemetry (right side of the toolbar) | simulated time, ticks per second and speed, battery voltage, physics step time and entity count, gamepad status |
| Console | **View > Console** (`Ctrl+J`); filter by level and source (robot stdout, robot stderr, emulator), search, copy, clear, follow |

In the 2026 HUD you see the fuel meter, match timer and phase, hub active badges, scores, shots fired and fuel on the
field.

### Driver station panel
Built into the run page; it replaces the real Driver Station. It sits at the bottom-left, next to the console. The **cog**
button next to the match clock opens a popup window (Setup and Joysticks tabs) with alliance/station, game message, match durations and joystick bindings.
In **Practice**, the running segment (Auto or Teleop) is highlighted green on the mode buttons while the match runs.

| Control | Behaviour |
|---|---|
| **Enable** (a toggle: click again to disable; `Enter` disables immediately) | the mode can only change while disabled, like the real DS |
| **EMERGENCY STOP** (`Space`) | latches until cleared (*E-STOPPED - click to clear*) or the robot restarts |
| Mode | Teleoperated / Autonomous / Test |
| Alliance and station | Red / Blue, Station 1-3 |
| Game message | free text the robot reads with `getGameSpecificMessage()`; when the game plugin provides FMS data (2026 hub order `R`/`B`) it overrides this text |
| **Start match** | enables the robot and runs Autonomous (default 20 s), a 3 s disabled gap, then Teleop (default 140 s). The match time seen by the robot counts down. The durations are editable. |
| Joysticks (popup) | six ports; each port takes the keyboard or a gamepad. Default keyboard: `W A S D` left stick, arrow keys right stick, `U` / `O` triggers, `Z X C V Q E R F G H N M` buttons 1-12, `I J K L` POV. Rebind by clicking a binding and pressing the key or moving the control (`Esc` cancels). |

Gamepads need SDL2 in the build (a build without it still works with the keyboard; the Joysticks tab then says that
gamepad support is not available).

## 6. Settings and where data is stored

**File > Settings...**: *Appearance* (theme System / Light / Dark; units in dialogs Imperial or Metric),
*Toolchain* (WPILib folder and JDK folder; empty = detect automatically; applies next time the toolchain is
checked), *Simulation* (physics rate 100 / 200 default / 250 / 500 / 1000 Hz; the value is written into the robot setup when the
setup wizard finishes, so rerun **Set up robot...** after changing it - the dialog's "applies to the next robot run" is
currently only true after that),
*General* (reopen the last project on start).

| What | Where |
|---|---|
| Robot setup (working copy) | `%APPDATA%\WpiEmulator\projects\<folder>-<hash>\robot.json`; macOS `~/Library/Application Support/WpiEmulator`; Linux `$XDG_DATA_HOME/wpi-emulator` or `~/.local/share/wpi-emulator` |
| Shareable copy | `<project>\.wpiemu\robot.json` (commit it for your team if you want; add `.wpiemu/` to `.gitignore` if not) |
| Older-format backups | `robot.json.v<N>.bak` next to the file, made before migrating |
| Window and UI preferences | Qt settings of the app (registry on Windows) |

Environment variables for advanced use: `WPIEMU_LAUNCH_MODE=direct` (run the project's `simulateExternal*` task and start
the robot directly instead of the Gradle init script), `WPIEMU_GRADLE_OFFLINE=1`, `WPIEMU_EXTENSION` /
`WPIEMU_EXTENSION_DIR` (location of the HALSim extension), `WPIEMU_DISABLE=1` (set inside the robot process to make the
extension inert).

## 7. Troubleshooting

| Symptom | What to check |
|---|---|
| **Robot won't start** ("Robot failed to start") | Open the console (`Ctrl+J`). Typical causes: the project has no Gradle wrapper (*"This project has no Gradle wrapper"*); the build fails (read the Gradle output in the console and fix the project; build once from VS Code to prove it compiles); no JDK/WPILib found (wizard step 2 or Settings > Toolchain); the HALSim extension `halsim_wpiemu` was not found next to the app (*"Reinstall the emulator or set WPIEMU_EXTENSION"*); the robot configuration has validation errors (fix them in the Validation tab); the first build is simply slow, wait for it. For C++ projects make sure the compiler of step 2 is installed. |
| **"Robot crashed"** | The process ended with a non-zero exit code. The console tail shows the stack trace or the Java exception from your own code. |
| **No devices detected** | Devices are listed only when the robot code *creates* them, so the code must reach its constructors (`robotInit`/`RobotContainer`). Give the code a moment to build your subsystems are built; check the console for exceptions that abort startup. Vendor libraries other than CTRE (e.g. REV) are not bridged in v1, so only their raw CAN traffic is seen (see [Limitations](#8-limitations)). Make sure the project's season is 2026 and that detection actually started (status *Detecting devices...*). |
| **Handshake timeout** ("The simulator extension did not connect") | The robot process runs but the HALSim extension never said hello (timeout 5 minutes). Look for an extension load error in the console (wrong WPILib year, antivirus blocking the DLL or the named pipe), and make sure the project actually ran the simulation task and simulation is enabled. Reinstall the emulator so the app and `halsim_wpiemu` match. |
| **Protocol mismatch** | "Incompatible simulator extension": the extension and the app are from different builds. Reinstall / rebuild both. |
| **"Robot not responding"** | The lock-step watchdog expired: your code is blocked or in an infinite loop. Use **Restart robot code**. |
| **Unsupported WPILib year** | v1 ships a game module only for **2026**. For other seasons the wizard shows *Core-only mode* and the game chip is yellow: chassis, drivetrain and mechanisms simulate on an empty field. The HALSim extension is built and validated against WPILib 2026.2.1 only; other years may fail to load it. If the year shows as *Unknown*, the GradleRIO version in `build.gradle` could not be read (use a literal `id "edu.wpi.first.GradleRIO" version "2026.2.1"`). |
| **Phoenix 6 notes** | Devices are seen through WPILib's SimDevice layer; the emulator drives the **rotor**, so Phoenix applies its own `SensorToMechanismRatio` and offsets (set your real gear ratios in the robot code, not in the emulator's slot). TalonFX/TalonFXS, CANcoder, Pigeon 2, CANdle, CANrange and CANdi are supported; temperature and faults are not simulated; Pigeon 2 has no acceleration input in CTRE's simulation. Phoenix 5 is included but derived from documentation, not tested against the real library. Do not drive a Phoenix device's sim state from your own robot code at the same time as the emulator (two writers); see [bridges-vendor.md](bridges-vendor.md) for the details. |
| **CANivore bus labels** | Phoenix simulation keys devices by type and CAN ID only, so the **Bus** column in the detected-devices table is blank/unknown: the app cannot tell a CANivore from the roboRIO bus (and never invents a label). The same type + ID on two buses is the same simulated device; mapping it twice shows an *alias conflict*, and the conflicting device runs unmapped until fixed. |
| **Mechanism does not move** | Check the Validation tab (required motor slot unmapped?) and that the mapped device is the one your code drives. A mechanism only moves when its motor slot is mapped. |
| **Config file could not be read** | *"The saved robot setup could not be read... set the robot up again"*: rerun the wizard. |
| **Nothing shows in the 3D view** | Update the graphics driver; Qt Quick 3D needs a working OpenGL/Direct3D device. The app also runs headless for tests only. |

## 8. Limitations

* **Suspension** is not simulated: the chassis is a rigid body on a flat floor with a tire model (bumps are climbed as
  ramps).
* **One robot** per session. Other robots on the field, opponent play and multi-robot game logic are out of scope.
* **No vision**: cameras, AprilTag/PhotonVision/Limelight pipelines are not simulated.
* **REV Robotics** (SPARK MAX / FLEX, PH) is not bridged in v1; only CTRE Phoenix 6 (and Phoenix 5, unverified) are
  mapped to mechanisms. Generic HAL devices (PWM, DIO, analog, encoders, relays, solenoids, PCM/PH state, power
  distribution) are bridged. The vendor recipe is in [CONTRIBUTING.md](CONTRIBUTING.md).
* **Game coverage**: only the 2026 plugin ships. Tower points, ranking points, penalties and G-rules are not scored;
  the 2026 field is approximate (see [game_2026.md](game_2026.md) section 10).
* **Real driver station**: not connected; use the built-in panel.
* **Platforms**: Windows is built, tested and packaged here. **Linux and macOS builds and packages have not been run
  locally** (CI is configured; expect first-run fixes). On Linux/macOS the extension's interaction with the robot's own
  `libwpiHal` is an open item of [ADR 0001](decisions/0001-halsim-extension-injection.md).
* **Plugins** are compiled in (static); there is no runtime plugin loading.
* **Season**: validated against WPILib/GradleRIO 2026.2.1.
