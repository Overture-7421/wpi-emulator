# WPI Emulator user guide

WPI Emulator runs your **unmodified** WPILib robot project (Java or C++) against a simulated robot and field.
Your code is started through Gradle exactly as `./gradlew simulateJava` would, with a small HALSim extension added.
The extension hands the motors, encoders and sensors your code creates to the emulator app, which owns the physics
(Box3D), the mechanism models and the 3D view.

This guide describes what is implemented in this repository. Where something is not verified or not supported it
says so (see [Limitations](#8-limitations)).

* [1. Install](#1-install)
* [2. First run: the setup wizard](#2-first-run-the-setup-wizard)
* [3. The editor](#3-the-editor) (including *Cameras*: simulated Limelight / PhotonVision)
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

Parts that are mechanically linked share one motor: a tank drive with three wheels per side
needs one motor per side, not six. Select the wheel or mechanism that should **follow** and use
the **Drive** card of its inspector: choose **Linked to another part**, pick the source (a wheel or a mechanism
axis) and set the signed **Ratio**. Ratio 2 turns a rotary follower twice per source revolution;
-1 reverses it. Mixed rotary/linear links show explicit **m / rev** or **rev / m** units.
Choose **Own motors** to unlink. Linking a part drops its motor mapping in the same step;
everything supports Undo/Redo.

In the project tree a follower is nested **under the part that drives it**, dimmed and read-only:
it shows `follows <source> ×ratio`, has no motor slot and cannot take a dropped device. The
source shows `drives n`. A mechanism keeps its attachment (shown as `@ parent` on its row and
editable in its inspector), even though the tree places it under its source. Encoders and limit
sensors of a follower can still be mapped.

Link a tank side wheel by wheel: select each follower, choose **Linked to another part** and pick the powered
wheel of its side; then map controllers only to the source wheels. Wizard-made tank robots start linked.

Links also work between supported mechanism axes: `axis` of a mechanism (or of the plugin's own wrappers). One source can drive several followers, including chains, but
cycles are not allowed.

This is deliberately simple motion following. Followers retain their starting positions,
stop at their own travel limits, and update visuals/sensors, but do not load or stop the source.
It is not an accurate gearbox, torque-sharing, or energy-conservation simulation.
### Editor layout

The **Editor** and **Run** tabs centred in the menu bar row (`Ctrl+1` / `Ctrl+2`) switch between the editing page and the run page.

```
+---------------------+-----------------------------------+------------------+
| Project tree        |  3D viewport                      | Inspector        |
| Detected devices    |  (Front / Side / Top / 3D)        | (Name, Drive,    |
|                     |-----------------------------------|  Configure ...)  |
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
The project tree has a **Chassis** node (hull) with its **Centre of mass**, collision shapes and a **Drivetrain** node nested inside it;
the wheels are listed under Drivetrain. Chassis starts collapsed (and Drivetrain inside it); selecting a wheel, the centre of mass or a
shape expands the path to it. Select Chassis to
get the chassis page in the inspector (dimensions, mass, centre of mass). **Drivetrain** has its type, tire model and load-transfer
filter. Select a wheel to edit it (see *Linking wheels and mechanisms* for
followers), right-click Drivetrain or a wheel for **Add wheel**, a wheel for **Delete wheel**. The **Robot** node, which also
holds Mechanisms, Robot sensors and the game node, has the
robot-level settings.
* **Shapes (n)**: boxes and cylinders (**+ Box**, **+ Cylinder**) that model the visible, colliding parts. All the shapes of a
  mechanism move together with it (they rotate, swing or slide with the motor) about the mechanism's mount point, so place a
  shape on the axis to spin it in place.

### Mechanisms
**Right-click** the robot, the chassis or a mechanism in the tree and choose **Add mechanism** and then **Rotary**, **Arm** or **Linear** (on a mechanism:
**Add attached mechanism**, which hangs the new one on its link). There is no catalog panel. Core has one
template, **Mechanism**: a single motor-driven degree of freedom whose **Type** you choose in Configure:

| Type | Use |
|---|---|
| Rotary | flywheel, roller, indexer belt, turret (stops optional; equal stops = continuous) |
| Arm | hood, pivot, wrist, deploying intake (gravity-loaded, stops) |
| Linear | elevator, climber, slide (gravity along Z, stops) |

An intake is a rotary mechanism (plus an Arm if it deploys). Game-specific places such as a pickup volume or a shooter muzzle are not
mechanisms but **zones** (next section).

The template exposes **device slots** (motors, encoder, limit switches) and properties (motor model and count,
gearing, mass/inertia, stops, friction...) generated from its metadata.

Selecting a wheel or a mechanism shows its **Name** (editable) and then colour-coded cards:

| Card | Contents |
|---|---|
| **Drive** (orange) | How it is actuated, the same for every part. Per axis: **Own motors** (motor model, motor count, gear ratio, efficiency, current limit, neutral mode, and one row per controller, **Motor 1..N**, with **Reversed** and unmap) or **Linked to another part** (source and ratio). |
| **Configure** (blue) | What it is and how it is simulated: the **Type** (a wheel's type, or a mechanism's Rotary / Arm / Linear), then the properties of that type: dynamics, limits, steering of a swerve module (with its steer motor), game-specific settings, and the read-only list of its sensors. |
| **Placement** (green) | Attach to, attachment link, mount pose (wheels: hub position). |
| **Shapes** (purple) | Visible and colliding shapes, attached mechanisms. |

The chassis, drivetrain and shapes use the same card look with a single Configure card. **Add attached mechanism** hangs one on another mechanism's named link (for example a roller on an arm);
use **Attach to** / **Attachment link** and **Mount pose** to place it. Mechanisms move with the part they are
attached to. Plant models are documented in [mechanisms.md](mechanisms.md).

### Zones
A **zone** is a place on the robot that the game cares about and that is not a machine: a fuel **Pickup** volume in front of an intake,
a **Shooter** muzzle on a turret. Zones have no motors or slots; they hang on the chassis or on a mechanism's link and move with it.
The game plugin decides which kinds exist, so a season without zones simply has none.

* **Add**: right-click the game node (for example *REBUILT (2026)*) and choose **Add Pickup** / **Add Shooter** (one entry per zone kind
  the plugin offers). Right-click a mechanism for the same entries attached to that mechanism, or the Chassis for chassis-attached zones.
  The new zone is selected with its default placement and settings. The game node lists every zone below its own settings, with
  `@ chassis`, `@ intake` or `@ turret.mover` showing what it hangs on (a warning dot means the Validation tab has an issue for it).
* **Move and attach**: a **Volume** zone (Pickup) is a translucent box in the viewport, a **Muzzle** zone (Shooter) a small marker with
  an arrow along its +X direction (the firing direction). Pick it in the viewport or the tree, then drag the move / rotate gizmo
  (and, for volumes, scale) or type the **Pose** and **Size** in the **Placement** card. **Attached to** and **Attachment link**
  re-attach it to the chassis or another mechanism without moving it in the world; dragging a zone in the tree onto a
  mechanism or the Chassis does the same (or right-click **Attach to**). **Duplicate** and **Delete** are in the zone's inspector and
  context menu. Deleting a mechanism moves its zones to the mechanism's own parent, where they were.
* **Activation**: the **Activation** card (red) says when the zone works. With no conditions it says *Always active*. Each
  **+ Add condition** row tests one value a mechanism publishes: pick the **mechanism**, the **value** name (`velocity`, `position`...),
  **Above**, **Below** or **Within** a **threshold** (and a tolerance for Within), in SI units (m, rad, m/s, rad/s). **absolute
  value** compares the magnitude, so a roller spinning backwards still counts. **Active when** all or any of the
  conditions hold. A condition that names a mechanism that no longer exists stays in the list marked *(missing)* and the Validation tab warns until you
  pick another mechanism or remove it.
* **Configure** holds the settings of the zone kind (for example a pickup's capacity or a shooter's fire rate).

All of this is undoable like any other edit.

### Cameras (Limelight / PhotonVision)

A **camera** is a simulated vision coprocessor. Your robot code reads it exactly as on the real robot (`LimelightHelpers` for a
Limelight, PhotonLib's `PhotonCamera` for PhotonVision) and the emulator supplies AprilTag detections of the season's field; your
project is not modified. The model is simple on purpose: a tag is seen when it is inside the camera's field of view, within its
range and facing it, results arrive after the configured latency, and nothing blocks the view (see [Limitations](#8-limitations)).
Technical details: [vision.md](vision.md).

* **Add**: right-click the **Chassis**, a mechanism, the Robot node or the **Cameras** group in the tree and choose **Add camera** and
  then **Limelight** or **PhotonVision** (on a mechanism the camera hangs on that mechanism, on the others on the chassis). The new camera
  is selected, faces forward just behind the front of the hull (or sits at the mechanism's link origin), and is listed under the
  Robot node's **Cameras** group with `@ chassis` or `@ turret.mover` showing what it hangs on (a warning dot means the Validation
  tab has an issue for it). In the viewport it is a small body marker with a translucent view pyramid (the field of view out to its
  range); a Limelight is orange and a PhotonVision camera blue.
* **Move and attach**: pick it in the viewport or the tree and use the move / rotate gizmo (no scaling), or type the pose in the
  **Placement** card. **Attached to** / **Attachment link** (or dragging the camera onto a mechanism or the Chassis in the tree, or
  right-click **Attach to**) re-attach it without moving it in the world. **Duplicate** and **Delete** are in its inspector and
  context menu; deleting a mechanism moves its cameras to the mechanism's own parent.
* **Inspector**:

| Card | Fields |
|---|---|
| **General** | **Label**; **Type** (Limelight or PhotonVision); **Camera name**: for a Limelight the name from `LimelightHelpers` (empty = the default `limelight` table, otherwise `limelight-<name>`), for PhotonVision the `PhotonCamera` name. Names must be unique per type; changing the type picks a new default name |
| **Optics** | horizontal and vertical **field of view**, **maximum range** and **maximum incidence angle** (tags seen more obliquely are not detected), **frame rate** (fps), **latency** (ms from capture to the result being published), **resolution** (pixels, used for tag corners and area) and **pose noise** (position and yaw, 1 sigma; 0 = exact) |
| **Placement** | **Attached to**, **Attachment link** and the **pose** relative to that link (camera +X forward, +Y left, +Z up; for the chassis, relative to the centre of mass), then a read-only **Robot-to-camera (WPILib robot frame)** line: the transform to use in robot code |

* **Match your robot code.** The emulator cannot read the transform your code uses, so set the camera to the same mount:
  - Name: the camera name must equal the one in your code (`LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front")` ->
    **Camera name** `front`; `new PhotonCamera("front")` -> `front`).
  - Position and rotation: your code's `robotToCamera` (`Transform3d`, WPILib robot frame: origin on the floor below the centre of the
    robot, +X forward, +Y left, +Z up, rotation as roll/pitch/yaw) is the pose of the camera in the robot frame. The **Robot-to-camera**
    line under the Placement card shows exactly that transform for the camera as placed (its tooltip gives metres and radians), so
    edit the pose until it shows your numbers. The pose fields themselves are relative to the chassis **centre of mass**, so for a
    chassis camera they differ from your `Transform3d` by the COM offset: with the COM 0.05 m behind the hull centre and 0.2 m high,
    a camera at `(0.30, 0.0, 0.50)` in the code is entered as `(0.35, 0.0, 0.30)`. Rotation is entered as is (a camera pitched up 20
    degrees has pitch -20 degrees in WPILib's convention). For a camera on a mechanism the line shows the rest pose; in the
    simulation the transform follows the mechanism's motion.
  - Optics: copy the field of view, resolution, fps and latency from your Limelight or PhotonVision settings (the defaults are a
    generic 75 x 50 degree camera at 30 fps and 40 ms).
* **Pose reporting**: Limelight `botpose*` and PhotonVision multi-tag results report the robot pose in the field frame computed from the
  simulation's true pose (plus the pose noise you configured). Use noise > 0 to test your filter's rejection thresholds.
* **PhotonLib version.** PhotonVision's data format depends on the PhotonLib version. The emulator reads the `photonlib` version
  from your project's `vendordeps` and supports the versions whose result definition it verified (**v2025.3.2, v2026.1.1 to v2026.3.4
  and v2026.7**, see [vision.md](vision.md)). With another version the Console shows one error and **no PhotonVision camera is
  published** (`PhotonCamera` then reports *not connected*); Limelight cameras are unaffected and need no version setting.
* **In the run view** (the **AprilTags** item of the **View** menu toggles the tags): the field shows
  the season's AprilTags as the real 36h11 markers on their panels, and the panel of a tag seen by any camera turns green. The simulated result is also on
  NetworkTables, so AdvantageScope or Shuffleboard connected to the robot see it like on the real robot.
* **Cameras need a game module**: the tag layout belongs to the season's plugin (2026 in v1). Without one the cameras exist but
  never see a tag.

Cameras are saved in `robot.json` with the rest of the robot, are undoable, and are kept when the setup wizard is re-run.

### Device mapping
Your robot code creates devices (motor controllers, encoders, gyros, solenoids...) at run time. The simulator needs to
know which mechanism slot each one drives:

1. Entering the **Editor / Run** tab launches your robot project once (the first launch builds it and can take a
   while). The same instance detects devices and later runs the simulation; the *Detected devices* panel shows
   *"Starting robot program..."* until it connects. The robot code is always running, so devices appear
   as your code creates them.
2. The **Detected devices** table lists **Device**, **Type**, **ID**, **Bus** and **Status**
   (`unmapped (loopback)`, `built in`, `gone`, alias warnings).
3. **Drag a detected device** from the table onto a part or one of its slots **in the project tree**. Compatible
   targets highlight; incompatible drops are refused (*"Cannot map the device"*). The inspector only shows what is
   mapped; the **x** on a controller or sensor row unmaps it.
4. **Several motors on one gearbox.** A shooter with five motors or a tank side with three has one controller per
   motor in your robot code. Select the mechanism (or wheel): the **Drive** card has one controller row per motor. Drop the
   next controller on the part in the tree and a new motor row appears (or raise **Motor count**; lowering it unmaps
   the last rows). Tick **Reversed** on a motor that spins against the mechanism's positive direction (mounted mirrored, or inverted
   in robot code such as `setInverted(true)` / `Opposed`): the simulator flips its command and feedback, so two equal
   motors with opposite signs cancel and a wrongly inverted motor visibly fights the others. A motor row you leave
   unmapped mirrors motor 1. A wheel that follows another wheel (see *Linking wheels and mechanisms*) has no motor
   rows; give the **source** wheel the controllers of its side.
5. Controller and sensor rows show their state: required and unmapped, optional, mapped, *not detected* (mapped to a device the last
   discovery did not see), *gone*, or a type mismatch.

Restarting robot code invalidates the old live inventory. Devices absent from subsequent complete
discovery snapshots remain listed as `gone`; their saved slot mappings are preserved. Numeric
runtime device IDs being reused by a different device do not transfer physical identity or mappings.

For swerve with external/fused CANcoder feedback, map each module's CANcoder to its **steer encoder**
sensor slot (`fl.steerEncoder`, etc.), as well as mapping its steer motor. The encoder reports wheel
azimuth, not steer-motor rotations. Phoenix magnet offset and sensor direction are compensated from
firmware config read-back; do not copy the same magnet offset into a mechanism's sensor offset.
A calibration warning means raw passthrough is in use, not a calibrated run.

**Autonomous placement:** select Robot and enable **Use custom start pose** under Start pose, then set
field X, Y and heading to the path's initial pose. Start and Reset restore that physical placement.
Robot odometry resets do not teleport the chassis. Mirror/flip the start yourself when changing
the selected path or alliance; the physical start pose is not automatically alliance-dependent.

Devices that are detected but not mapped run in **loopback**: outputs are echoed back as inputs, so the code does not
break, but nothing physical happens. Required slots that stay unmapped read neutral values. Devices built into the
robot controller, such as the roboRIO, are listed as `built in`: there is no slot to map them to, so they never raise a warning.

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

**Your own intake and shooter** are built from normal Mechanisms (rotary roller / flywheel / indexer, arm hood, rotary
turret) with their devices mapped as above. The game part is two **zones** (see *Zones* in the editor chapter), added from the
game node's right-click menu:

* **Pickup** (a volume, shown in the viewport): place it where the intake reaches, attach it to the intake mechanism so it moves with a
  deploy arm, and give it an **Activation** such as *roller velocity above 20 rad/s*. Fuel inside the volume is absorbed while the
  zone is active and the meter is not full.
* **Shooter** (a muzzle marker with an arrow along +X): place it at the exit, attach it to the turret link, and give it an
  **Activation** such as *flywheel velocity above 30 rad/s* and *feeder running*. Shots are real physics bodies.

A condition that names a mechanism that does not exist leaves that zone inactive and the Validation tab warns. The settings
tables are in [game_2026.md](game_2026.md).

For CANrange fuel detection, add a **Distance sensor** (Add > Sensor), choose a game value such as
*Fuel count* as its **Source**, and map its device slot. In threshold mode set the fuel count at which it
reads the near distance, plus near/far distances suited to the robot's proximity tests. This approximates
inventory rather than individual fuel staging.

### Shelby 2026 characterization

The unmodified no-Maple Shelby project exercised all 28 CAN devices (29 discovered including
infrastructure), four external swerve encoders and two Limelights. A 12.1 m driving loop agreed with
robot pose to under 4 cm at sampled leg endpoints; stationary module azimuth errors were below 2 degrees.
Intake filled inventory from 8 to 30, and both CANrange-derived robot booleans cleared as shots depleted
the inventory. A 165-second auto/teleop run delivered 8,251 robot loops with zero dropped frames.

The locally saved model uses estimated geometry and inertia, disabled hood/pivot gravity (no CAD),
inventory-based range sensors, and a rear-facing ballistic muzzle fitted to the robot's shot tables.
Motor **Reversed** is on for hood 24, pivot 29, roller 32 and shooter 25/26; it is off for roller 31,
indexer 20/23, shooter 27/28 and drive/steer motors; indexer 21/22 are reversed.
The blue-right auto physical start is (4.51, 0.52654, 0 rad). Select `Double pass right` through the
robot's NetworkTables **Auto Chooser** (for example in Elastic); the emulator does not choose an auto.

**Known robot-code limitations, deliberately not hidden by model compensation:** `Hood.setMotor`
does not assign an in-range request to `target`, so the hood remains at zero despite the shot table.
Teleop homes successfully, but the autonomous command group leaves the hood unhomed on a fresh boot.
Short-range characterization scored 8/8 at 1.51, 2.0 and 2.5 m; a 5 m run fired 8 and scored none.
Full-range shooting is therefore not validated. Blue right/left and red-right autos completed the
first two paths and began the third, but did not complete all three within the 20-second period or
produce the two intended shots. Robot source and vendor libraries were left unchanged. A brief
Teleop enable can home the hood before trying Auto, but does not repair the in-range target bug.

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
| View controls | Icon buttons at the bottom-right of the 3D view: **Robot state** (list icon, opens the state window), **Restart** (`Ctrl+R`, puts the robot and game pieces back to the start; the robot code keeps running). Its dropdown adds **Restart robot code** (`Ctrl+Shift+R`, reboots the code and resets everything). **Speed** (relative to real time) |
| Camera | Same camera controls as the editor viewport (right-drag or Alt+left-drag to rotate; left-drag, middle-drag or Shift+drag to pan; wheel to zoom) and **Home view** (⌂ button, last in the bottom-right controls, `Ctrl+0`, or double-click the view) |
| Overlays | **View** menu: game HUD, zones (intake / shooter), vision cameras, AprilTags, individually switchable (Run tab only) |
| Robot state window | **Robot state** button in the 3D view, or **View > Robot state**: a separate window with a live tree of every mechanism (active / idle and its values), sensor (reading, game value it follows, and that value; a warning when the game does not publish it), zone (active / inactive) and camera (visible AprilTags). Rows expand and collapse; the window remembers its size and position |
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
| **Camera sees no tags / `tv` is 0 / PhotonCamera not connected** | Check that the camera **name** and **type** match your robot code, that the camera is at the pose your code assumes (a camera facing the floor or behind the robot sees nothing), the field of view and range, and that the season has a game module (tags come from it). A tag seen by a camera gets a green panel in the run view. For PhotonVision also check the console for a PhotonLib version error ([Cameras](#cameras-limelight--photonvision)). Remember there is a latency: results appear `latency` ms after the camera sees the tag. |
| **Config file could not be read** | *"The saved robot setup could not be read... set the robot up again"*: rerun the wizard. |
| **Nothing shows in the 3D view** | Update the graphics driver; Qt Quick 3D needs a working OpenGL/Direct3D device. The app also runs headless for tests only. |

## 8. Limitations

* **Suspension** is not simulated: the chassis is a rigid body on a flat floor with a tire model (bumps are climbed as
  ramps).
* **One robot** per session. Other robots on the field, opponent play and multi-robot game logic are out of scope.
* **Vision is simplified**: Limelight and PhotonVision AprilTag results are simulated with a plain field-of-view / range / facing
  model, a frame rate, a latency and optional pose noise (section 3, *Cameras*; [vision.md](vision.md)). Not simulated: occlusion
  (a tag behind the hub or another robot is still seen), lens distortion, exposure and lighting, colour / neural / object-detection
  pipelines, and image streams (no `HttpCamera` / `CameraServer` video). Robot poses reported by the camera come from the true pose plus
  the configured noise, not from a solve. The camera mount is configured in the editor, not read from your code. PhotonVision needs a
  PhotonLib version the emulator verified (v2025.3.2, v2026.1.1 to v2026.3.4, v2026.7); the tag layout exists only for seasons with a
  game plugin (2026). The vision sample for C++ was verified with the robot stationary (Phoenix 6 driving from C++ is unverified).
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
