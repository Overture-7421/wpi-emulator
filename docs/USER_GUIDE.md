# WPI Emulator user guide

Use this guide to open a team's robot project, describe its mechanisms, map its devices,
and run robot code on the simulated field. Start with the [swerve](../samples/swerve_robot/README.md)
or [vision](../samples/vision_robot/README.md) sample if you want a known device layout.

You need a WPILib 2026 Java or C++ GradleRIO project with its complete Gradle wrapper.
You do **not** need to install WPILib. C++ projects also need the platform's compiler
(MSVC build tools on Windows, Xcode command-line tools on macOS, or a Linux C++ compiler).
The 3D view needs a working graphics driver.

The emulator runs your robot code in a separate process. It injects its HALSim extension
through Gradle; it does not rewrite your robot source, build files, or vendordeps. The robot
program still builds normally and the emulator models the devices you describe and map.

## Install and launch

- **Windows:** run the installer or extract the whole portable ZIP. Launch
  `bin\wpiemu_app.exe` (installed by default at
  `C:\Program Files\WPI Emulator\bin\wpiemu_app.exe`).
- **macOS:** copy `wpiemu_app.app` from the DMG to Applications.
- **Linux:** make the AppImage executable and run it.

Packages include the app's Qt runtime, not a Java or WPILib toolchain. For package names and
caveats, see [Releasing](releasing.md); for source builds, see [Building](BUILDING.md).

For example, in PowerShell:

```powershell
& "C:\Program Files\WPI Emulator\bin\wpiemu_app.exe" --project "C:\Robots\MyRobot"
```

`--project <folder>` opens a project at startup. Otherwise, **Reopen the last project when the
app starts** is enabled by default. `--demo` opens the scripted demo without launching robot
code. `--version` prints the SDK, IPC, core, and plugin version summary; it is not the
product-release version. An installed copy can update itself; see [Updates](#updates).

## Set up a robot

Drop a project folder on the window or use **File > Open project…** (`Ctrl+O` on Windows).
If it has no saved robot model, the app opens **Set up your robot**. Reopen the wizard with
**File > Set up robot…**.

1. **Project:** select the folder containing `build.gradle`. Check the detected WPILib season,
   language, team number, and vendor libraries. An unreadable project or missing GradleRIO is
   an error; unknown season information or an incomplete wrapper needs attention before launch.
2. **Toolchain:** wait for the Java, wrapper, WPILib, C++ compiler (when needed), and game-module
   checks. Use each row's fix, then **Re-check**. If no Java that can run the project is found,
   the Java row may offer **Get Java** (with its version); on Windows, a missing C++ compiler may
   offer **Install C++ build tools**. You can finish setup with missing tools, but robot code will
   not launch until they are fixed. For Java selection and launch rules, see the
   [Launcher boundary](reference/runtime.md#launcher-boundary).

   **Core-only mode** means the season has no game module, so game pieces, scoring and
   season-specific field structure are not simulated. The robot drives on a generic FRC field
   (carpet, alliance walls and guardrails). It does not promise that the bundled HALSim extension
   works with another WPILib year.
3. **Robot:** choose **Swerve**, **Tank**, or **Mecanum** and enter the frame, wheel, motor,
   gearing, alliance, and station details.
4. **Summary:** review the model and save location, then choose **Open editor**.

The setup is saved in your project as `.wpiemu\robot.json`; commit it to share the same model
with your team. Running setup again keeps existing mechanisms, cameras, zones, game settings,
and surviving device mappings. A changed wheel layout can remove wheel mappings, so check
them afterward. For file reload, save, and recovery behavior, see
[Persistence responsibilities](reference/runtime.md#persistence-responsibilities).

## Describe the robot

The **Editor** page has a project tree, a detected-device list, a viewport, and an inspector.
Select **Chassis** to edit its mass, dimensions, and centre of mass; **Drivetrain** contains
the drive type and wheels. Select a wheel to edit its position, motor, gearing, and (for
swerve) steering. Right-click Drivetrain or a wheel to **Add wheel**.

Use boxes and cylinders in **Shapes** for visible or colliding robot parts. Right-click Robot
or Chassis and choose **Add mechanism > Rotary / Arm / Linear** for a roller, gravity-loaded
pivot, or elevator/slide. **Drive** controls actuation; **Configure** holds model settings and
sensors; **Placement** sets attachment and mount pose. Use **Add attached mechanism** to model
parts that move together.

Select a part in the tree or viewport. The viewport toolbar's **Move**, **Rotate**, and
**Scale** modes use `W` / `E` / `R`; **Front**, **Side**, **Top**, and **3D** choose views,
and **Frame** (`F`) frames the selection. **Snap** uses grid and 15° increments; hold `Ctrl`
while dragging to toggle snapping for that drag. Left/middle-drag pans, right-drag orbits,
and the wheel zooms. `Del` deletes and `Ctrl+D` duplicates supported selections; `Ctrl+Z` /
`Ctrl+Y` undo and redo. The bottom panel contains **Console**, **Validation**, and **Telemetry**.

For parts sharing motion, select the follower and choose **Drive > Linked to another part**.
Select its source and signed ratio; a negative ratio reverses direction. The follower gives
up its own motor mapping but retains sensor slots. Link source parts to controllers.
Links do not send follower loads back to the source and are not a torque-sharing gearbox model.
Display units do not change saved SI values.

## Discover and map devices

Open a configured project to launch its robot code for device discovery. The first Gradle build
may take time. Devices appear when robot code constructs them. In the editor, **Detect devices**
requests discovery again; **Stop detecting** stops robot code. The same robot process normally
serves discovery and simulation; after model edits, entering Run may relaunch it with the
updated configuration.

Drag a detected controller or sensor onto a compatible part or slot in the project tree.
Map drive motors, swerve steer motors, external steer encoders, and the robot IMU separately.
An external steer encoder measures wheel azimuth, not steer-motor rotations. For multiple
controllers on a mechanism, increase **Motor count** or drop another controller on the part,
then map each motor row. Use **Reversed** when a controller's positive direction opposes the
modeled part. An unmapped additional motor row mirrors the first; map only source wheels when
the other wheels are linked followers.

`unmapped (loopback)` means a device's output is echoed as feedback without driving a physical
model. Other unmapped slots return neutral values. `built in` devices need no mapping; `gone`
devices keep their saved mapping but are not currently live. Fix required
slots, incompatible or duplicate mappings, and alias conflicts in **Validation**.

Phoenix simulation may not distinguish physical CAN buses. A blank or unknown **Bus** is not
evidence of a roboRIO bus, and the same device type and ID on separate physical buses cannot
necessarily be mapped as independent simulated devices. See [Bridge limits](reference/bridges.md).

## Configure the game and cameras

The built-in REBUILT module supplies the 2026 field, fuel, and hub-shift scoring. Select the
game node to set inventory/preload, field staging, and match rules. Model the intake and
shooter as mechanisms, then add **Fuel pickup** and **Fuel shooter** zones from the game,
Chassis, or a mechanism's context menu. A pickup adds fuel to inventory; a shooter launches it.
Attach each zone to the chassis or a moving mechanism link. **Activation** tests mechanism
values (all or any conditions, optionally using absolute value); with no conditions it is
always active. Configure rate and shooter parameters in the inspector. Missing mechanism
references produce validation warnings. A fuel-count distance sensor estimates inventory,
not the locations of individual balls.

Add a camera from **Add camera > Limelight / PhotonVision** on Robot, Chassis, a mechanism, or
Cameras. Match the camera name to robot code: Limelight `front` publishes as `limelight-front`
(an empty name publishes as `limelight`); PhotonVision uses the exact `PhotonCamera` name.
Use the inspector's **Robot-to-camera (WPILib robot frame)** transform to match your code's
mount. The editable pose is relative to its attachment; for a chassis attachment it is relative
to the centre of mass. A mechanism-mounted camera moves with its link.

**Optics** sets field of view, range/incidence, frame rate, latency, resolution, and pose noise.
The modeled cameras report AprilTag results to the robot's NetworkTables; they need a season
AprilTag layout. This is geometric simulation, not image processing: there are no camera images,
occlusion, lens distortion, lighting, or object detection, and reported robot pose is truth plus
configured noise. PhotonVision publication depends on the project's PhotonLib result definition
and an emulator build with ntcore support; an unsupported definition is reported in Console and
does not suppress Limelight. See [bridge compatibility](reference/bridges.md) and the
[vision sample](../samples/vision_robot/README.md).

## Run with the Driver Station

Switch between **Editor** and **Run** with the page tabs (`Ctrl+1` / `Ctrl+2`). Entering Run
starts simulation when the robot is ready. Use the built-in **Driver Station**, not the real
Driver Station application:

- Select **Auto**, **Teleop**, or **Test** while disabled; click the large toggle, captioned
  **Enabled** even while disabled, to run that mode. **Practice** runs autonomous,
  a disabled gap, then teleop on a match clock.
  Practice defaults are 20 seconds autonomous, 3 seconds gap, and 140 seconds teleop; adjust
  them in **Driver Station settings > Setup**.
- Click the enable toggle again or press `Enter` to disable. `Space` latches emergency stop; click
  **E-STOPPED** to clear it. The Driver Station settings also set alliance, station, game
  message, Practice timings, and joystick bindings.
- The default keyboard joystick uses `W A S D` for the left stick, arrows for the right stick,
  `U` / `O` for triggers, `Z X C V Q E R F G H N M` for buttons, and `I J K L` for the POV.
  Click a binding to relearn it; `Esc` cancels. Rebinding works while the Driver Station
  settings window itself is focused; you do not need to click the main window first. Gamepads
  require an SDL2-enabled build; keyboard input works without SDL2.

Practice plays field sounds for the match start, autonomous end, teleop start, and match end;
the selected game can add cues. Restart plays a reset cue; only an emergency stop during a
running Practice match plays the foghorn. Disable ends the match quietly. Sounds play only
while Run is visible. Turn them off or adjust volume in **File > Settings… > General**. See
[game sound cues](reference/game-plugin.md#sound-cues).

The viewport controls include **Restart** (`Ctrl+R`), which resets physics and game pieces
while keeping robot code running, and its **Restart robot code** option (`Ctrl+Shift+R`), which
reboots code too. **Speed** changes simulation pacing (default `1×`); **Home view** (`Ctrl+0`
or double-click the view) restores the camera. **View > Robot state** opens live mechanisms,
sensors, zones, and camera detections; **View > Console** toggles output (`Ctrl+J`).

For repeatable autonomous placement, set **Use custom start pose** or add a named start
position from Robot's context menu. A start position records alliance, field X/Y, and heading;
**Copy to Red** / **Copy to Blue** uses game-field symmetry when available. The Run
**Start position** control chooses where the chassis is placed on restart. It does not reset
your robot's odometry; reset that in robot code. In REBUILT, alliance walls and guardrails are
transparent above their opaque base, while the tower wall and outpost stay opaque.

The REBUILT HUD shows fuel and shots; Practice adds match phase, hub activity, and fuel scores.
Tower points, penalties, and ranking points are not scored.

## Settings and project files

**File > Settings…** includes theme, dialog units, WPILib and JDK folders, physics rate and
threads, reopening the last project, update checks, and Practice sound settings. Physics rate
defaults to **200 Hz**; physics threads default to **Auto** (up to four threads). Physics rate
is copied into the model when setup finishes, so rerun setup to apply a changed rate. Thread
choice applies at the next robot launch and does not change results.

The robot model autosaves to `.wpiemu\robot.json` in the project. Edits made outside the app
are reloaded; if you have unsaved edits, choose whether to load the disk copy or keep yours.
Save and backup details are in [Persistence responsibilities](reference/runtime.md#persistence-responsibilities).

### Updates

An installed copy checks for a stable release at startup by default. When one is available,
use the menu-bar **Update** button or **Help > Check for updates…**. The update button's action
depends on the installed package and platform; development builds and `--demo` do not update.
Disable the startup check in **File > Settings… > General**. For download verification,
installation, and platform details, see [Updates](reference/app.md#updates).

## Troubleshooting and limits

- **Robot does not start:** check Console and Validation; confirm the wrapper and its JARs,
  build the project outside the emulator, and check the Java and (for C++) compiler rows in setup.
  The first build downloads WPILib and vendor libraries and may take time.
- **Old MSVC runtime:** if the launcher reports a runtime below 14.40, select WPILib's Java or
  use the setup wizard's **Get Java** option when offered.
- **No extension or handshake:** check the WPILib season, matching app/extension installation,
  and robot load errors. Advanced extension-location overrides are in [Releasing](releasing.md).
- **No devices or motion:** confirm the robot constructors ran, map the commanded devices, and
  check enable state, motor signs, and motion links. Avoid another simulator writing the same
  vendor simulation state.
- **No vision:** check names, mount transform, optics, AprilTag layout, and latency; inspect
  PhotonLib errors. Vision publication also needs an emulator build with ntcore support.
- **Unreadable setup or save failure:** check the project file and write access; see the
  [persistence reference](reference/runtime.md#persistence-responsibilities).
- **Blank viewport:** check the graphics backend. Headless tests do not prove 3D rendering.

### Limits and platforms

One robot runs at a time. Multi-season packages contain separate WPILib 2026 and 2027
extensions, selected from the live project's year. The Toolchain page reports a missing
season extension before launch. `WPIEMU_EXTENSION` explicitly overrides selection and
permits a year mismatch with a warning; it does not make incompatible HAL ABIs compatible.
2027 is preview support pinned to an alpha; it uses Java 25, Gradle 9.2 and the new native
toolchain. A season without a game module runs core-only on a generic FRC field. A project on a different
WPILib release of a known season than the emulator was built for still runs: the Toolchain page and run
console warn that it is best effort.
There is no suspension model, and field geometry and game rules are approximations.
HAL and CTRE bridges are supplied, not universal vendor support: REV motor controllers have no
vendor bridge, and Phoenix 5 behavior is not characterized against the real library. The
[vision sample end-to-end check](../samples/vision_robot/README.md#end-to-end-check) exercises
Phoenix 6 driving, wheel feedback, and camera results in Java and C++ on Windows, but does not
characterize every Phoenix model or control mode.
For the pinned 2027 preview, Java PhotonLib and Limelight have actual client coverage;
the pinned C++ PhotonLib has an upstream UUID-check failure and is not supported yet.
See the [2027 probes](../samples/season_2027/README.md) for exact versions and limits.

Windows is the locally exercised workflow. Linux/macOS builds, presets, and package checks do
not establish native robot-process HAL loading or interactive 3D validation on those platforms.
Validate real sessions on the target platform.

Next: [documentation home](README.md) for task guides, or
[runtime reference](reference/runtime.md) for configuration and launcher contracts.
