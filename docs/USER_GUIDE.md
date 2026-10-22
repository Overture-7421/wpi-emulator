# WPI Emulator user guide

Read this when installing the app, describing a robot or running your robot code.
Prerequisites: a WPILib 2026 Java or C++ project using GradleRIO, its complete Gradle wrapper,
and the WPILib desktop simulation toolchain. Use WPILib's bundled JDK; the toolchain check
accepts JDK 17+. C++ projects also need MSVC C++ tools on Windows, Xcode command line tools
on macOS, or a Linux C++ compiler. The 3D view needs a working graphics driver.

The launcher injects a HALSim extension through Gradle without rewriting your robot source,
build configuration or vendordeps. Normal Gradle outputs/caches and the robot setup in
`.wpiemu\robot.json` are still written.

For a first run, the [swerve](../samples/swerve_robot/README.md) and
[vision](../samples/vision_robot/README.md) samples provide known device layouts and expected output.
To change the emulator rather than a robot model, start with the [developer guides](extending.md).

## Install and launch

Windows: run the NSIS installer, or extract the **whole** portable ZIP. The installed executable
is `C:\Program Files\WPI Emulator\bin\wpiemu_app.exe`; a portable tree also uses `bin\wpiemu_app.exe`.
macOS: open the DMG and copy `wpiemu_app.app` to Applications. Linux: make the AppImage executable
and run it. Packages include the app's Qt runtime, not the robot's WPILib toolchain.
Artifact names and packaging caveats are in [Releasing](releasing.md);
source-build instructions and Qt DLL setup are in [Building](BUILDING.md). The app can update itself; see [Updates](#updates).

For example, in Windows PowerShell:

```powershell
& "C:\Program Files\WPI Emulator\bin\wpiemu_app.exe" --project "C:\Robots\MyRobot"
```

`--project <folder>` opens that project at startup. Without it, the app can reopen the last
project; disable that in Settings. `--demo` substitutes scripted devices and motion without
launching robot code; open a project to use the editor/run pages. `--version` prints the existing
SDK, IPC, core and plugin summary without opening a window; it is not a product-release version.

## Set up your robot

Drop a project folder on the window or use **File > Open project…** (`Ctrl+O` on Windows).
A project without a saved model opens **Set up your robot**. Use **File > Set up robot…**
to revisit it.

**Project:** choose the folder containing `build.gradle`. Check the detected season, language,
team number and vendor libraries. Missing GradleRIO or an unreadable project is an error;
an unknown season or incomplete wrapper needs attention before running.

**Toolchain:** wait for checks of Java, the wrapper, WPILib, the C++ compiler when needed,
and the game module. Follow each missing row's hint, then **Re-check**.
You may finish setup with missing tools, but must fix them before robot code can run.
**Core-only mode** means no season field, game pieces or scoring; it does not promise that
the bundled HALSim extension works with another WPILib year.

**Robot:** choose **Swerve**, **Tank** or **Mecanum**. Enter dimensions, mass, bumpers, wheel
diameter, motors, gearing, alliance and station. Fix highlighted values before continuing.
**Summary:** review the model and save locations, then choose **Open editor**.

Revisiting the wizard keeps mechanisms, cameras, zones, game settings and surviving device
mappings. Changing wheel layout can remove wheel mappings; check them again afterward.

## Build the model

The **Editor** and **Run** tabs switch pages (`Ctrl+1` / `Ctrl+2`). In the editor, the project
tree and detected devices are on the left, the viewport in the centre and the inspector
on the right; the bottom panel contains Console, Validation and Telemetry.

Select **Chassis** to edit mass, dimensions and centre of mass; **Drivetrain** contains its
settings and wheels. Select individual wheels for hub positions, motors, gearing and steering.
Right-click Drivetrain or a wheel to add a wheel; a wheel's menu can delete it.
Use boxes/cylinders in **Shapes** to describe visible and colliding parts.

Right-click Robot or Chassis for **Add mechanism > Rotary / Arm / Linear**. Use Rotary for
rollers, flywheels or turrets, Arm for gravity-loaded pivots, and Linear for elevators/slides.
**Drive** controls actuation; **Configure** holds dynamics, limits and sensors;
**Placement** sets the parent/link and mount pose. **Add attached mechanism** on an existing
mechanism makes a moving assembly. Shapes move with their owning part.

For parts sharing motion, select the follower and choose **Drive > Linked to another part**,
then the source and signed ratio. A negative ratio reverses motion; mixed rotary/linear
links show their units. The follower loses its own motor mappings but keeps sensor slots.
Map controllers to the source; choose **Own motors** to unlink. Wizard-made tank wheels
already follow one powered wheel per side. Links do not reflect follower loads back to
the source and are not a torque-sharing gearbox model.

Left-click selects. Right-drag or Alt+left-drag orbits; middle-drag or left-drag on empty
space pans; the wheel zooms. `W` / `E` / `R` choose move/rotate/scale, `F` frames selection,
and `1`–`4` choose Front/Side/Top/3D. Hold `Ctrl` during a gizmo drag to toggle snapping.
Use `Del` to delete and `Ctrl+D` to duplicate supported selections while the editor has focus.
Undo/redo (`Ctrl+Z` / `Ctrl+Y`) includes gizmo drags. Display units do not change saved SI values.

## Map devices

Opening a configured project launches robot code for live discovery. The first Gradle build
can take time. Devices appear as robot code constructs them; **Detect devices** in the editor
can request discovery again; **Stop detecting** stops robot code.
A robot instance normally serves discovery and simulation;
entering Run after model edits can relaunch it with the changed configuration.

Drag a detected controller or sensor onto a compatible part or slot **in the project tree**.
The inspector shows mapped devices; use a row's unmap button to remove a mapping.
Map drive motors, swerve steer motors, external steer encoders and the robot IMU separately.
An external steer encoder represents wheel azimuth, not steer-motor rotations.

For multiple controllers on one gearbox, increase **Motor count** or drop another controller
on the part, then map each motor row. Use **Reversed** for a controller opposing the part's
positive direction; incorrect signs can make motors fight. An unmapped additional motor row
mirrors the first motor. Map only source wheels when the others are linked followers.

`unmapped (loopback)` means a detected device's outputs are echoed as feedback without driving
a physical model. Other unmapped slots read neutral values. `built in` devices need no mapping;
`gone` devices retain saved mappings but are not currently live.
Fix required-slot, incompatible-device, duplicate-mapping and alias-conflict issues in **Validation**.
Phoenix simulation may not distinguish CAN buses: a blank/unknown Bus is not a guessed roboRIO bus,
and the same type/ID on separate physical buses cannot be mapped as independent simulated devices.

## Configure the game and cameras

The built-in REBUILT game supplies the 2026 field, fuel and hub-shift scoring. Select its
game node to adjust inventory/preload, field staging and match rules.
Build the intake/shooter machines from ordinary mechanisms, then right-click the game node,
Chassis or a mechanism for **Add Fuel pickup** / **Add Fuel shooter**.

A pickup is a volume where fuel enters inventory; a shooter is a muzzle pointing along +X.
Use **Placement** to attach them to the chassis or a moving mechanism link.
**Activation** conditions test mechanism values, with all/any conditions and optional absolute
value. No conditions means always active. **Configure** sets pickup rate or shooter flywheel,
aiming and fire-rate parameters. Missing referenced mechanisms produce validation warnings.
For inventory-based sensing, use **Add sensor**, choose a game value as **Source**, and map
the sensor slot. A fuel-count distance sensor approximates inventory, not individual ball staging.

Right-click Chassis, a mechanism, Robot or Cameras for **Add camera > Limelight / PhotonVision**.
Match **Camera name** to robot code: Limelight `front` publishes as `limelight-front`
(empty means `limelight`); PhotonVision uses the exact `PhotonCamera` name.
In **Placement**, match the read-only **Robot-to-camera (WPILib robot frame)** transform to
your code's mount. The editable pose is relative to its attachment, not necessarily the floor:
chassis attachments are relative to the centre of mass. Mechanism-mounted cameras move with the link.

Set **Optics** for field of view, range/incidence, frame rate, latency, resolution and pose noise.
Results reach the robot's own NetworkTables; cameras need a season AprilTag layout.
The app reads PhotonLib's version from vendordeps. An unsupported result definition produces a
Console error and suppresses PhotonVision publication; Limelight is independent.
See [bridge compatibility](reference/bridges.md) and the [sample robots](../samples/vision_robot/README.md).

## Run with the Driver Station

Entering **Run** starts simulation automatically when the robot is ready. There are no
start/stop/pause/step buttons in this run view. Robot code normally stays active between
pages; project closure, Stop detecting or Restart robot code ends that instance.

Use the built-in **Driver Station**, not the real Driver Station application.
Choose **Auto**, **Teleop**, **Test** or **Practice** while disabled, then **Enable**.
Auto/Teleop/Test run until disabled. Practice runs autonomous, a disabled gap, then teleop
using the configured match clock. Click Enable again or press `Enter` to disable.
`Space` latches emergency stop; click the E-STOPPED control to clear it.

Practice plays the field's sounds: the start charge, the buzzer when autonomous ends, three bells when teleop begins,
a buzzer at the end, a chime when you Restart and, in REBUILT, a pop at each alliance shift (2:10, 1:45, 1:20, 0:55),
a whistle when the end game starts (0:30) and a chime when the final score posts. Only an E-Stop of a running
Practice match sounds the foghorn; Disable ends a match quietly. Sounds play only while the Run page is showing.
Turn them off or set the volume in **File > Settings… > General**.

The cog opens **Setup** (alliance/station, game message, Practice durations) and **Joysticks**.
Game-supplied field data can override the manual game message.
Assign the keyboard or a gamepad to the robot's joystick port and adjust bindings there.
Default keyboard controls: `W A S D` left stick, arrows right stick, `U` / `O` triggers,
`Z X C V Q E R F G H N M` buttons, and `I J K L` POV. Click a binding to relearn it;
`Esc` cancels. Gamepads require an SDL2-enabled build; keyboard input remains available without it.

At the viewport's bottom-right, **Restart** (`Ctrl+R`) resets physics and game pieces but keeps
robot code running. Its dropdown's **Restart robot code** (`Ctrl+Shift+R`) reboots code too.
**Speed** changes pacing. **Home view** (`Ctrl+0` or double-click) restores the camera.
Orbit/pan/zoom work as in the editor. In REBUILT the alliance walls and guardrails are see-through above their
opaque base, so you can put the camera behind a wall where the drivers stand and still see the field; the tower wall
and outpost stay opaque, as on the real field.

For autonomous placement, set Robot's **Use custom start pose** or **Add start position** from
its context menu. Named starts have alliance, field X/Y and heading; **Copy to Red/Blue**
uses the game's field symmetry when available. The Run **Start position** flag selects one,
changes alliance and resets there; Restart keeps that choice until changed.
The emulator places the physical chassis, not robot odometry: reset odometry in your robot code.

**View > Robot state** opens live mechanisms, sensors, zones and camera detections.
The View menu also toggles game HUD, zones, AprilTags and Console (`Ctrl+J`).
Console can filter/search/copy robot and emulator output. Stats show simulation timing and battery.
The REBUILT HUD shows fuel and shots; Practice adds match phase, hub activity and fuel scores.
Tower points, penalties and ranking points are not scored.

## Settings and project files

**File > Settings…** controls theme, dialog units, the WPILib and JDK folders, physics rate, physics threads,
reopening the last project, the update check and Practice sound effects (on/off and volume). The folders apply to the next toolchain check and robot launch; left
empty, Gradle runs with WPILib's JDK for the project's season (all-users or this-user install),
then `JAVA_HOME`, then `java` on `PATH`. The physics-rate setting
is copied into the model when the wizard finishes: rerun setup to apply a changed rate.
Physics threads (Auto, 1, 2, 4 or 8) belongs to the computer, not the model: it applies at the next robot
launch, Auto picks up to four threads from the core count, and results are identical for every choice.
The Physics value in the run view shows the thread count in its tooltip.
An inherited `JAVA_HOME` that has no Java executable is ignored; an explicitly selected JDK
folder must be corrected in Settings rather than silently falling back to another JDK.

Edits autosave; **Save** / `Ctrl+S` saves immediately. Closing, switching projects or rerunning
setup first saves pending edits. A save failure cancels that transition and leaves the document open.

The robot setup lives in the project, at `<project>\.wpiemu\robot.json`, and nowhere else.
Commit it to share the setup with your team: what is in version control is what runs.
The open project watches the file: when it changes outside the emulator (a git pull, a hand edit)
the setup is loaded again on its own. With unsaved edits open, the emulator asks first whether to
load the file or keep your edits, which the next save then writes over it. The emulator only writes
the file after an edit, so an open project never overwrites a pull by itself.
A file that cannot be loaded (a merge conflict, a hand edit, a newer schema) is reported: at open it
offers setup, while a project is open the setup in memory stays. Whatever replaces an unreadable file
first keeps it as `robot.json.unreadable.bak` beside it.
Projects set up before this kept their setup under the app-data root
(`projects\<folder>-<hash>\robot.json` in `%APPDATA%\WpiEmulator`, `~/Library/Application Support/WpiEmulator`
or `~/.local/share/wpi-emulator`); opening one moves that setup into the project once and keeps
the old file as `robot.json.migrated.bak`. UI preferences are separate Qt settings.

## Updates

An installed copy asks `github.com` whether a newer stable release exists each time it starts. When one does, a green
**Update** button appears at the right of the menu bar; the same action is under **Help**. Click it to download the installer
(the button shows the progress and "Verifying…"), then click **Restart to update**: WPI Emulator closes, saving your work
first and asking before it ends a running simulation, and the installer takes over (Windows asks for administrator rights; you
can start the new version from its last page). On macOS the button reads **Open update** and opens the disk image, and on
Linux the new AppImage is saved beside the old one and started. A portable ZIP or tree only opens the release page.
Right-click the button for the release notes, to skip that version or to cancel a download. **Help > Check for updates…**
checks on demand and says when you are current.

A download is deleted unless it matches the checksum published with the release. The installers are not code-signed
yet, so Windows may show an "unknown publisher" prompt. Only the version request goes to GitHub, and nothing about you or
your projects is sent. Turn the start-up check off in **File > Settings…**. Development builds and `--demo` never check.

## Troubleshooting

- **Failed start/crash:** read Console and Validation. Confirm the wrapper and its JAR exist,
  the project builds outside the emulator, and WPILib's JDK/compiler are available. Allow time for first build.
- **No extension/handshake/protocol:** reinstall a matching app and extension, check robot load errors
  and the WPILib year. Advanced extension-location overrides are described in [Releasing](releasing.md).
- **Robot not responding:** blocked robot code can trip the watchdog; inspect output and use Restart robot code.
- **No devices/motion:** ensure constructors ran, map the actual commanded devices, check enable state,
  motor signs and links. Do not let another simulator also write the same vendor simulation state.
- **No vision:** check name, mount, FOV/range, tag layout and latency; inspect PhotonLib errors.
  Camera publication also needs an emulator build with ntcore support.
- **Unreadable setup/save failure:** fix or remove the file (setup keeps it as `.unreadable.bak`
  before replacing it); check disk space and that the project folder is writable. A blank viewport needs a working graphics backend; headless tests do not prove rendering.

## Limits and platforms

Only one robot and the built-in 2026 season are supplied. Other seasons can use an empty field,
but the bundled extension targets WPILib 2026; another year is not a compatibility guarantee.
There is no suspension model. Field geometry and game rules are approximations.
HAL and CTRE bridges are supplied, not universal vendor support: REV motor controllers lack
a vendor bridge, and Phoenix 5 rules are not characterized against the real library.
The [vision sample checks](../samples/vision_robot/README.md#end-to-end-check) exercise Phoenix 6
driving, wheel feedback and camera results in both Java and C++ on Windows; this does not characterize
every Phoenix model or control mode.
Vision models AprilTags using geometry, latency and optional noise, not image processing:
no occlusion, lens distortion, lighting, object detection or video streams; reported robot pose uses truth plus noise.

Windows is the locally exercised workflow. Linux/macOS source, presets and CI/package checks exist,
but do not establish native robot-process HAL loading or interactive 3D validation on those platforms.
Headless smoke tests are not visual tests; validate real sessions on the target platform.

Next: [documentation home](README.md) for task guides, or [runtime](reference/runtime.md) for file contracts.
