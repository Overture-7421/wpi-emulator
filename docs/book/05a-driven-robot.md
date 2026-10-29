# Drive day workshop: give the robot something to move

**In 30 seconds:** a motor command needs a model and a mapping before it can move anything.
We will replace the repository's one-motor API probes with matching C++ and Java drive fixtures.
They use only the pinned 2027 alpha-7 APIs and normal PWM controllers, not plugin calls.

## 1. Agree on the wiring before writing autonomous

| PWM | Robot controller | Model destination |
|---|---|---|
| 0 | Left drive | Left traction wheel |
| 1 | Right drive | Right traction wheel |
| 2 | Release roller | Generic rotary mechanism called `roller` |

This is a deliberately small two-wheel physics fixture, with default motor plants and a fixed
release chute. PWM describes how a motor is controlled, not its motor model. The two drive
plants use the default Kraken X60 model and a 10.71:1 reduction; the PWM controller can command
that plant without a vendor dependency. This is not a detailed or competition-legal robot CAD model.

We start at X=1 m, aligned with the near switch's Blue plate. The robot drives straight into the
fence, stops driving after six seconds, then runs the roller at seven seconds. The wall provides
repeatable positioning for this fixture; there is no encoder feedback or path planner here.
The release muzzle is above and beyond the fence, so the released cube falls onto the plate.
Teleop intentionally stops all motors while ownership continues scoring.

## 2. Build the C++ routine

Replace <code>samples\season_2027\cpp\src\main\cpp\Robot.cpp</code>.
Read and validate the **entire** FMS message in `AutonomousInit`, then reset the periodic counter.
We only drive for a legal assignment. The saved start pose is matched to the chosen field side;
this routine does not steer across the field when the assignment changes.

<!-- book-file: samples\season_2027\cpp\src\main\cpp\Robot.cpp -->
```cpp
#include <iostream>

#include <wpi/driverstation/MatchState.hpp>
#include <wpi/drivers/motor/PWMSparkMax.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/framework/TimedRobot.hpp>

class Robot final : public wpi::TimedRobot {
public:
    void AutonomousInit() override {
        const auto gameData = wpi::MatchState::GetGameData();
        validAssignment_ = gameData && (*gameData == "LLL" || *gameData == "LRL" ||
                                         *gameData == "RLR" || *gameData == "RRR");
        autoTicks_ = 0;
        std::cout << "POWER_UP auto " << gameData.value_or("") << std::endl;
    }
```

`TimedRobot` calls the periodic routine every 20 ms of **simulated** time. Three hundred callbacks
are about six seconds; 350 are about seven. We run the roller briefly after stopping the drive,
and route every disabled/teleop callback through the same stop helper.

<!-- book-file: samples\season_2027\cpp\src\main\cpp\Robot.cpp -->
```cpp
    void AutonomousPeriodic() override {
        ++autoTicks_;
        const double drive = validAssignment_ && autoTicks_ < 300 ? 0.25 : 0.0;
        left_.SetThrottle(drive);
        right_.SetThrottle(drive);
        roller_.SetThrottle(validAssignment_ && autoTicks_ >= 350 && autoTicks_ < 400 ? 0.4 : 0.0);
    }

    void TeleopInit() override {
        stop();
        std::cout << "POWER_UP teleop" << std::endl;
    }
    void TeleopPeriodic() override { stop(); }
    void DisabledInit() override { stop(); }

private:
    void stop() {
        left_.StopMotor();
        right_.StopMotor();
        roller_.StopMotor();
    }

    wpi::PWMSparkMax left_{0};
    wpi::PWMSparkMax right_{1};
    wpi::PWMSparkMax roller_{2};
    int autoTicks_ = 0;
    bool validAssignment_ = false;
};

int main() { return wpi::StartRobot<Robot>(); }
```

## 3. Match it in Java

Replace <code>samples\season_2027\java\src\main\java\frc\robot\Main.java</code>.
The structure is identical: normal WPILib startup, three PWM outputs, validated FMS message,
and a reset periodic counter. The Java package is `org.wpilib`, not the pre-2027 namespace.

<!-- book-file: samples\season_2027\java\src\main\java\frc\robot\Main.java -->
```java
package frc.robot;

import org.wpilib.driverstation.MatchState;
import org.wpilib.drivers.motor.PWMSparkMax;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    public static final class Robot extends TimedRobot {
        private final PWMSparkMax left = new PWMSparkMax(0);
        private final PWMSparkMax right = new PWMSparkMax(1);
        private final PWMSparkMax roller = new PWMSparkMax(2);
        private int autoTicks;
        private boolean validAssignment;

        @Override
        public void autonomousInit() {
            String gameData = MatchState.getGameData().orElse("");
            validAssignment = switch (gameData) {
                case "LLL", "LRL", "RLR", "RRR" -> true;
                default -> false;
            };
            autoTicks = 0;
            System.out.printf("POWER_UP auto %s%n", gameData);
        }
```

The timing and stop behavior are intentionally the same as C++. If one language delivers and
the other does not, we want to investigate the launcher/ABI path, not two different strategies.

<!-- book-file: samples\season_2027\java\src\main\java\frc\robot\Main.java -->
```java
        @Override
        public void autonomousPeriodic() {
            ++autoTicks;
            double drive = validAssignment && autoTicks < 300 ? 0.25 : 0.0;
            left.setThrottle(drive);
            right.setThrottle(drive);
            roller.setThrottle(validAssignment && autoTicks >= 350 && autoTicks < 400 ? 0.4 : 0.0);
        }

        @Override
        public void teleopInit() {
            stop();
            System.out.println("POWER_UP teleop");
        }

        @Override
        public void teleopPeriodic() {
            stop();
        }

        @Override
        public void disabledInit() {
            stop();
        }

        private void stop() {
            left.stopMotor();
            right.stopMotor();
            roller.stopMotor();
        }
    }
}
```

Build each project from its own directory with the WPILib 2027 alpha-7 JDK/toolchain:

```powershell
.\gradlew.bat build
```

Compilation is only half the job: without the following model, the discovered outputs would
run in loopback rather than move our chassis.

## 4. Save the C++ robot model

Create <code>samples\season_2027\cpp\.wpiemu\robot.json</code> in the scratch export.
The project section chooses the ABI; `gamePluginId` chooses the rules. Start pose puts the
fixture on the low-Y Blue plate for `RLR`. The chassis dimensions are full extents in metres.

<!-- book-file: samples\season_2027\cpp\.wpiemu\robot.json -->
```json
{
  "schemaVersion": 1,
  "name": "Power UP fixed-height cube delivery fixture",
  "project": {
    "language": "cpp",
    "wpilibYear": 2027,
    "gradleRioVersion": "2027.0.0-alpha-7"
  },
  "gamePluginId": "game_2027",
  "startPose": {"xM": 1.0, "yM": 2.7431998, "headingRad": 0.0},
  "chassis": {
    "sizeM": {"x": 0.7, "y": 0.7, "z": 0.2},
    "massKg": 50.0,
    "comHeightM": 0.15
  },
```

Each traction wheel gets its own discovered HAL PWM device. Positive throttle is forward
on both sides because wheel commands are defined in the wheel frame; there is no extra right-side
inversion in this fixture. Do not paste a hardware wiring inversion into both mappings and robot code.

<!-- book-file: samples\season_2027\cpp\.wpiemu\robot.json -->
```json
  "drivetrain": {
    "kind": "tank",
    "tireModel": "brush",
    "wheels": [
      {
        "name": "left", "kind": "standard",
        "positionM": {"x": 0.0, "y": 0.28, "z": 0.0508},
        "driveGearRatio": 10.71
      },
      {
        "name": "right", "kind": "standard",
        "positionM": {"x": 0.0, "y": -0.28, "z": 0.0508},
        "driveGearRatio": 10.71
      }
    ],
    "mappings": [
      {"owner": "drivetrain", "slot": "left.drive",
       "device": {"family": "hal", "type": "pwm", "id": 0, "bus": "rio", "channel": -1}},
      {"owner": "drivetrain", "slot": "right.drive",
       "device": {"family": "hal", "type": "pwm", "id": 1, "bus": "rio", "channel": -1}}
    ]
  },
```

The roller uses the generic mechanism's default rotary motion, with the chosen inertia and
reduction. Its owner name must match the activation condition we are about to add.

<!-- book-file: samples\season_2027\cpp\.wpiemu\robot.json -->
```json
  "mechanisms": [
    {
      "id": "roller", "type": "mechanism", "label": "Cube release roller",
      "properties": {"gearRatio": 2.0, "momentOfInertiaKgM2": 0.02},
      "mappings": [
        {"owner": "roller", "slot": "motor",
         "device": {"family": "hal", "type": "pwm", "id": 2, "bus": "rio", "channel": -1}}
      ]
    }
  ],
```

The release zone is chassis-mounted by default. Its pose is **relative to the chassis COM**,
not the floor. Above 1 rad/s, the plugin releases its one held cube from that point.
The fixed 5 ms step divides the robot's 20 ms callback period. The last section selects `RLR`
and requests a preload, which the plugin deducts from Blue's portal supply at reset.

<!-- book-file: samples\season_2027\cpp\.wpiemu\robot.json -->
```json
  "zones": [
    {
      "id": "release", "kind": "game_2027.release", "label": "Fixed-height delivery chute",
      "pose": {"position": {"x": 0.75, "y": 0.0, "z": 0.75}},
      "activation": {
        "mode": "all",
        "conditions": [
          {"mechanism": "roller", "value": "velocity", "op": "above",
           "threshold": 1.0, "absolute": true}
        ]
      }
    }
  ],
  "sim": {"stepUs": 5000, "seed": 1},
  "plugins": {
    "game_2027": {
      "schemaVersion": 1,
      "data": {"assignment": "RLR", "preload": true}
    }
  }
}
```

## 5. Save the Java model without changing its mechanics

Use the same model for Java, changing exactly three values: language `cpp` to `java`,
assignment `RLR` to `LRL`, and start Y to 5.4864 m. The following complete JSON is folded only
to avoid making you scroll through the same mappings twice. It is included in replay verification.

<details>
<summary>Complete Java robot model</summary>

<!-- book-file: samples\season_2027\java\.wpiemu\robot.json -->
```json
{
  "schemaVersion": 1,
  "name": "Power UP fixed-height cube delivery fixture",
  "project": {
    "language": "java",
    "wpilibYear": 2027,
    "gradleRioVersion": "2027.0.0-alpha-7"
  },
  "gamePluginId": "game_2027",
  "startPose": {"xM": 1.0, "yM": 5.4864, "headingRad": 0.0},
  "chassis": {
    "sizeM": {"x": 0.7, "y": 0.7, "z": 0.2},
    "massKg": 50.0,
    "comHeightM": 0.15
  },
  "drivetrain": {
    "kind": "tank",
    "tireModel": "brush",
    "wheels": [
      {
        "name": "left", "kind": "standard",
        "positionM": {"x": 0.0, "y": 0.28, "z": 0.0508},
        "driveGearRatio": 10.71
      },
      {
        "name": "right", "kind": "standard",
        "positionM": {"x": 0.0, "y": -0.28, "z": 0.0508},
        "driveGearRatio": 10.71
      }
    ],
    "mappings": [
      {"owner": "drivetrain", "slot": "left.drive",
       "device": {"family": "hal", "type": "pwm", "id": 0, "bus": "rio", "channel": -1}},
      {"owner": "drivetrain", "slot": "right.drive",
       "device": {"family": "hal", "type": "pwm", "id": 1, "bus": "rio", "channel": -1}}
    ]
  },
  "mechanisms": [
    {
      "id": "roller", "type": "mechanism", "label": "Cube release roller",
      "properties": {"gearRatio": 2.0, "momentOfInertiaKgM2": 0.02},
      "mappings": [
        {"owner": "roller", "slot": "motor",
         "device": {"family": "hal", "type": "pwm", "id": 2, "bus": "rio", "channel": -1}}
      ]
    }
  ],
  "zones": [
    {
      "id": "release", "kind": "game_2027.release", "label": "Fixed-height delivery chute",
      "pose": {"position": {"x": 0.75, "y": 0.0, "z": 0.75}},
      "activation": {
        "mode": "all",
        "conditions": [
          {"mechanism": "roller", "value": "velocity", "op": "above",
           "threshold": 1.0, "absolute": true}
        ]
      }
    }
  ],
  "sim": {"stepUs": 5000, "seed": 1},
  "plugins": {
    "game_2027": {
      "schemaVersion": 1,
      "data": {"assignment": "LRL", "preload": true}
    }
  }
}
```

</details>

**Inspection:** use Blue alliance for these fixtures. If you change the assignment, change start Y
with it; if you change the alliance, author the rotated start and corresponding routine.
Neither language automatically chooses a path from any starting position.

Next: [prove cube contacts, layout and reset on real physics](05b-physics-tests.md).
