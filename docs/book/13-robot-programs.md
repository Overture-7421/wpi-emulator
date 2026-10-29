# 13. Robot programs and saved models

## 13.1 Objectives

After this chapter the reader can connect PWM outputs to a drivetrain and a
release mechanism, explain a saved robot model, and build equivalent C++ and
Java programs. Chapter 12 is the prerequisite. The two programs and models
address R8 and R9.

## 13.2 Problem statement

A plugin test can move a chassis directly. A robot cannot: its outputs must
cross HALSim, the local IPC connection, device mapping and the drivetrain.
The demonstration therefore uses ordinary WPILib motor objects, without an
emulator library in the robot project.

## 13.3 Background and definitions

The season pack pins WPILib to 2027.0.0-alpha-7. In that preview, C++ uses
`wpi::TimedRobot`, `wpi::PWMSparkMax` and `wpi::MatchState::GetGameData()`.
Java uses the corresponding preview classes. These are not the older
`frc::TimedRobot` and `edu.wpi.first.wpilibj` APIs.

A saved model describes the simulated machine, not the robot program.
A mapping connects an announced device to a named simulation slot. The robot
declares PWM channel 2; the model maps that device to the roller's motor.
The release zone observes the resulting mechanism velocity.

| Output | Simulation owner and slot | Purpose |
|---|---|---|
| PWM 0 | drivetrain, left.drive | Left wheel drive |
| PWM 1 | drivetrain, right.drive | Right wheel drive |
| PWM 2 | roller, motor | Release activation |

## 13.4 Design

A driver-operated example would require joystick setup before its first test.
A timed autonomous routine instead gives a repeatable first experiment.
It drives forward for approximately six seconds, stops, and requests release
between seven and eight seconds. `TimedRobot` callbacks run at 20 ms; this is
distinct from the emulator's 5 ms physics tick.

**Invariant 13.1 (robot boundary).** The robot knows its motors and game data,
but not a pool index, plate counter or score API.

The example assumes the model's fixed-height delivery chute and a prepared
start pose. It is an integration fixture, not a competition autonomous routine.

## 13.5 Implementation

### 13.5.1 C++ program

Autonomous initialization validates the four legal assignments. An absent or
unexpected message prevents motion and release. Disabled and teleoperated
initialization stop every motor. The complete class appears in one listing.

Listing 13.1. The C++ robot program. <code>samples\season_2027\power_up_cpp\src\main\cpp\Robot.cpp</code>.

<!-- book-file: samples\season_2027\power_up_cpp\src\main\cpp\Robot.cpp -->
```cpp
#include <iostream>
#include <wpi/drivers/motor/PWMSparkMax.hpp>
#include <wpi/driverstation/MatchState.hpp>
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

int main() {
    return wpi::StartRobot<Robot>();
}
```

### 13.5.2 Java program

The Java program uses the same channels, callback counts and stopping behavior.
Language changes do not change the simulated robot or the game contract.

Listing 13.2. The equivalent Java robot program. <code>samples\season_2027\power_up_java\src\main\java\frc\robot\Main.java</code>.

<!-- book-file: samples\season_2027\power_up_java\src\main\java\frc\robot\Main.java -->
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

### 13.5.3 Model

The project section selects the pinned season. `startPose` places the chassis
opposite its assigned home-switch plate. `chassis` supplies mass and dimensions;
the tank drivetrain supplies wheel geometry, gearing and mappings. No motor
inversion is added to compensate for a second inversion elsewhere.

The roller is a generic mechanism with inertia, not a detailed intake.
Its activation condition uses absolute velocity above 1 rad/s. The zone's
position is relative to the chassis centre-of-mass frame. At a 0.15 m chassis
centre height, its local Z of 0.75 m places the release centre at 0.90 m.
Gravity then carries the released cube onto the plate.

Listing 13.3. The complete C++ saved model. <code>samples\season_2027\power_up_cpp\.wpiemu\robot.json</code>.

<!-- book-file: samples\season_2027\power_up_cpp\.wpiemu\robot.json -->
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
  "startPose": {
    "xM": 1.0,
    "yM": 2.7431998,
    "headingRad": 0.0
  },
  "chassis": {
    "sizeM": {
      "x": 0.7,
      "y": 0.7,
      "z": 0.2
    },
    "massKg": 50.0,
    "comHeightM": 0.15
  },
  "drivetrain": {
    "kind": "tank",
    "tireModel": "brush",
    "wheels": [
      {
        "name": "left",
        "kind": "standard",
        "positionM": {
          "x": 0.0,
          "y": 0.28,
          "z": 0.0508
        },
        "driveGearRatio": 10.71
      },
      {
        "name": "right",
        "kind": "standard",
        "positionM": {
          "x": 0.0,
          "y": -0.28,
          "z": 0.0508
        },
        "driveGearRatio": 10.71
      }
    ],
    "mappings": [
      {
        "owner": "drivetrain",
        "slot": "left.drive",
        "device": {
          "family": "hal",
          "type": "pwm",
          "id": 0,
          "bus": "rio",
          "channel": -1
        }
      },
      {
        "owner": "drivetrain",
        "slot": "right.drive",
        "device": {
          "family": "hal",
          "type": "pwm",
          "id": 1,
          "bus": "rio",
          "channel": -1
        }
      }
    ]
  },
  "mechanisms": [
    {
      "id": "roller",
      "type": "mechanism",
      "label": "Cube release roller",
      "properties": {
        "gearRatio": 2.0,
        "momentOfInertiaKgM2": 0.02
      },
      "mappings": [
        {
          "owner": "roller",
          "slot": "motor",
          "device": {
            "family": "hal",
            "type": "pwm",
            "id": 2,
            "bus": "rio",
            "channel": -1
          }
        }
      ]
    }
  ],
  "zones": [
    {
      "id": "release",
      "kind": "game_2027.release",
      "label": "Fixed-height delivery chute",
      "pose": {
        "position": {
          "x": 0.75,
          "y": 0.0,
          "z": 0.75
        }
      },
      "activation": {
        "mode": "all",
        "conditions": [
          {
            "mechanism": "roller",
            "value": "velocity",
            "op": "above",
            "threshold": 1.0,
            "absolute": true
          }
        ]
      }
    }
  ],
  "sim": {
    "stepUs": 5000,
    "seed": 1
  },
  "plugins": {
    "game_2027": {
      "schemaVersion": 1,
      "data": {
        "assignment": "RLR",
        "preload": true
      }
    }
  }
}
```

The Java model changes project language and the selected assignment/start
lane. It is listed in full so either project is independently reproducible.

Listing 13.4. The complete Java saved model. <code>samples\season_2027\power_up_java\.wpiemu\robot.json</code>.

<!-- book-file: samples\season_2027\power_up_java\.wpiemu\robot.json -->
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
  "startPose": {
    "xM": 1.0,
    "yM": 5.4864,
    "headingRad": 0.0
  },
  "chassis": {
    "sizeM": {
      "x": 0.7,
      "y": 0.7,
      "z": 0.2
    },
    "massKg": 50.0,
    "comHeightM": 0.15
  },
  "drivetrain": {
    "kind": "tank",
    "tireModel": "brush",
    "wheels": [
      {
        "name": "left",
        "kind": "standard",
        "positionM": {
          "x": 0.0,
          "y": 0.28,
          "z": 0.0508
        },
        "driveGearRatio": 10.71
      },
      {
        "name": "right",
        "kind": "standard",
        "positionM": {
          "x": 0.0,
          "y": -0.28,
          "z": 0.0508
        },
        "driveGearRatio": 10.71
      }
    ],
    "mappings": [
      {
        "owner": "drivetrain",
        "slot": "left.drive",
        "device": {
          "family": "hal",
          "type": "pwm",
          "id": 0,
          "bus": "rio",
          "channel": -1
        }
      },
      {
        "owner": "drivetrain",
        "slot": "right.drive",
        "device": {
          "family": "hal",
          "type": "pwm",
          "id": 1,
          "bus": "rio",
          "channel": -1
        }
      }
    ]
  },
  "mechanisms": [
    {
      "id": "roller",
      "type": "mechanism",
      "label": "Cube release roller",
      "properties": {
        "gearRatio": 2.0,
        "momentOfInertiaKgM2": 0.02
      },
      "mappings": [
        {
          "owner": "roller",
          "slot": "motor",
          "device": {
            "family": "hal",
            "type": "pwm",
            "id": 2,
            "bus": "rio",
            "channel": -1
          }
        }
      ]
    }
  ],
  "zones": [
    {
      "id": "release",
      "kind": "game_2027.release",
      "label": "Fixed-height delivery chute",
      "pose": {
        "position": {
          "x": 0.75,
          "y": 0.0,
          "z": 0.75
        }
      },
      "activation": {
        "mode": "all",
        "conditions": [
          {
            "mechanism": "roller",
            "value": "velocity",
            "op": "above",
            "threshold": 1.0,
            "absolute": true
          }
        ]
      }
    }
  ],
  "sim": {
    "stepUs": 5000,
    "seed": 1
  },
  "plugins": {
    "game_2027": {
      "schemaVersion": 1,
      "data": {
        "assignment": "LRL",
        "preload": true
      }
    }
  }
}
```

The new projects retain the existing probe projects. This helper copies their
unchanged Gradle scaffolding only where a destination file is absent. It never
replaces the program or model extracted from this chapter.

Listing 13.5. Reuse the pinned season's Gradle scaffolding. <code>seasons\2027\game\tools\copy_robots.py</code>.

<!-- book-file: seasons\2027\game\tools\copy_robots.py -->
```python
"""Supply unchanged Gradle scaffolding without replacing the book's robot listings."""
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[4]
samples = root / "samples" / "season_2027"
excluded = {"build", ".gradle", ".git", "logs", "bin", ".wpiemu"}
for language in ("cpp", "java"):
    source = samples / language
    destination = samples / ("power_up_" + language)
    if not source.is_dir():
        raise SystemExit(f"missing baseline sample: {source}")
    for path in source.rglob("*"):
        relative = path.relative_to(source)
        if any(part in excluded for part in relative.parts) or not path.is_file():
            continue
        target = destination / relative
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    print(destination)
```

The season manifest records both new projects alongside the standard probes.
This keeps the sample inventory complete without changing the existing projects.

Listing 13.6. Register the two Power UP robot projects in the season pack. <code>seasons\2027\season.json</code>.

<!-- book-edit: seasons\2027\season.json -->
```json
{
  "old": "    \"samples/season_2027/vision_java\",\n    \"samples/season_2027/vision_cpp\"\n  ]\n",
  "new": "    \"samples/season_2027/vision_java\",\n    \"samples/season_2027/vision_cpp\",\n    \"samples/season_2027/power_up_cpp\",\n    \"samples/season_2027/power_up_java\"\n  ]\n"
}
```

## 13.6 Verification

Building a robot establishes source and dependency compatibility. Chapter 14
adds the stronger test: real outputs must move the chassis, release the existing
preload, establish ownership and survive reset in both languages.

## 13.7 Reproduction

Extract through Chapter 13. From the disposable export, run the scaffold helper
before either Gradle build. On Windows, use the Visual Studio environment from
[Building](../BUILDING.md) and the season's JDK:

```powershell
python seasons\2027\game\tools\copy_robots.py
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk'
Push-Location samples\season_2027\power_up_cpp
.\gradlew.bat build --no-daemon
Pop-Location
Push-Location samples\season_2027\power_up_java
.\gradlew.bat build --no-daemon
Pop-Location
```

For an interactive run, build the desktop application, open either new project,
load its saved model, and start simulation. Select Blue station 1 and run a
timed match. The assignment and start lane must remain consistent.

## 13.8 Discussion

The model's chute substitutes for a physical elevator. The ordinary output
path is exercised, but elevator design is not inferred from that result.
F-24 distinguishes this fixture from a legal competition robot.

## 13.9 Summary

Both programs express the same behavior through the pinned robot API. The
saved model explains how ordinary PWM outputs acquire physical meaning.
No scoring operation has been added to the robot.

## 13.10 Exercises

1. Calculate the nominal autonomous release interval from the callback counts.
2. Explain why changing PWM 2 in the program without changing its mapping
   prevents release.
3. Determine the start-lane change required when Blue's home-switch assignment
   changes from L to R.

[Chapter 14](14-verification.md)
