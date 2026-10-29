# Weeks 2–3: cubes, see-saws, and the match clock

**In 30 seconds:** the plugin owns the rules, but the emulator owns time. Read the Driver Station's
mode and match time; don't assume that every practice match is exactly 15 seconds of autonomous and
135 seconds of teleop.

## 1. Turn physics events into game events

The game-state step follows the physics step. That gives it fresh contacts and sensor overlaps.
Keep that order: physics answers “what touched what this tick?” and the plugin answers “did that
touch score?” Use a small, explicit transition for cube pickup, delivery, vault deposit, and reset.
Test each transition with entry, exit, duplicate events, and reset.

A one-robot simulation cannot model the opponent alliance's actual robot and human player. Treat
opponent scoring and human-fed cubes as explicit scenario inputs or documented constants. Do not
pretend that an absent robot won an interaction because the score code needed a tie-breaker.

## 2. The match clock belongs to the Driver Station

The emulator's Practice profile has editable durations; its defaults differ from Power UP's
15-second autonomous and 135-second teleop. Use the Driver Station mode and match time from the
simulation context to decide which rules are active. Derive autonomous, teleop, endgame, and
disabled behavior from that input. Do not create a second timer that drifts away from the simulated
clock.

Award ownership points while an alliance owns its switch or the scale: 2 points per second during
autonomous and 1 point per second during teleop. In a discrete simulation, accumulate the actual
tick duration only while the scoring condition is true. Freeze scoring while disabled and clear
match-local accumulators on reset.

**Student asks:** “Why did our 15-second autonomous last 20 seconds?” Because Practice mode is a
practice configuration, not the game manual. The referee listens to the Driver Station; the
referee does not overrule it with a stopwatch.

## 3. Send game data before autonomous starts

Implement `IGameState::gameSpecificMessage()` from the reset state. The runtime forwards it through
`DsState` to the robot HAL simulation before the first autonomous tick. The field management
system's three-character message describes the own switch, scale, and far switch from the alliance's
point of view. Power UP uses `LLL`, `LRL`, `RLR`, and `RRR`; the first and third characters match.
Both alliances receive the same message. Set it at reset, before robot autonomous initialization,
and return no message only when the scenario genuinely has no assignment.

This is the season-specific contract between referee and robot. In 2027 alpha-7 Java,
`org.wpilib.driverstation.MatchState.getGameData()` returns an `Optional<String>`. The C++ API is
`wpi::MatchState::GetGameData()` from `wpi/driverstation/MatchState.hpp`, returning
`std::optional<std::string>`. These are **not** the 2026 `DriverStation.getGameSpecificMessage()`
APIs. A year upgrade is not a rename exercise; compile against the selected season's headers.

## 4. A complete Java robot sample

This complete `Main.java` replaces the source file in the vendor-free 2027 Java probe. It reads the
message once in `autonomousInit`, then sets one PWM output to a small positive or negative value
based on the near-switch assignment. It is an API probe, not a drivetrain or a cube-scoring routine.
The source was built with the pinned alpha-7 toolchain.

```java
package frc.robot;

import org.wpilib.driverstation.MatchState;
import org.wpilib.drivers.motor.PWMSparkMax;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.discrete.DigitalInput;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    public static final class Robot extends TimedRobot {
        private final PWMSparkMax motor = new PWMSparkMax(0);
        private final DigitalInput limit = new DigitalInput(1);
        private double autonomousThrottle;

        @Override
        public void autonomousInit() {
            String gameData = MatchState.getGameData().orElse("");
            autonomousThrottle = switch (gameData) {
                case "LLL", "LRL" -> 0.15;
                case "RLR", "RRR" -> -0.15;
                default -> 0.0;
            };
        }

        @Override
        public void autonomousPeriodic() {
            motor.setThrottle(autonomousThrottle);
        }

        @Override
        public void teleopInit() {
            System.out.println("SEASON_SAMPLE teleop");
        }

        @Override
        public void teleopPeriodic() {
            motor.setThrottle(limit.get() ? 0.25 : 0.5);
        }

        @Override
        public void disabledInit() {
            motor.stopMotor();
        }
    }
}
```

## 5. The same check in C++

This is the complete `Robot.cpp` replacement for the 2027 C++ probe. It uses the same four legal
messages and the same safe default. The single motor output is not a drivetrain. Do not copy a Java
Optional example into C++ and hope the compiler is feeling generous.

```cpp
#include <iostream>

#include <wpi/driverstation/MatchState.hpp>
#include <wpi/drivers/motor/PWMSparkMax.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/framework/TimedRobot.hpp>
#include <wpi/hardware/discrete/DigitalInput.hpp>

class Robot final : public wpi::TimedRobot {
public:
    void AutonomousInit() override {
        const auto gameData = wpi::MatchState::GetGameData();
        if (!gameData) {
            autonomousThrottle_ = 0.0;
        } else if (*gameData == "LLL" || *gameData == "LRL") {
            autonomousThrottle_ = 0.15;
        } else if (*gameData == "RLR" || *gameData == "RRR") {
            autonomousThrottle_ = -0.15;
        } else {
            autonomousThrottle_ = 0.0;
        }
    }

    void AutonomousPeriodic() override { motor_.SetThrottle(autonomousThrottle_); }
    void TeleopInit() override { std::cout << "SEASON_SAMPLE teleop" << std::endl; }
    void TeleopPeriodic() override { motor_.SetThrottle(limit_.Get() ? 0.25 : 0.5); }
    void DisabledInit() override { motor_.StopMotor(); }

private:
    wpi::PWMSparkMax motor_{0};
    wpi::DigitalInput limit_{1};
    double autonomousThrottle_ = 0.0;
};

int main() { return wpi::StartRobot<Robot>(); }
```

Build each project from its own directory with the WPILib 2027 alpha-7 toolchain:

```powershell
.\gradlew.bat build
```

**Going deeper:** for a real autonomous routine, validate the entire message against the four allowed
strings and make the robot's choice explicit. A missing or malformed value is not a secret promise
that the switch is on the left.

## You now have…

The referee can distinguish autonomous from teleop, the robot can receive the field assignment
before autonomous starts, and both language examples compile against the 2027 API. Next, we count
points and describe the controls without teaching Qt about Power UP.

Next: [Week 4 and 5—scoring, editor metadata, and robot interactions](04-scoring-and-features.md).
