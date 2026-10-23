package frc.robot;

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
