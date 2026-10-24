package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import java.util.Locale;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.bus.CANPort;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    public static final class Robot extends TimedRobot {
        private static final double kEncoderOffsetRot = 0.125;
        private final CANBus canBus = new CANBus(CANPort.CAN_S2);
        private final TalonFX motor = new TalonFX(1, canBus);
        private final CANcoder encoder = new CANcoder(2, canBus);
        private final Pigeon2 gyro = new Pigeon2(3, canBus);
        private final DutyCycleOut output = new DutyCycleOut(0.35);
        private int sampleCount;

        public Robot() {
            TalonFXConfiguration motorConfig = new TalonFXConfiguration();
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            requireOk("TalonFX apply", motor.getConfigurator().apply(motorConfig));
            TalonFXConfiguration motorReadback = new TalonFXConfiguration();
            requireOk("TalonFX refresh", motor.getConfigurator().refresh(motorReadback));

            CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
            encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffsetRot;
            encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            requireOk("CANcoder apply", encoder.getConfigurator().apply(encoderConfig));
            CANcoderConfiguration encoderReadback = new CANcoderConfiguration();
            requireOk("CANcoder refresh", encoder.getConfigurator().refresh(encoderReadback));

            requireOk("Pigeon2 apply", gyro.getConfigurator().apply(new Pigeon2Configuration()));
            requireOk("Pigeon2 yaw", gyro.setYaw(22.5));
            System.out.printf(Locale.ROOT,
                    "CTRE27_CONFIG motorInverted=%s encoderOffsetRot=%.6f encoderDirection=%s%n",
                    motorReadback.MotorOutput.Inverted,
                    encoderReadback.MagnetSensor.MagnetOffset,
                    encoderReadback.MagnetSensor.SensorDirection);
        }

        @Override
        public void teleopInit() {
            System.out.println("CTRE27_TELEOP");
        }

        @Override
        public void teleopPeriodic() {
            motor.setControl(output);
        }

        @Override
        public void robotPeriodic() {
            if (++sampleCount % 10 == 0) {
                System.out.printf(Locale.ROOT,
                        "CTRE27_SAMPLE duty=%.6f volts=%.6f rotorPosition=%.6f rotorVelocity=%.6f "
                                + "encoderPosition=%.6f encoderVelocity=%.6f yaw=%.6f%n",
                        motor.getDutyCycle().refresh().getValueAsDouble(),
                        motor.getMotorVoltage().refresh().getValueAsDouble(),
                        motor.getRotorPosition().refresh().getValueAsDouble(),
                        motor.getRotorVelocity().refresh().getValueAsDouble(),
                        encoder.getPosition().refresh().getValueAsDouble(),
                        encoder.getVelocity().refresh().getValueAsDouble(),
                        gyro.getYaw().refresh().getValueAsDouble());
            }
        }

        @Override
        public void disabledInit() {
            motor.stopMotor();
            System.out.println("CTRE27_DISABLED");
        }

        private static void requireOk(String operation, StatusCode status) {
            if (!status.isOK()) {
                throw new IllegalStateException(operation + " failed: " + status);
            }
        }
    }
}
