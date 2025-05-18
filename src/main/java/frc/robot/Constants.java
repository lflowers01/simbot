package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class Constants {
    class constDrivetrain {

    }

    public class constArm {
        public static final double L0 = 0.5;
        public static final double L1 = 0.75;
        public static final double L2 = 0.75;

        public class shoulder {
            public static TalonFXConfiguration SHOULDER_CONFIG = new TalonFXConfiguration();
            static {
                SHOULDER_CONFIG.Slot0.kG = 0.5; // Volts to overcome gravity
                SHOULDER_CONFIG.Slot0.kS = 0.4; // Volts to overcome static friction
                SHOULDER_CONFIG.Slot0.kV = 0.05; // Volts for a velocity target of 1 rps
                SHOULDER_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/
                SHOULDER_CONFIG.Slot0.kP = 3;
                SHOULDER_CONFIG.Slot0.kI = 0.0;
                SHOULDER_CONFIG.Slot0.kD = 0.05;

                SHOULDER_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
                SHOULDER_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
                SHOULDER_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.04;
                SHOULDER_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.005;

                SHOULDER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
                SHOULDER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
                SHOULDER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
                SHOULDER_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;
            }
        }
    }

    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    public static final double kArmReduction = 1;
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-75);
    public static final double kMaxAngleRads = Units.degreesToRadians(255);

}
