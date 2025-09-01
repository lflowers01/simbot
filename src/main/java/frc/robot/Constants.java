package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Constants {
    public class constDrivetrain {
        public static final int JOYSTICK_PORT = 0;
        public static final double MAX_ANGULAR_RATE = 0.75;
        public static final double DEADBAND_PERCENT = 0.1;

        // Advanced Drive Control Constants
        public static final double DEADBAND = 0.1;
        public static final double HALF_SPEED_FACTOR = 0.35;
        public static final double ROTATION_ACTIVE_TIMEOUT = 0.1; // seconds
        public static final double ROTATION_ACTIVE_THRESHOLD_DEGREES = 5.0; // degrees
        public static final double INPUT_CURVE = 3.0; // Input exponent (1.0 = linear, 2.0 = squared, etc.)

        // Speed Control Constants
        public static final double MAX_SPEED = 4.5; // Maximum robot speed in m/s
        public static final double SPEED_MODIFIER = 1.0; // Speed modifier (0.0 to 1.0)

        // Demensions
        public static final double CHASSIS_WIDTH = Units.inchesToMeters(29.5);
        public static final double CHASSIS_LENGTH = Units.inchesToMeters(29.5);
    }

    public class constElevator {

        public static int LEAD_MOTOR_ID = 10;
        public static int FOLLOW_MOTOR_ID = 11;

        public static double KG = 0.0;
        public static double KS = 0.00;
        public static double KV = 0.05;
        public static double KA = 0.000;

        public static double KP = 3.37;
        public static double KI = 0.00;
        public static double KD = 0.06;

        public static double motionVelocity = 32.0; // m/s
        public static double motionAcceleration = 48.0; // m/s²

        public static double EXPO_KV = 0.04;
        public static double EXPO_KA = 0.005;

        public static double IDLE = 1;
        public static double L1 = 1.325;
        public static double L2 = 1.65;
        public static double L3 = 1.975;
        public static double L4 = 2.3;

        // Simulated parameters
        public static double simulationTick = 0.02; // Update every 20 ms
        public static double gearing = 6; // 8:48 reduction
        public static double carriageMass = Units.lbsToKilograms(6.081);
        public static double drumRadius = Units.inchesToMeters(1.538 / 2); // 1.538 inches diameter
        public static double minHeightMeters = Units.inchesToMeters(39.25);
        public static double maxHeightMeters = Units.inchesToMeters(93.75);
        public static double verticalOffset = Units.inchesToMeters(1.75); // Ground to bottom of elevator
        public static double horizontalOffset = Units.inchesToMeters(11.00); // Center of robot to elevator
        public static Color8Bit color = new Color8Bit(255, 0, 0);
        public static double lineWidth = 5;
        public static double ROTATIONS_PER_METER = gearing / (2 * Math.PI * drumRadius);
    }

}
