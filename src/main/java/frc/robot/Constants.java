package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Constants {
    public class constDrivetrain {
        public static final int joystickPort = 0;
        public static final double maxAngularRate = 0.75;
        public static final double deadbandPercent = 0.1;

        // Advanced Drive Control Constants
        public static final double deadband = 0.1;
        public static final double halfSpeedFactor = 0.35;
        public static final double rotationActiveTimeout = 0.1; // seconds
        public static final double rotationActiveThresholdDegrees = 5.0; // degrees
        public static final double inputCurve = 3.0; // Input exponent (1.0 = linear, 2.0 = squared, etc.)

        // Speed Control Constants
        public static final double maxSpeed = 4.5; // Maximum robot speed in m/s
        public static final double speedModifier = 1.0; // Speed modifier (0.0 to 1.0)

        // Dimensions
        public static final double chassisWidth = Units.inchesToMeters(29.5);
        public static final double chassisLength = Units.inchesToMeters(29.5);
    }

    public class constElevator {
        public static int leadMotorId = 10;
        public static int followMotorId = 11;

        public static double kG = 0.0;
        public static double kS = 0.00;
        public static double kV = 0.05;
        public static double kA = 0.000;

        public static double kP = 3.37;
        public static double kI = 0.00;
        public static double kD = 0.06;

        public static double motionVelocity = 32.0; // m/s
        public static double motionAcceleration = 48.0; // m/s²

        public static double expoKV = 0.04;
        public static double expoKA = 0.005;

        public static double idle = 1;
        public static double l1 = 1.325;
        public static double l2 = 1.65;
        public static double l3 = 1.975;
        public static double l4 = 2.3;

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
        public static double rotationsPerMeter = gearing / (2 * Math.PI * drumRadius);
    }

    public class constVision {
        // Basic filtering thresholds
        public static final double maxAmbiguity = 0.4;
        public static final double maxZError = 0.3; // Meters

        // Standard deviation baselines, for 1 meter distance and 1 tag
        public static final double linearStdDevBaseline = 0.01; // Meters
        public static final double angularStdDevBaseline = 5.0; // Degrees

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        // Camera names, must match names configured on coprocessor
        public static final String camera0Name = "camera_0";
        public static final String camera1Name = "camera_1";

        // Robot to camera transforms
        public static final Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2,
                new Rotation3d(0.0, -0.4, 0.0));
        public static final Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2,
                new Rotation3d(0.0, -0.4, Math.PI));

        // Standard deviation multipliers for each camera
        public static final double[] cameraStdDevFactors = new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
        };

        // Multipliers to apply for MegaTag 2 observations
        public static final double linearStdDevMegatag2Factor = 0.5;
        public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
    }
}
