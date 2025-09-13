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
        public static int leadMotorId = 22;
        public static int followMotorId = 21;

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
        public static double horizontalOffset = Units.inchesToMeters(29.5); // Center of robot to elevator
        public static Color8Bit color = new Color8Bit(255, 0, 0);
        public static double lineWidth = 5;
        public static double rotationsPerMeter = gearing / (2 * Math.PI * drumRadius);
    }

    public class constVision {
        // Basic filtering thresholds
        public static final double maxAmbiguity = 0.4;
        public static final double maxZError = 0.3; // Meters

        // Tag filtering for auto-alignment
        public static final double maxTagDistance = 5.0; // Maximum distance to consider tags (meters)
        public static final int[] blueTagIds = { 17, 18, 19, 20, 21, 22 };
        public static final int[] redTagIds = { 6, 7, 8, 9, 10, 11 };

        public enum StationTag {
            BLUE_LEFT(13),
            BLUE_RIGHT(12),
            RED_LEFT(1),
            RED_RIGHT(2);

            public final int id;

            StationTag(int id) {
                this.id = id;
            }
        }

        // Triangle detection parameters for auto-alignment
        public static final double detectionTriangleAngleDegrees = 120.0; // Angle of triangle detection zones for tag
                                                                          // selection

        // Tag selection scoring weights (all values represent penalty/bonus in meters)
        public static final double distanceWeight = 2.0; // Weight for distance component (1.0 = 1 meter = 1 point)
        public static final double skewWeight = 3.5; // Weight for tag orientation penalty (higher = more bias towards
                                                     // front-facing tags)
        public static final double ambiguityWeight = 0.25; // Weight for ambiguity penalty (higher = more penalty for
                                                           // uncertain detections)
        public static final double multiTagWeight = 0; // Weight for multi-tag bonus (negative = bonus, positive =
                                                       // penalty)

        // Standard deviation baselines, for 1 meter distance and 1 tag
        public static final double linearStdDevBaseline = 0.01; // Meters
        public static final double angularStdDevBaseline = 5.0; // Degrees

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        // Camera names, must match names configured on coprocessor
        public static final String camera0Name = "camera_0";
        public static final String camera1Name = "camera_1";

        // Robot to camera transforms - SIMPLIFIED for testing
        // Left camera (camera_0): Front-left of robot
        // Right camera (camera_1): Front-right of robot
        // Using simple positions for testing - 12" forward, ±12" sideways, 9.3" up
        public static final Transform3d robotToCamera0 = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45))); // Look forward-left

        public static final Transform3d robotToCamera1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45))); // Look forward-right

        // Standard deviation multipliers for each camera
        public static final double[] cameraStdDevFactors = new double[] {
                2.0, // Camera 0
                2.0 // Camera 1
        };

        // Multipliers to apply for MegaTag 2 observations
        public static final double linearStdDevMegatag2Factor = 0.5;
        public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

        // Camera simulation properties
        public static final int cameraFPS = 30;
        public static final int cameraResolutionWidth = 800;
        public static final int cameraResolutionHeight = 600;
        public static final double cameraFOVDegrees = 70.0;
    }

    public class constAutoAlign {
        // Position robot 18 inches in front of the tag, facing the tag
        public static final Transform3d goalOffsetLeft = new Transform3d(
                new Translation3d(Units.inchesToMeters(18), Units.inchesToMeters(-6.5), 0), // 18 inches back from tag
                                                                                            // (in tag's -X direction)
                new Rotation3d(0, 0, Math.toRadians(180))); // 180 degrees rotation to face the tag
        public static final Transform3d goalOffsetRight = new Transform3d(
                new Translation3d(Units.inchesToMeters(18), Units.inchesToMeters(6.5), 0),
                new Rotation3d(0, 0, Math.toRadians(180)));
        public static final Transform3d goalOffsetStation = new Transform3d(
                new Translation3d(Units.inchesToMeters(18), Units.inchesToMeters(0), 0),
                new Rotation3d(0, 0, 0));
        public static final double speedMod = 0.4; // Increased for smoother movement (was 0.005)

        // Maximum distance to allow alignment with any tag
        public static final double maxAlignmentDistance = 5.0; // 5 meters maximum alignment distance
    }

    public class constAutoAlignTrajectory {
        // Minimum distance threshold
        public static final double minTrajectoryDistance = 0.03; // 3cm minimum distance for trajectory generation
        public static final double minMovementDistance = 0.015; // 5cm forward movement for close poses

        // SMOOTH trajectory speeds - much slower and more controlled
        public static final double velocityMultiplier = 1; // Much slower (was 1.5)
        public static final double accelerationMultiplier = 1.2; // Much gentler acceleration (was 1.2)
        public static final double slowTrajectorySpeed = 0.3; // Very gentle for minimal movements

        // Smoother waypoint generation - less aggressive movement
        public static final double translationBias = 0.2; // More gradual translation (was 0.2)
        public static final double rotationBias = 0.8; // Less aggressive rotation bias (was 0.8)
    }

    public class constAutoAlignController {
        // MUCH gentler PID gains for smooth movement
        public static final double translationKP = 1.0; // Much lower P gain (was 5.0)
        public static final double translationKI = 0.0; // No integral term to avoid oscillation
        public static final double translationKD = 0.0; // Light damping (was 0.5)

        // Gentler rotation PID gains
        public static final double rotationKP = 2.0; // Much lower P gain (was 8.5)
        public static final double rotationKI = 0.0; // No integral term
        public static final double rotationKD = 0.0; // Light damping (was 0.8)

        // MUCH slower rotation profile constraints
        public static final double maxRotationSpeedMultiplier = 2.0; // Much slower (was 2.5)
        public static final double maxRotationAccelerationMultiplier = 1.0; // Much gentler (was 1.5)
    }

    public class constAutoAlignLogging {
        public static final double logInterval = 0.5; // Log progress every 0.5 seconds
        public static final double logTolerance = 0.02; // Tolerance for log timing
    }

    public class constAutoScore {
        public static final double scoringDelaySeconds = 0.75; // Time to wait during scoring operation (fallback)
        public static final double elevatorMovementTimeoutSeconds = 2.0; // Max time to wait for elevator movement
        public static final double completionRadiusMeters = Units.inchesToMeters(24); // Distance from tag to trigger
                                                                                      // scoring completion
                                                                                      // and
                                                                                      // return to idle
    }
}
