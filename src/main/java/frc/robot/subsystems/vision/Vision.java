// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private Pose2d latestAcceptedVisionPose = null;

    // Configurable triangle detection parameters
    private static final double DETECTION_TRIANGLE_ANGLE_DEGREES = 120.0; // Change this to adjust triangle angle

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    /**
     * Returns the best tag pose using alliance-specific filtering and isosceles triangle detection.
     * Creates isosceles triangles from each tag facing outward with configurable angle and maxTagDistance altitude.
     * Selects the tag whose triangle the robot is most centered within.
     * If no triangles contain the robot, falls back to the closest detected alliance tag.
     * 
     * @return The best tag pose, or null if no valid tags are found
     */
    public Pose3d getBestTagPose() {
        // Get alliance-specific tag IDs automatically
        int[] allianceTagIds;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                allianceTagIds = constVision.blueTagIds;
                System.out.println("Alliance: Blue - Looking for tags: " + java.util.Arrays.toString(allianceTagIds));
            } else {
                allianceTagIds = constVision.redTagIds;
                System.out.println("Alliance: Red - Looking for tags: " + java.util.Arrays.toString(allianceTagIds));
            }
        } else {
            // Default to all tags if alliance not available
            allianceTagIds = new int[constVision.blueTagIds.length + constVision.redTagIds.length];
            System.arraycopy(constVision.blueTagIds, 0, allianceTagIds, 0, constVision.blueTagIds.length);
            System.arraycopy(constVision.redTagIds, 0, allianceTagIds, constVision.blueTagIds.length,
                    constVision.redTagIds.length);
            System.out
                    .println("Alliance: Unknown - Looking for all tags: " + java.util.Arrays.toString(allianceTagIds));
        }

        // Use the observed pose from vision observations as robot position
        Pose2d currentRobotPose = null;

        // Find the most recent accepted pose observation to use as robot position
        double latestTimestamp = 0.0;
        for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
            if (!inputs[cameraIndex].connected)
                continue;

            for (var observation : inputs[cameraIndex].poseObservations) {
                if (observation.timestamp() > latestTimestamp &&
                        observation.tagCount() > 0 &&
                        observation.ambiguity() <= constVision.maxAmbiguity &&
                        Math.abs(observation.pose().getZ()) <= constVision.maxZError) {
                    latestTimestamp = observation.timestamp();
                    currentRobotPose = observation.pose().toPose2d();
                }
            }
        }

        // Fallback to stored vision pose if no current observations
        if (currentRobotPose == null) {
            currentRobotPose = getLatestVisionPose();
        }

        if (currentRobotPose == null) {
            System.out.println("No robot pose available for triangle calculation");
            return null;
        }

        System.out.println("Using robot pose for triangle calc: " + currentRobotPose);

        // Isosceles triangle parameters
        final double TRIANGLE_ANGLE_RADIANS = Math.toRadians(DETECTION_TRIANGLE_ANGLE_DEGREES);
        final double TRIANGLE_ALTITUDE = constVision.maxTagDistance; // Altitude from tag to opposite side
        final double HALF_TRIANGLE_ANGLE = TRIANGLE_ANGLE_RADIANS / 2.0;

        System.out.println("Triangle parameters: angle=" + DETECTION_TRIANGLE_ANGLE_DEGREES + "°, altitude=" + TRIANGLE_ALTITUDE + "m");

        Pose3d bestTagPose = null;
        double bestTriangleScore = 0.0; // Higher score = more centered in triangle
        int bestTagId = -1;
        int candidateCount = 0;

        // Check if robot is detected by any camera (must have valid observations)
        boolean hasValidObservations = false;
        for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
            if (!inputs[cameraIndex].connected)
                continue;

            for (var observation : inputs[cameraIndex].poseObservations) {
                if (observation.tagCount() > 0 &&
                        observation.ambiguity() <= constVision.maxAmbiguity &&
                        Math.abs(observation.pose().getZ()) <= constVision.maxZError) {
                    hasValidObservations = true;
                    break;
                }
            }
            if (hasValidObservations)
                break;
        }

        if (!hasValidObservations) {
            System.out.println("No valid vision observations available");
            return null;
        }

        // Track closest tag for fallback
        Pose3d closestTagPose = null;
        double closestTagDistance = Double.POSITIVE_INFINITY;
        int closestTagId = -1;

        // Loop through all alliance tags and check isosceles triangle intersections
        for (int tagId : allianceTagIds) {
            candidateCount++;

            // Get the tag pose from the AprilTag layout
            var tagPoseOptional = constVision.aprilTagLayout.getTagPose(tagId);
            if (!tagPoseOptional.isPresent()) {
                System.out.println("Tag " + tagId + ": Pose not found in layout, skipping");
                continue;
            }

            var tagPose = tagPoseOptional.get();

            // Check if tag is detected by any camera
            boolean tagDetected = false;
            for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
                if (!inputs[cameraIndex].connected)
                    continue;

                for (int detectedTagId : inputs[cameraIndex].tagIds) {
                    if (detectedTagId == tagId) {
                        tagDetected = true;
                        break;
                    }
                }
                if (tagDetected)
                    break;
            }

            if (!tagDetected) {
                System.out.println("Tag " + tagId + ": Not currently detected by any camera, skipping");
                continue;
            }

            // Calculate distance for fallback tracking
            var tagPosition = tagPose.getTranslation().toTranslation2d();
            var robotPosition = currentRobotPose.getTranslation();
            var tagToRobot = robotPosition.minus(tagPosition);
            double distanceToRobot = tagToRobot.getNorm();

            // Update closest tag for fallback
            if (distanceToRobot < closestTagDistance && distanceToRobot <= constVision.maxTagDistance) {
                closestTagDistance = distanceToRobot;
                closestTagPose = tagPose;
                closestTagId = tagId;
            }

            // Calculate isosceles triangle parameters
            var tagFacingDirection = tagPose.getRotation().toRotation2d(); // Tag faces in its +X direction

            // Check if robot is within triangle altitude (distance along the facing direction)
            if (distanceToRobot > TRIANGLE_ALTITUDE || distanceToRobot < 0.1) {
                System.out.println("Tag " + tagId + ": Robot distance invalid (" +
                        String.format("%.2f", distanceToRobot) + "m, range: 0.1-" + TRIANGLE_ALTITUDE + "m)");
                continue;
            }

            // Calculate angle from tag facing direction to robot
            var robotAngleFromTag = Math.atan2(tagToRobot.getY(), tagToRobot.getX());
            var angleDifferenceFromFacing = robotAngleFromTag - tagFacingDirection.getRadians();
            
            // Normalize angle difference to [-π, π]
            while (angleDifferenceFromFacing > Math.PI) {
                angleDifferenceFromFacing -= 2 * Math.PI;
            }
            while (angleDifferenceFromFacing < -Math.PI) {
                angleDifferenceFromFacing += 2 * Math.PI;
            }
            
            // Take absolute value for triangle check
            double absAngleDifference = Math.abs(angleDifferenceFromFacing);

            // For isosceles triangle: check if robot is within the triangle angle
            double maxAllowedAngle = HALF_TRIANGLE_ANGLE;
            
            if (absAngleDifference > maxAllowedAngle) {
                System.out.println("Tag " + tagId + ": Robot outside triangle (" +
                        String.format("%.1f", Math.toDegrees(absAngleDifference)) + "° > " +
                        String.format("%.1f", Math.toDegrees(maxAllowedAngle)) + "°)");
                continue;
            }

            // Robot is inside the isosceles triangle - calculate quality score
            double angleScore = 1.0 - (absAngleDifference / maxAllowedAngle); // 1.0 = perfectly centered, 0.0 = at edge
            double distanceScore = 1.0 - (distanceToRobot / TRIANGLE_ALTITUDE); // 1.0 = very close, 0.0 = at max distance

            // Combine scores with emphasis on being centered in the triangle
            double triangleScore = (angleScore * 0.7) + (distanceScore * 0.3);

            System.out.println("Tag " + tagId + ": INSIDE TRIANGLE - " +
                    "dist=" + String.format("%.2f", distanceToRobot) + "m, " +
                    "angle_diff=" + String.format("%.1f", Math.toDegrees(absAngleDifference)) + "°, " +
                    "triangle_score=" + String.format("%.3f", triangleScore) + " " +
                    "(angle:" + String.format("%.3f", angleScore) + " dist:" + String.format("%.3f", distanceScore) + ")");

            // Update best tag if this has a better triangle score
            if (triangleScore > bestTriangleScore) {
                bestTriangleScore = triangleScore;
                bestTagPose = tagPose;
                bestTagId = tagId;
                System.out.println("New best tag " + tagId + " with triangle score: " + String.format("%.3f", bestTriangleScore));
            }
        }

        System.out.println("Total alliance tags checked: " + candidateCount);

        // Check if we found a tag within a triangle
        if (bestTagPose != null) {
            System.out.println("Final best tag " + bestTagId + " with triangle score: " + String.format("%.3f", bestTriangleScore));
            return bestTagPose;
        }

        // Fallback: if robot not inside any triangle, use closest detected alliance tag
        if (closestTagPose != null) {
            System.out.println("FALLBACK: Robot not inside any triangle, using closest tag " + closestTagId + 
                    " at distance " + String.format("%.2f", closestTagDistance) + "m");
            return closestTagPose;
        }

        System.out.println("No valid alliance tags found (neither in triangles nor within max distance)");
        return null;
    }

    /**
     * Returns the latest vision-estimated robot pose from accepted observations.
     * 
     * @return The latest vision robot pose, or null if no valid vision data is
     *         available
     */

    public Pose2d getLatestVisionPose() {
        return latestAcceptedVisionPose;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

    // Initialize logging values
    // 
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d>allRobotPosesRejected=new LinkedList<>();

    // Loop over cameras 
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = constVision.aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > constVision.maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > constVision.maxZError // Must have realistic Z
                                                                                       // coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > constVision.aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > constVision.aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Update latest accepted pose
                latestAcceptedVisionPose = observation.pose().toPose2d();

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = constVision.linearStdDevBaseline * stdDevFactor;
                double angularStdDev = constVision.angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= constVision.linearStdDevMegatag2Factor;
                    angularStdDev *= constVision.angularStdDevMegatag2Factor;
                }
                if (cameraIndex < constVision.cameraStdDevFactors.length) {
                    linearStdDev *= constVision.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= constVision.cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));

            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
