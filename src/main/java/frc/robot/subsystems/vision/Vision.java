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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    /*
     * VISION SYSTEM ARCHITECTURE:
     * 
     * 1. ODOMETRY FUNCTION (periodic() method):
     * - Vision observations of AprilTags are used to update robot odometry
     * - Camera detections provide pose corrections via VisionConsumer
     * - This keeps the robot's position estimate accurate over time
     * 
     * 2. AUTO-ALIGNMENT FUNCTION (getBestTagPose() method):
     * - Uses LAYOUT POSES (ground truth) for triangle geometry and target selection
     * - Uses ODOMETRY POSITION to determine which tag triangle the robot is within
     * - Does NOT require current vision - purely odometry-based positioning
     * - Returns layout pose as navigation target for auto-alignment
     * 
     * Data Flow: Vision Observations → Odometry Updates → Triangle Intersection →
     * Layout Target
     */

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
     * AUTO-ALIGNMENT FUNCTION:
     * Returns the best tag pose using alliance-specific filtering and isosceles
     * triangle detection.
     * Creates isosceles triangles from each tag's LAYOUT POSE (not estimated)
     * facing outward with configurable angle and maxTagDistance altitude.
     * Selects the tag whose triangle the robot is most centered within based on
     * ODOMETRY position.
     * Does NOT require tags to be currently visible - uses layout poses and robot
     * odometry for triangle intersection.
     * If no triangles contain the robot, falls back to the closest alliance tag.
     * 
     * @return The best tag LAYOUT POSE (from AprilTag layout) for auto-alignment,
     *         or null if no valid tags are found
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

                    // Create floor-constrained pose from observation (ignore roll/pitch)
                    var originalPose = observation.pose();
                    currentRobotPose = new Pose2d(
                            originalPose.getX(),
                            originalPose.getY(),
                            new Rotation2d(originalPose.getRotation().getZ()) // Only yaw
                    );
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
        final double TRIANGLE_ANGLE_RADIANS = Math.toRadians(constVision.detectionTriangleAngleDegrees);
        final double TRIANGLE_ALTITUDE = constVision.maxTagDistance; // Altitude from tag to opposite side
        final double HALF_TRIANGLE_ANGLE = TRIANGLE_ANGLE_RADIANS / 2.0;

        System.out.println("Triangle parameters: angle=" + constVision.detectionTriangleAngleDegrees + "°, altitude="
                + TRIANGLE_ALTITUDE + "m");

        Pose3d bestTagPose = null;
        double bestTriangleScore = 0.0; // Higher score = more centered in triangle
        int bestTagId = -1;
        int candidateCount = 0;

        // Track closest tag for fallback (using layout poses only)
        Pose3d closestTagLayoutPose = null;
        double closestTagDistance = Double.POSITIVE_INFINITY;
        int closestTagId = -1;

        // Loop through all alliance tags and check isosceles triangle intersections
        // NOTE: We check ALL alliance tags regardless of whether they're currently
        // detected
        for (int tagId : allianceTagIds) {
            candidateCount++;

            // Get the LAYOUT tag pose (ground truth position, not estimated)
            var tagLayoutPoseOptional = constVision.aprilTagLayout.getTagPose(tagId);
            if (!tagLayoutPoseOptional.isPresent()) {
                System.out.println("Tag " + tagId + ": Layout pose not found, skipping");
                continue;
            }

            var tagLayoutPose = tagLayoutPoseOptional.get();

            // Calculate distance for fallback tracking using LAYOUT POSE
            var tagLayoutPosition = tagLayoutPose.getTranslation().toTranslation2d();
            var robotPosition = currentRobotPose.getTranslation();
            var tagToRobot = robotPosition.minus(tagLayoutPosition);
            double distanceToRobot = tagToRobot.getNorm();

            // Update closest tag for fallback using LAYOUT POSE
            if (distanceToRobot < closestTagDistance && distanceToRobot <= constVision.maxTagDistance) {
                closestTagDistance = distanceToRobot;
                closestTagLayoutPose = tagLayoutPose;
                closestTagId = tagId;
            }

            // Calculate isosceles triangle parameters using LAYOUT POSE (floor-constrained)
            var tagLayoutFacingDirection = new Rotation2d(tagLayoutPose.getRotation().getZ()); // Only yaw from layout

            System.out.println("Tag " + tagId + ": Checking triangle intersection - " +
                    "pos=" + String.format("(%.2f, %.2f)", tagLayoutPosition.getX(), tagLayoutPosition.getY()) +
                    ", facing=" + String.format("%.1f°", Math.toDegrees(tagLayoutFacingDirection.getRadians())));

            // Check if robot is within triangle altitude (distance from layout position)
            if (distanceToRobot > TRIANGLE_ALTITUDE || distanceToRobot < 0.1) {
                System.out.println("Tag " + tagId + ": Robot distance invalid (" +
                        String.format("%.2f", distanceToRobot) + "m, range: 0.1-" + TRIANGLE_ALTITUDE + "m)");
                continue;
            }

            // Calculate angle from tag LAYOUT facing direction to robot
            var robotAngleFromTag = Math.atan2(tagToRobot.getY(), tagToRobot.getX());
            var angleDifferenceFromFacing = robotAngleFromTag - tagLayoutFacingDirection.getRadians();

            // Normalize angle difference to [-π, π]
            while (angleDifferenceFromFacing > Math.PI) {
                angleDifferenceFromFacing -= 2 * Math.PI;
            }
            while (angleDifferenceFromFacing < -Math.PI) {
                angleDifferenceFromFacing += 2 * Math.PI;
            }

            // Take absolute value for triangle check
            double absAngleDifference = Math.abs(angleDifferenceFromFacing);

            // For isosceles triangle: check if robot is within the triangle angle (from
            // LAYOUT pose)
            double maxAllowedAngle = HALF_TRIANGLE_ANGLE;

            if (absAngleDifference > maxAllowedAngle) {
                System.out.println("Tag " + tagId + ": Robot outside triangle (" +
                        String.format("%.1f", Math.toDegrees(absAngleDifference)) + "° > " +
                        String.format("%.1f", Math.toDegrees(maxAllowedAngle)) + "°)");
                continue;
            }

            // Robot is inside the isosceles triangle - calculate quality score
            double angleScore = 1.0 - (absAngleDifference / maxAllowedAngle); // 1.0 = perfectly centered, 0.0 = at edge
            double distanceScore = 1.0 - (distanceToRobot / TRIANGLE_ALTITUDE); // 1.0 = very close, 0.0 = at max
                                                                                // distance

            // Combine scores with emphasis on being centered in the triangle
            double triangleScore = (angleScore * 0.7) + (distanceScore * 0.3);

            System.out.println("Tag " + tagId + ": INSIDE TRIANGLE (odometry-based) - " +
                    "dist=" + String.format("%.2f", distanceToRobot) + "m, " +
                    "angle_diff=" + String.format("%.1f", Math.toDegrees(absAngleDifference)) + "°, " +
                    "triangle_score=" + String.format("%.3f", triangleScore) + " " +
                    "(angle:" + String.format("%.3f", angleScore) + " dist:" + String.format("%.3f", distanceScore)
                    + ")");

            // Update best tag if this has a better triangle score (return LAYOUT pose)
            if (triangleScore > bestTriangleScore) {
                bestTriangleScore = triangleScore;
                bestTagPose = tagLayoutPose; // Use LAYOUT pose as target
                bestTagId = tagId;
                System.out.println(
                        "New best tag " + tagId + " with triangle score: " + String.format("%.3f", bestTriangleScore));
            }
        }

        System.out.println("Total alliance tags checked: " + candidateCount);

        // Check if we found a tag within a triangle (return LAYOUT pose)
        if (bestTagPose != null) {
            System.out.println("Final best tag " + bestTagId + " with triangle score: " +
                    String.format("%.3f", bestTriangleScore) + " (returning LAYOUT pose, no vision required)");
            return bestTagPose;
        }

        // Fallback: if robot not inside any triangle, use closest alliance tag (LAYOUT
        // pose)
        if (closestTagLayoutPose != null) {
            System.out.println("FALLBACK: Robot not inside any triangle, using closest tag " + closestTagId +
                    " at distance " + String.format("%.2f", closestTagDistance) + "m (returning LAYOUT pose)");
            return closestTagLayoutPose;
        }

        System.out.println("No alliance tags found within max distance");
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

    /**
     * ODOMETRY FUNCTION:
     * Processes vision observations and sends pose corrections to the robot's
     * odometry system.
     * This is the PRIMARY vision function - keeping robot position estimate
     * accurate.
     */
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
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

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

                // Create floor-constrained pose (ignore roll AND pitch, only keep yaw)
                var originalPose = observation.pose();
                var originalRotation = originalPose.getRotation();

                // Extract only the yaw (Z rotation) from the 3D rotation, ignore roll and pitch
                double yawRadians = originalRotation.getZ();

                var floorConstrainedPose = new Pose2d(
                        originalPose.getX(),
                        originalPose.getY(),
                        new Rotation2d(yawRadians) // Only keep yaw rotation, ignore roll/pitch completely
                );

                // Convert floor-constrained 2D pose back to 3D for logging (Z=0, no roll/pitch)
                var floorConstrainedPose3d = new Pose3d(
                        floorConstrainedPose.getX(),
                        floorConstrainedPose.getY(),
                        0.0, // Z = 0 (on floor)
                        new Rotation3d(0, 0, yawRadians) // Only yaw rotation, no roll/pitch
                );

                // Add pose to log - use floor-constrained pose for logging
                robotPoses.add(floorConstrainedPose3d); // Log floor-constrained instead of original
                if (rejectPose) {
                    robotPosesRejected.add(floorConstrainedPose3d);
                } else {
                    robotPosesAccepted.add(floorConstrainedPose3d);
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Update latest accepted pose (floor-constrained) - ONLY when we have a new
                // accepted pose
                // This prevents odometry flashing when no new poses are received
                latestAcceptedVisionPose = floorConstrainedPose;

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

                // Send vision observation (floor-constrained)
                consumer.accept(
                        floorConstrainedPose,
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
