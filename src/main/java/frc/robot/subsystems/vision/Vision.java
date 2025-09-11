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
     * Returns the best tag pose using alliance-specific filtering.
     * Finds the closest unobstructed tag that is in the allowed alliance tags list.
     * Biases towards tags that are facing more directly towards the robot.
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
        // This gives us the most accurate position for distance calculations
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
            System.out.println("No robot pose available for distance calculation");
            return null;
        }

        System.out.println("Using robot pose for distance calc: " + currentRobotPose);

        Pose3d bestTagPose = null;
        double bestScore = Double.POSITIVE_INFINITY;
        int bestTagId = -1;
        int candidateCount = 0;

        // Loop through all cameras
        for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
            if (!inputs[cameraIndex].connected) {
                System.out.println("Camera " + cameraIndex + " not connected, skipping");
                continue;
            }

            System.out.println("Camera " + cameraIndex + ": " + inputs[cameraIndex].tagIds.length +
                    " tags detected, " + inputs[cameraIndex].poseObservations.length + " observations");

            // Check each detected tag ID for alliance membership and get its pose
            for (int tagId : inputs[cameraIndex].tagIds) {
                candidateCount++;

                // Check if this tag is in the allowed alliance list
                boolean isAllowedTag = false;
                for (int allowedId : allianceTagIds) {
                    if (tagId == allowedId) {
                        isAllowedTag = true;
                        break;
                    }
                }

                if (!isAllowedTag) {
                    System.out.println("Tag " + tagId + ": Not an alliance tag, skipping");
                    continue;
                }

                // Get the tag pose from the AprilTag layout
                var tagPoseOptional = constVision.aprilTagLayout.getTagPose(tagId);
                if (!tagPoseOptional.isPresent()) {
                    System.out.println("Tag " + tagId + ": Pose not found in layout, skipping");
                    continue;
                }

                var tagPose = tagPoseOptional.get();

                // Calculate direct distance from robot to tag (2D distance for consistency)
                double distanceFromRobot = currentRobotPose.getTranslation()
                        .getDistance(tagPose.getTranslation().toTranslation2d());

                // Skip if too far
                if (distanceFromRobot > constVision.maxTagDistance) {
                    System.out.println("Tag " + tagId + ": Too far (" +
                            String.format("%.2f", distanceFromRobot) + "m > " + constVision.maxTagDistance + "m)");
                    continue;
                }

                // Find the best observation of this tag from this camera
                double bestObservationScore = Double.POSITIVE_INFINITY;
                for (var observation : inputs[cameraIndex].poseObservations) {
                    // Skip invalid observations
                    if (observation.tagCount() == 0 ||
                            observation.ambiguity() > constVision.maxAmbiguity ||
                            Math.abs(observation.pose().getZ()) > constVision.maxZError ||
                            observation.averageTagDistance() > constVision.maxTagDistance) {
                        continue;
                    }

                    // Calculate tag facing angle (how well the tag faces toward the robot)
                    var tagFacingAngle = tagPose.getRotation().toRotation2d().getRadians();
                    var tagToRobot = currentRobotPose.getTranslation()
                            .minus(tagPose.getTranslation().toTranslation2d());
                    var angleFromTag = Math.atan2(tagToRobot.getY(), tagToRobot.getX());

                    var angleDifference = Math.abs(tagFacingAngle - angleFromTag);
                    if (angleDifference > Math.PI) {
                        angleDifference = 2 * Math.PI - angleDifference;
                    }

                    // Normalize angle factor (0.0 = facing directly, 1.0 = facing away)
                    double angleFactor = angleDifference / Math.PI;

                    // Calculate scoring components - balance the weights better
                    double distanceComponent = distanceFromRobot * constVision.distanceWeight;
                    double skewComponent = angleFactor * constVision.skewWeight;
                    double ambiguityComponent = observation.ambiguity() * constVision.ambiguityWeight;
                    // Multi-tag bonus should reduce score (negative component)
                    double multiTagComponent = -(observation.tagCount() > 1 ? constVision.multiTagWeight : 0.0);

                    double observationScore = distanceComponent + skewComponent + ambiguityComponent
                            + multiTagComponent;

                    if (observationScore < bestObservationScore) {
                        bestObservationScore = observationScore;
                    }
                }

                if (bestObservationScore == Double.POSITIVE_INFINITY) {
                    System.out.println("Tag " + tagId + ": No valid observations found");
                    continue;
                }

                System.out.println("Tag " + tagId + ": " +
                        "dist=" + String.format("%.2f", distanceFromRobot) + "m, " +
                        "score=" + String.format("%.2f", bestObservationScore));

                // Update best tag if this has a better score
                if (bestObservationScore < bestScore) {
                    bestScore = bestObservationScore;
                    bestTagPose = tagPose;
                    bestTagId = tagId;
                    System.out.println("New best tag " + tagId + " with score: " + String.format("%.2f", bestScore));
                }
            }
        }

        System.out.println("Total tags checked: " + candidateCount);

        if (bestTagPose != null) {
            System.out.println("Final best tag " + bestTagId + " with score: " + String.format("%.2f", bestScore));
        } else {
            System.out.println("No valid alliance tags found");
        }

        return bestTagPose;
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
