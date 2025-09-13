package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.constAutoAlign;
import frc.robot.Constants.constAutoAlignTrajectory;
import frc.robot.Constants.constAutoAlignController;
import frc.robot.Constants.constAutoAlignLogging;
import frc.robot.Constants.constDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class AutoAlignCommand extends Command {
        public enum AlignmentSide {
                LEFT, RIGHT, STATION_LEFT, STATION_RIGHT
        }

        private final Drive drivetrain;
        private final Vision vision;
        private final HolonomicDriveController controller;
        private final Trajectory trajectory;
        private final Pose2d goalPose;
        private final Timer timer = new Timer();
        private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        // Move the static variable to class level for exponential smoothing
        private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

        /**
         * Creates a smooth trajectory from current pose to target pose with gentle
         * curves
         */
        private Trajectory createLinearTrajectory(Pose2d currentPose, Pose2d targetPose) {
                // Check if poses are too close together
                double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

                if (distance < constAutoAlignTrajectory.minTrajectoryDistance) {
                        System.out.println("Poses too close for trajectory generation. Distance: " + distance + "m");
                        // Create a simple stationary trajectory - don't try to generate a path
                        TrajectoryConfig config = new TrajectoryConfig(
                                        constAutoAlignTrajectory.slowTrajectorySpeed,
                                        constAutoAlignTrajectory.slowTrajectorySpeed);

                        // Create a minimal movement trajectory by moving slightly forward then to
                        // target
                        // Convert rotation to a translation vector
                        double x = currentPose.getX() + Math.cos(currentPose.getRotation().getRadians())
                                        * constAutoAlignTrajectory.minMovementDistance;
                        double y = currentPose.getY() + Math.sin(currentPose.getRotation().getRadians())
                                        * constAutoAlignTrajectory.minMovementDistance;

                        Pose2d intermediateNose = new Pose2d(x, y, currentPose.getRotation());

                        return TrajectoryGenerator.generateTrajectory(
                                        currentPose,
                                        List.of(intermediateNose.getTranslation()),
                                        targetPose,
                                        config);
                }

                // Create aggressive trajectory config with high speeds
                TrajectoryConfig config = new TrajectoryConfig(
                                constDrivetrain.maxSpeed * constAutoAlign.speedMod
                                                * constAutoAlignTrajectory.velocityMultiplier,
                                constDrivetrain.maxSpeed * constAutoAlign.speedMod
                                                * constAutoAlignTrajectory.accelerationMultiplier);

                // Create intermediate waypoint that prioritizes rotation first
                // Calculate waypoint position for rotation-biased movement
                double translationX = currentPose.getX()
                                + (targetPose.getX() - currentPose.getX()) * constAutoAlignTrajectory.translationBias;
                double translationY = currentPose.getY()
                                + (targetPose.getY() - currentPose.getY()) * constAutoAlignTrajectory.translationBias;

                // Interpolate rotation more aggressively for rotation-first movement
                Rotation2d intermediateRotation = currentPose.getRotation().interpolate(targetPose.getRotation(),
                                constAutoAlignTrajectory.rotationBias);

                Pose2d rotationBiasedWaypoint = new Pose2d(translationX, translationY, intermediateRotation);

                // Generate trajectory with rotation-biased waypoint - only pass Translation2d
                // for waypoints
                return TrajectoryGenerator.generateTrajectory(
                                currentPose, // Start pose
                                List.of(rotationBiasedWaypoint.getTranslation()), // Only the translation component
                                targetPose, // End pose - hit target directly
                                config // Aggressive trajectory configuration
                );
        }

        public AutoAlignCommand(Drive drivetrain, Vision vision, Pose3d targetPose, AlignmentSide alignmentSide) {
                this.drivetrain = drivetrain;
                this.vision = vision;
                addRequirements(drivetrain);

                // Store the target pose - this will NOT change during command execution
                System.out.println("=== Auto-Align Command Created ===");
                System.out.println("Target tag pose locked in: " + targetPose);
                System.out.println("Alignment side: " + alignmentSide);

                // More aggressive PID controller gains for precise trajectory following
                this.controller = new HolonomicDriveController(
                                new PIDController(constAutoAlignController.translationKP,
                                                constAutoAlignController.translationKI,
                                                constAutoAlignController.translationKD), // X controller
                                new PIDController(constAutoAlignController.translationKP,
                                                constAutoAlignController.translationKI,
                                                constAutoAlignController.translationKD), // Y controller
                                new ProfiledPIDController(constAutoAlignController.rotationKP,
                                                constAutoAlignController.rotationKI,
                                                constAutoAlignController.rotationKD, // Rotation controller
                                                new TrapezoidProfile.Constraints(
                                                                constDrivetrain.maxSpeed * constAutoAlign.speedMod
                                                                                * constAutoAlignController.maxRotationSpeedMultiplier,
                                                                constDrivetrain.maxSpeed * constAutoAlign.speedMod
                                                                                * constAutoAlignController.maxRotationAccelerationMultiplier)));

                // Get current robot pose from vision odometry, fallback to drivetrain if no
                // vision data
                Pose2d currentRobotPose = vision.getLatestVisionPose();
                if (currentRobotPose == null) {
                        currentRobotPose = drivetrain.getPose();
                        System.out.println("No vision pose available, using drivetrain odometry");
                }

                // Check if robot is within alignment distance of the target tag
                double distanceToTag = currentRobotPose.getTranslation()
                                .getDistance(targetPose.getTranslation().toTranslation2d());

                if (distanceToTag > constAutoAlign.maxAlignmentDistance) {
                        System.out.println("ERROR: Robot too far from tag for alignment!");
                        System.out.println("Distance: " + String.format("%.2f", distanceToTag) + "m > " +
                                        String.format("%.2f", constAutoAlign.maxAlignmentDistance) + "m (max)");
                        System.out.println("Auto-align command will not execute - move closer to the tag");

                        // Create a dummy trajectory that immediately finishes
                        this.goalPose = currentRobotPose; // Stay in current position
                        this.trajectory = createLinearTrajectory(currentRobotPose, currentRobotPose);
                        return;
                }

                System.out.println("Distance check passed: " + String.format("%.2f", distanceToTag) + "m <= " +
                                String.format("%.2f", constAutoAlign.maxAlignmentDistance) + "m");

                // Select the appropriate goal offset based on alignment side
                var goalOffset = switch (alignmentSide) {
                        case LEFT -> constAutoAlign.goalOffsetLeft;
                        case RIGHT -> constAutoAlign.goalOffsetRight;
                        case STATION_LEFT, STATION_RIGHT -> constAutoAlign.goalOffsetStation;
                };

                // Apply the offset relative to the TAG pose to position robot facing the tag
                // FLOOR-CONSTRAINED: Ignore all pitch/roll from tag, only use X,Y,yaw
                var goalPose3d = targetPose.plus(goalOffset);

                // Extract only floor-relevant components (X, Y, yaw) - SIMPLIFIED approach
                this.goalPose = new Pose2d(
                                goalPose3d.getX(),
                                goalPose3d.getY(),
                                new Rotation2d(goalPose3d.getRotation().getZ()) // Only yaw, ignore roll/pitch
                );

                System.out.println("=== Auto-Align Debug (FLOOR-CONSTRAINED) ===");
                System.out.println("Current robot pose: " + currentRobotPose);
                System.out.println("Tag pose (floor-constrained): " + new Pose2d(targetPose.getX(), targetPose.getY(),
                                new Rotation2d(targetPose.getRotation().getZ())));
                System.out.println("Tag yaw only (degrees): " + Math.toDegrees(targetPose.getRotation().getZ()));
                System.out.println("Goal offset (" + alignmentSide + "): " + goalOffset);
                System.out.println("FINAL goal pose (floor-constrained): " + goalPose);
                System.out.println("Goal yaw only (degrees): " + goalPose.getRotation().getDegrees());
                System.out.println("Distance to travel: "
                                + currentRobotPose.getTranslation().getDistance(goalPose.getTranslation()) + "m");

                // Create trajectory once with the FIXED target - this will not change
                this.trajectory = createLinearTrajectory(currentRobotPose, goalPose);

                System.out.println("Trajectory created with " + trajectory.getStates().size() + " states, " +
                                "duration: " + String.format("%.2f", trajectory.getTotalTimeSeconds()) + "s");

                // Log first and last trajectory states for debugging
                if (trajectory.getStates().size() > 0) {
                        var firstState = trajectory.getStates().get(0);
                        var lastState = trajectory.getStates().get(trajectory.getStates().size() - 1);
                        System.out.println("Trajectory start: " + String.format("(%.3f, %.3f, %.1f°)",
                                        firstState.poseMeters.getX(), firstState.poseMeters.getY(),
                                        firstState.poseMeters.getRotation().getDegrees()));
                        System.out.println("Trajectory end: " + String.format("(%.3f, %.3f, %.1f°)",
                                        lastState.poseMeters.getX(), lastState.poseMeters.getY(),
                                        lastState.poseMeters.getRotation().getDegrees()));
                }
        }

        // Constructor overload for station alignment with elevator control
        public AutoAlignCommand(Drive drivetrain, Vision vision, Pose3d targetPose, AlignmentSide alignmentSide,
                        frc.robot.subsystems.elevator.Elevator elevator) {
                // Call the main constructor
                this(drivetrain, vision, targetPose, alignmentSide);

                // For station alignments, set elevator to idle position
                if (alignmentSide == AlignmentSide.STATION_LEFT || alignmentSide == AlignmentSide.STATION_RIGHT) {
                        System.out.println("Station alignment detected - setting elevator to idle position");
                        elevator.setHeight(frc.robot.Constants.constElevator.idle);
                }
        }

        @Override
        public void initialize() {
                timer.restart();
        }

        @Override
        public void execute() {
                // Sample the trajectory at the current time
                Trajectory.State goalTrajectory = trajectory.sample(timer.get());

                // Get current robot pose - prioritize vision, fallback to drivetrain
                Pose2d currentRobotPose = vision.getLatestVisionPose();
                if (currentRobotPose == null) {
                        var drivetrainPose = drivetrain.getPose();
                        currentRobotPose = new Pose2d(
                                        drivetrainPose.getX(),
                                        drivetrainPose.getY(),
                                        drivetrainPose.getRotation());
                }

                // Calculate chassis speeds with gentle controller
                ChassisSpeeds autoAlignInputs = controller.calculate(
                                currentRobotPose, goalTrajectory, goalPose.getRotation());
                // least
                // 2cm/s

                // Apply deadband to reduce jitter from small commands
                final double TRANSLATION_DEADBAND = 0.005; // 5mm/s deadband
                final double ROTATION_DEADBAND = Math.toRadians(0.5); // 0.5°/s deadband

                // Apply deadband
                if (Math.abs(autoAlignInputs.vxMetersPerSecond) < TRANSLATION_DEADBAND) {
                        autoAlignInputs.vxMetersPerSecond = 0.0;
                }
                if (Math.abs(autoAlignInputs.vyMetersPerSecond) < TRANSLATION_DEADBAND) {
                        autoAlignInputs.vyMetersPerSecond = 0.0;
                }
                if (Math.abs(autoAlignInputs.omegaRadiansPerSecond) < ROTATION_DEADBAND) {
                        autoAlignInputs.omegaRadiansPerSecond = 0.0;
                }

                // Update last speeds for next iteration
                lastSpeeds = new ChassisSpeeds(autoAlignInputs.vxMetersPerSecond,
                                autoAlignInputs.vyMetersPerSecond,
                                autoAlignInputs.omegaRadiansPerSecond);

                // Apply the smooth speeds to the drivetrain
                drivetrain.setControl(applyRobotSpeeds.withSpeeds(autoAlignInputs));

        }

        @Override
        public boolean isFinished() {
                return timer.hasElapsed(trajectory.getTotalTimeSeconds());
        }

        @Override
        public void end(boolean interrupted) {
                // Stop the drivetrain by applying zero speeds
                drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
        }

}
