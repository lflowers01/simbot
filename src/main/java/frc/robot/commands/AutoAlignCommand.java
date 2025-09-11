package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import java.util.List;

public class AutoAlignCommand extends Command {
        public enum AlignmentSide {
                LEFT, RIGHT
        }

        private final Drive drivetrain;
        private final Vision vision;
        private final HolonomicDriveController controller;
        private final Trajectory trajectory;
        private final Pose2d goalPose;
        private final Timer timer = new Timer();
        private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        /**
         * Creates a simple linear trajectory from current pose to target pose
         * 
         * @return A trajectory from current robot pose to target pose
         * @param targetPose The desired end pose
         * @return A trajectory from current robot pose to target pose
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

                // Select the appropriate goal offset based on alignment side
                var goalOffset = (alignmentSide == AlignmentSide.LEFT) ? constAutoAlign.goalOffsetLeft
                                : constAutoAlign.goalOffsetRight;

                // Apply the offset relative to the TAG pose to position robot facing the tag
                // This goalPose is now FIXED and will not change during command execution
                this.goalPose = targetPose.plus(goalOffset).toPose2d();

                System.out.println("=== Auto-Align Debug ===");
                System.out.println("Current robot pose: " + currentRobotPose);
                System.out.println("Tag pose: " + targetPose.toPose2d());
                System.out.println("Tag rotation (degrees): " + targetPose.getRotation().toRotation2d().getDegrees());
                System.out.println("Goal offset (" + alignmentSide + "): " + goalOffset);
                System.out.println("FIXED goal pose (robot target): " + goalPose);
                System.out.println("Goal rotation (degrees): " + goalPose.getRotation().getDegrees());
                System.out.println("Distance to travel: "
                                + currentRobotPose.getTranslation().getDistance(goalPose.getTranslation()) + "m");

                // Create trajectory once with the FIXED target - this will not change
                this.trajectory = createLinearTrajectory(currentRobotPose, goalPose);

                System.out.println("Trajectory created with " + trajectory.getStates().size() + " states, " +
                                "duration: " + String.format("%.2f", trajectory.getTotalTimeSeconds()) + "s");
        }

        @Override
        public void initialize() {
                timer.restart();
        }

        @Override
        public void execute() {
                // Sample the trajectory at the current time
                Trajectory.State goalTrajectory = trajectory.sample(timer.get());

                // Calculate chassis speeds to follow the trajectory
                // IMPORTANT: Use the FIXED goalPose rotation, not a dynamic target
                ChassisSpeeds autoAlignInputs = controller.calculate(
                                drivetrain.getPose(), goalTrajectory, goalPose.getRotation());

                // Apply the speeds to the drivetrain using robot-centric control
                drivetrain.setControl(applyRobotSpeeds.withSpeeds(autoAlignInputs));

                // Optional: Log progress at regular intervals
                if (timer.get() % constAutoAlignLogging.logInterval < constAutoAlignLogging.logTolerance) {
                        System.out.println("Auto-align progress: " +
                                        String.format("%.1f", timer.get()) + "s / " +
                                        String.format("%.1f", trajectory.getTotalTimeSeconds()) + "s, " +
                                        "current pose: " + drivetrain.getPose());
                }
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
