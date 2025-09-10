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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.constAutoAlign;
import frc.robot.Constants.constDrivetrain;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class AutoAlignCommand extends Command {
        private final Drive drivetrain;
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
                // Create trajectory config with speed and acceleration limits
                TrajectoryConfig config = new TrajectoryConfig(
                                constDrivetrain.maxSpeed * constAutoAlign.speedMod, // Max velocity (50% of max speed)
                                constDrivetrain.maxSpeed * constAutoAlign.speedMod * 0.5 // Max acceleration (25% of max
                                                                                         // speed)
                );

                // Generate simple linear trajectory
                return TrajectoryGenerator.generateTrajectory(
                                currentPose, // Start pose
                                List.of(), // No interior waypoints for linear path
                                targetPose, // End pose
                                config // Trajectory configuration
                );
        }

        public AutoAlignCommand(Drive drivetrain, Pose3d targetPose) {
                this.drivetrain = drivetrain;
                addRequirements(drivetrain);

                this.controller = new HolonomicDriveController(
                                new PIDController(1, 0, 0), new PIDController(1, 0, 0),
                                new ProfiledPIDController(1, 0, 0,
                                                new TrapezoidProfile.Constraints(
                                                                constDrivetrain.maxSpeed * constAutoAlign.speedMod,
                                                                constDrivetrain.maxSpeed * constAutoAlign.speedMod
                                                                                * 0.5)));

                this.goalPose = new Pose2d(
                                targetPose.getTranslation().toTranslation2d()
                                                .plus(constAutoAlign.goalOffset.getTranslation().toTranslation2d()),
                                targetPose.getRotation().toRotation2d()
                                                .plus(constAutoAlign.goalOffset.getRotation().toRotation2d()));

                this.trajectory = createLinearTrajectory(drivetrain.getPose(), goalPose);

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
                ChassisSpeeds autoAlignInputs = controller.calculate(
                                drivetrain.getPose(), goalTrajectory, goalPose.getRotation());

                // Apply the speeds to the drivetrain using robot-centric control
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
