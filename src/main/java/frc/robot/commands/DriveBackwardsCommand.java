package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.drive.Drive;

public class DriveBackwardsCommand extends Command {
    private final Drive drivetrain;
    private final Timer timer = new Timer();
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // Constants for the autonomous movement
    private static final double DRIVE_SPEED_MPS = 0.5; // 0.5 m/s backwards
    private static final double DRIVE_DISTANCE_M = 1.0; // 1 meter
    private static final double DRIVE_TIME_S = DRIVE_DISTANCE_M / DRIVE_SPEED_MPS; // 2 seconds

    public DriveBackwardsCommand(Drive drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
        System.out.println("Starting DriveBackwardsCommand - driving " + DRIVE_DISTANCE_M + "m backwards at "
                + DRIVE_SPEED_MPS + " m/s for " + DRIVE_TIME_S + " seconds");
    }

    @Override
    public void execute() {
        // Create chassis speeds for driving backwards (negative X velocity in robot
        // frame)
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-DRIVE_SPEED_MPS, 0.0, 0.0);

        // Apply the chassis speeds to the drivetrain
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(chassisSpeeds));

        // Log progress every 0.5 seconds
        if (timer.get() % 0.5 < 0.02) {
            double remainingTime = DRIVE_TIME_S - timer.get();
            double distanceTraveled = timer.get() * DRIVE_SPEED_MPS;
            System.out.println("DriveBackwards: " + String.format("%.2f", distanceTraveled) + "m / " + DRIVE_DISTANCE_M
                    + "m, " + String.format("%.1f", remainingTime) + "s remaining");
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DRIVE_TIME_S);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));

        if (interrupted) {
            System.out.println(
                    "DriveBackwardsCommand interrupted after " + String.format("%.2f", timer.get()) + " seconds");
        } else {
            System.out.println("DriveBackwardsCommand completed - drove backwards for "
                    + String.format("%.2f", timer.get()) + " seconds");
        }
    }
}
