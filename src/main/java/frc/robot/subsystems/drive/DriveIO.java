package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        // Robot pose (required for AdvantageScope)
        public Pose2d pose = new Pose2d();

        // Robot velocities in m/s and rad/s
        public double velocityX = 0.0;
        public double velocityY = 0.0;
        public double velocityOmegaRadPerSec = 0.0;

        // Swerve module states (measured) - AdvantageScope format
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        // Swerve module targets (setpoints) - AdvantageScope format
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        // Swerve module positions - AdvantageScope format
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        // Heading control state
        public boolean hasCurrentHeading = false;
        public double currentHeadingDegrees = 0.0;
        public double rotationLastTriggered = 0.0;

        // Odometry frequency for diagnostics
        public double odometryFrequency = 0.0;
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }
}
