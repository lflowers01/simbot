package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double targetMeters = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Set elevator target height in meters. */
    public default void setHeight(double meters) {} 
}
