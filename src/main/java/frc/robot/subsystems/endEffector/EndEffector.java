package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        System.out.println("EndEffector subsystem initialized with PID position control");
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }

    /**
     * Sets the end effector to a specific position using PID
     * 
     * @param position The target position in motor rotations
     */
    public void setPosition(double position) {
        io.setPosition(position);
        System.out.println("EndEffector: Setting position to " + position + " rotations");
    }

    /**
     * Gets the current position of the end effector
     * 
     * @return Current position in motor rotations
     */
    public double getPosition() {
        return inputs.position;
    }

    /**
     * Gets the target position that was last commanded
     * 
     * @return Target position in motor rotations
     */
    public double getTargetPosition() {
        return inputs.targetPosition;
    }

    /**
     * Checks if the end effector is at the target position within tolerance
     * 
     * @param toleranceRotations Tolerance in motor rotations
     * @return true if within tolerance
     */
    public boolean atTarget(double toleranceRotations) {
        double currentPos = getPosition();
        double targetPos = getTargetPosition();

        // Safety checks for invalid data
        if (Double.isNaN(currentPos) || Double.isNaN(targetPos) ||
                Double.isInfinite(currentPos) || Double.isInfinite(targetPos)) {
            System.out.println("EndEffector: Invalid position data - current=" + currentPos + ", target=" + targetPos);
            return false; // Don't claim we're at target if data is invalid
        }

        // Ensure tolerance is positive and reasonable
        double safeTolerance = Math.max(0.01, Math.abs(toleranceRotations));

        boolean result = Math.abs(currentPos - targetPos) <= safeTolerance;

        return result;
    }

    /**
     * Gets the current velocity of the end effector
     * 
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return inputs.velocityRotPerSec;
    }

    /**
     * Stops the end effector by clearing any position hold
     */
    public void stop() {
        io.clearPositionHold();
        System.out.println("EndEffector: Position hold cleared");
    }

    /**
     * Legacy method for compatibility - converts to position-based movement
     */
    public void spinForDuration(double speed, double durationSeconds) {
        // Convert to position-based movement
        double currentPosition = getPosition();
        double targetPosition = currentPosition + (speed * durationSeconds * 0.5); // Adjust multiplier as needed

        System.out.println("EndEffector: Legacy spin converted to position movement: " +
                currentPosition + " -> " + targetPosition + " rotations");
        setPosition(targetPosition);
    }

    /**
     * Legacy method for compatibility
     */
    public void stopMotor() {
        stop();
    }

    /**
     * Legacy method for compatibility
     */
    public boolean isSpinning() {
        return Math.abs(getVelocity()) > 0.1; // Consider moving if velocity > 0.1 rot/sec
    }

    /**
     * Legacy method for compatibility
     */
    public double getRemainingTime() {
        return 0.0; // Position control doesn't have a fixed duration
    }

    /**
     * Sets the outtake motor speed for coral manipulation
     * 
     * @param speed Speed from -1.0 to 1.0 (negative = intake, positive = outtake)
     */
    public void setOuttakeSpeed(double speed) {
        // Validate speed input
        if (Double.isNaN(speed) || Double.isInfinite(speed)) {
            System.out.println("EndEffector: Invalid outtake speed " + speed + ", setting to 0");
            speed = 0.0;
        }

        // Clamp speed to safe range
        speed = Math.max(-1.0, Math.min(1.0, speed));

        io.setOuttakeSpeed(speed);
        if (Math.abs(speed) > 0.01) {
            String action = speed > 0 ? "outtaking" : "intaking";
            System.out.println("EndEffector: " + action + " coral at speed " + String.format("%.2f", speed));
        }
    }

    /**
     * Start coral intake
     */
    public void startIntake() {
        setOuttakeSpeed(Constants.constEndEffector.intakeSpeed);
        System.out.println("EndEffector: Starting coral intake");
    }

    /**
     * Start coral outtake/scoring
     */
    public void startOuttake() {
        setOuttakeSpeed(Constants.constEndEffector.outtakeSpeed);
        System.out.println("EndEffector: Starting coral outtake");
    }

    /**
     * Stop coral intake/outtake
     */
    public void stopOuttake() {
        setOuttakeSpeed(Constants.constEndEffector.stopSpeed);
        System.out.println("EndEffector: Stopping coral motor");
    }

    /**
     * Gets the current outtake motor speed
     * 
     * @return Current outtake speed from -1.0 to 1.0
     */
    public double getOuttakeSpeed() {
        return inputs.outtakeSpeed;
    }

    /**
     * Check if coral motor is running
     * 
     * @return true if motor is running (speed > threshold)
     */
    public boolean isOuttakeRunning() {
        return Math.abs(getOuttakeSpeed()) > 0.1;
    }
}
