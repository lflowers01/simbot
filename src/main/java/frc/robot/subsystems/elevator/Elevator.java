package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    // Debounce state for atTarget() to avoid oscillation from sensor jitter
    private double atTargetSince = Double.NEGATIVE_INFINITY;
    private static final double AT_TARGET_DEBOUNCE_SECONDS = 0.10; // 100 ms stable required
    private static final double AT_TARGET_TOLERANCE_METERS = 0.02; // keep same tolerance

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Keep essential logging for elevator - needed for safety and diagnostics
        Logger.processInputs("Elevator", inputs);
    }

    public void setHeight(double meters) {
        // Validate height input
        if (Double.isNaN(meters) || Double.isInfinite(meters)) {
            System.out.println("Elevator: Invalid height " + meters + ", ignoring command");
            return;
        }

        // Bounds checking against constants
        double clampedHeight = Math.max(constElevator.minHeightMeters,
                Math.min(constElevator.maxHeightMeters, meters));

        if (clampedHeight != meters) {
            System.out.println("Elevator: Height " + meters + "m clamped to " + clampedHeight + "m");
        }

        io.setHeight(clampedHeight);
    }

    public double getHeight() {
        return inputs.positionMeters;
    }

    public double getTargetHeight() {
        return inputs.targetMeters;
    }

    public double getVelocity() {
        return inputs.velocityMetersPerSec;
    }

    // Debounced atTarget() to avoid small oscillations causing repeated movement
    // commands
    public boolean atTarget() {
        boolean within = Math.abs(inputs.positionMeters - inputs.targetMeters) < AT_TARGET_TOLERANCE_METERS;
        double now = Timer.getFPGATimestamp();
        if (within) {
            if (atTargetSince == Double.NEGATIVE_INFINITY) {
                atTargetSince = now;
            }
            return (now - atTargetSince) >= AT_TARGET_DEBOUNCE_SECONDS;
        } else {
            atTargetSince = Double.NEGATIVE_INFINITY;
            return false;
        }
    }

    /**
     * Checks if the elevator is at its target height within tolerance
     * 
     * @param toleranceMeters Tolerance in meters
     * @return true if at target
     */
    public boolean atTarget(double toleranceMeters) {
        double currentHeight = getHeight();
        double targetHeight = getTargetHeight();

        // Safety checks for invalid data
        if (Double.isNaN(currentHeight) || Double.isNaN(targetHeight) ||
                Double.isInfinite(currentHeight) || Double.isInfinite(targetHeight)) {
            System.out.println("Elevator: Invalid height data - current=" + currentHeight + ", target=" + targetHeight);
            return false; // Don't claim we're at target if data is invalid
        }

        // Ensure tolerance is positive and reasonable
        double safeTolerance = Math.max(0.005, Math.abs(toleranceMeters)); // Minimum 5mm tolerance

        boolean result = Math.abs(currentHeight - targetHeight) <= safeTolerance;

        return result;
    }

    /**
     * Gets the EndEffector angle based on current elevator height
     * 
     * @return EndEffector position for current elevator height
     */
    public double getEndEffectorAngleForHeight() {
        double currentHeight = getHeight();

        // Check which level we're closest to
        if (Math.abs(currentHeight - constElevator.l2) < 0.1) {
            return Constants.constEndEffector.l2Position;
        } else if (Math.abs(currentHeight - constElevator.l3) < 0.1) {
            return Constants.constEndEffector.l3Position;
        } else if (Math.abs(currentHeight - constElevator.l4) < 0.1) {
            return Constants.constEndEffector.l4Position;
        } else {
            // Default to stow position for L1 or idle
            return Constants.constEndEffector.stowPosition;
        }
    }
}
