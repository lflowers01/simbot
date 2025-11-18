package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import edu.wpi.first.wpilibj.Timer;

public class SetEndEffectorPositionCommand extends Command {
    private final EndEffector endEffector;
    private final double targetPosition;
    private final double tolerance;
    private final Timer timeoutTimer = new Timer();
    private final Timer progressTimer = new Timer();
    private static final double COMMAND_TIMEOUT_SECONDS = 3.0; // Reduced timeout for faster debugging

    /**
     * Command to move EndEffector to a specific position
     * 
     * @param endEffector    The EndEffector subsystem
     * @param targetPosition Target position in rotations
     */
    public SetEndEffectorPositionCommand(EndEffector endEffector, double targetPosition) {
        this(endEffector, targetPosition, 0.1); // Default 0.1 rotation tolerance
    }

    /**
     * Command to move EndEffector to a specific position with custom tolerance
     * 
     * @param endEffector    The EndEffector subsystem
     * @param targetPosition Target position in rotations
     * @param tolerance      Position tolerance in rotations
     */
    public SetEndEffectorPositionCommand(EndEffector endEffector, double targetPosition, double tolerance) {
        this.endEffector = endEffector;
        this.targetPosition = targetPosition;
        this.tolerance = Math.max(0.01, Math.abs(tolerance)); // Ensure minimum tolerance to prevent infinite loops

        addRequirements(endEffector);

        System.out.println("SetEndEffectorPositionCommand created: target=" + targetPosition +
                " rotations, tolerance=" + this.tolerance + " rotations");
    }

    @Override
    public void initialize() {
        System.out.println("=== SetEndEffectorPositionCommand INITIALIZE ===");
        System.out.println("  Target: " + targetPosition + " rotations");
        System.out.println("  Current position: " + endEffector.getPosition());
        System.out.println("  Distance to travel: " + Math.abs(endEffector.getPosition() - targetPosition));
        System.out.println("  Tolerance: " + tolerance);

        timeoutTimer.restart();
        progressTimer.restart();
        endEffector.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        // Log progress every 0.5 seconds
        if (progressTimer.hasElapsed(0.5)) {
            double currentPos = endEffector.getPosition();
            double targetPos = endEffector.getTargetPosition();
            double error = Math.abs(currentPos - targetPos);
            boolean atTarget = endEffector.atTarget(tolerance);

            System.out.println("EndEffector Progress: current=" + String.format("%.3f", currentPos) +
                    ", target=" + String.format("%.3f", targetPos) +
                    ", error=" + String.format("%.3f", error) +
                    ", atTarget=" + atTarget +
                    ", time=" + String.format("%.1f", timeoutTimer.get()) + "s");

            progressTimer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        // TIMEOUT SAFETY: Command finishes after 3 seconds regardless
        if (timeoutTimer.hasElapsed(COMMAND_TIMEOUT_SECONDS)) {
            System.out.println("=== SetEndEffectorPositionCommand TIMEOUT ===");
            System.out.println("  TIMEOUT after " + COMMAND_TIMEOUT_SECONDS + " seconds - FORCING COMPLETION");
            System.out.println("  Current: " + String.format("%.3f", endEffector.getPosition()) + " rotations");
            System.out.println("  Target: " + String.format("%.3f", targetPosition) + " rotations");
            System.out.println("  Difference: "
                    + String.format("%.3f", Math.abs(endEffector.getPosition() - targetPosition)) + " rotations");
            return true; // Force finish to prevent stuck commands
        }

        // Command ONLY finishes when EndEffector actually reaches the target within
        // tolerance
        boolean atTarget = endEffector.atTarget(tolerance);

        // Additional safety: Check if we have valid position data
        double currentPos = endEffector.getPosition();
        double targetPos = endEffector.getTargetPosition();

        if (Double.isNaN(currentPos) || Double.isNaN(targetPos) ||
                Double.isInfinite(currentPos) || Double.isInfinite(targetPos)) {
            System.out.println("=== SetEndEffectorPositionCommand INVALID DATA ===");
            System.out.println("  Invalid position data detected, finishing command");
            return true; // Finish to prevent infinite loops
        }

        if (atTarget) {
            System.out.println("=== SetEndEffectorPositionCommand SUCCESS ===");
            System.out.println("  EndEffector reached target position " +
                    String.format("%.3f", currentPos) + " rotations (tolerance: " + tolerance + " rotations)");
            System.out.println("  Command completed in " + String.format("%.2f", timeoutTimer.get()) + " seconds");
        }

        return atTarget;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("SetEndEffectorPositionCommand interrupted at position: " +
                    endEffector.getPosition() + " rotations (target was " + targetPosition + " rotations)");
        } else {
            System.out.println("SetEndEffectorPositionCommand completed successfully: " +
                    endEffector.getPosition() + " rotations reached");
        }
    }
}
