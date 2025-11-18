package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import edu.wpi.first.wpilibj.Timer;

public class SetElevatorHeightCommand extends Command {
    private final Elevator elevator;
    private final double targetHeight;
    private final double tolerance;
    private final Timer timeoutTimer = new Timer();
    private static final double COMMAND_TIMEOUT_SECONDS = 8.0; // Increased for testing

    /**
     * Command to move elevator to a specific height
     * 
     * @param elevator     The Elevator subsystem
     * @param targetHeight Target height in meters
     */
    public SetElevatorHeightCommand(Elevator elevator, double targetHeight) {
        this(elevator, targetHeight, 0.05); // Default 5cm tolerance
    }

    /**
     * Command to move elevator to a specific height with custom tolerance
     * 
     * @param elevator     The Elevator subsystem
     * @param targetHeight Target height in meters
     * @param tolerance    Height tolerance in meters
     */
    public SetElevatorHeightCommand(Elevator elevator, double targetHeight, double tolerance) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        this.tolerance = Math.max(0.005, Math.abs(tolerance)); // Ensure minimum tolerance (5mm)

        addRequirements(elevator);

        System.out.println("SetElevatorHeightCommand created: target=" + targetHeight +
                "m, tolerance=" + this.tolerance + "m");
    }

    @Override
    public void initialize() {
        System.out.println("SetElevatorHeightCommand STARTING: " + targetHeight + "m");
        System.out.println("  Current height: " + String.format("%.4f", elevator.getHeight()) + "m");
        System.out.println(
                "  Distance to travel: " + String.format("%.4f", Math.abs(elevator.getHeight() - targetHeight)) + "m");

        timeoutTimer.restart();
        elevator.setHeight(targetHeight);
    }

    @Override
    public void execute() {
        // MotionMagic handles the movement - we just monitor
    }

    @Override
    public boolean isFinished() {
        // TIMEOUT SAFETY: Command finishes after 8 seconds regardless
        if (timeoutTimer.hasElapsed(COMMAND_TIMEOUT_SECONDS)) {
            System.out.println("SetElevatorHeightCommand: TIMEOUT after " + COMMAND_TIMEOUT_SECONDS + " seconds");
            System.out.println("  Current: " + String.format("%.4f", elevator.getHeight()) + "m");
            System.out.println("  Target: " + String.format("%.4f", targetHeight) + "m");
            System.out.println(
                    "  Difference: " + String.format("%.4f", Math.abs(elevator.getHeight() - targetHeight)) + "m");
            return true; // Force finish to prevent stuck commands
        }

        // Command ONLY finishes when elevator actually reaches the target within
        // tolerance
        boolean atTarget = elevator.atTarget(tolerance);

        // Additional safety: Check if we have valid height data
        double currentHeight = elevator.getHeight();
        double targetHeight = elevator.getTargetHeight();

        if (Double.isNaN(currentHeight) || Double.isNaN(targetHeight) ||
                Double.isInfinite(currentHeight) || Double.isInfinite(targetHeight)) {
            System.out.println("SetElevatorHeightCommand: Invalid height data detected, finishing command");
            return true; // Finish to prevent infinite loops
        }

        if (atTarget) {
            System.out.println("SetElevatorHeightCommand: Elevator reached target height " +
                    String.format("%.4f", currentHeight) + "m (tolerance: " + tolerance + "m)");
        }

        return atTarget;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("SetElevatorHeightCommand interrupted at height: " +
                    elevator.getHeight() + "m (target was " + targetHeight + "m)");
        } else {
            System.out.println("SetElevatorHeightCommand completed successfully: " +
                    elevator.getHeight() + "m reached");
        }
    }
}
