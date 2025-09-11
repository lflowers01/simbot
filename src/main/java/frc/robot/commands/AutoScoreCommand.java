package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constAutoScore;
import frc.robot.commands.AutoAlignCommand.AlignmentSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.geometry.Pose3d;

public class AutoScoreCommand extends Command {
    public enum ScoringPhase {
        AUTO_ALIGN_AND_ELEVATOR,
        SCORING_ACTIVE,
        RETURN_TO_IDLE,
        FINISHED
    }

    private final Drive drivetrain;
    private final Vision vision;
    private final Elevator elevator;
    private final AlignmentSide alignmentSide;
    private final double targetElevatorLevel;

    private ScoringPhase currentPhase;
    private AutoAlignCommand autoAlignCommand;
    private final Timer phaseTimer = new Timer();
    private Pose3d targetTagPose; // Store the target tag pose for distance calculation

    public AutoScoreCommand(Drive drivetrain, Vision vision, Elevator elevator,
            AlignmentSide alignmentSide, double targetElevatorLevel) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.elevator = elevator;
        this.alignmentSide = alignmentSide;
        this.targetElevatorLevel = targetElevatorLevel;

        // Only require elevator - drivetrain requirement will be managed dynamically
        addRequirements(elevator);

        System.out.println("=== Auto Score Command Created ===");
        System.out.println("Alignment side: " + alignmentSide);
        System.out.println("Target elevator level: " + targetElevatorLevel + "m");
        System.out.println("Completion radius: " + constAutoScore.completionRadiusMeters + "m");
        System.out.println("Elevator will move concurrently with auto-alignment");
    }

    @Override
    public void initialize() {
        currentPhase = ScoringPhase.AUTO_ALIGN_AND_ELEVATOR;
        phaseTimer.restart();

        System.out.println("Starting scoring sequence - Phase: AUTO_ALIGN_AND_ELEVATOR");

        // Get the best tag pose and start auto-alignment
        var bestTagPose = vision.getBestTagPose();
        if (bestTagPose != null) {
            targetTagPose = bestTagPose; // Store for distance calculations
            System.out.println("Starting auto-align to tag: " + bestTagPose);
            autoAlignCommand = new AutoAlignCommand(drivetrain, vision, bestTagPose, alignmentSide);
            autoAlignCommand.initialize();

            // Start elevator movement immediately alongside auto-alignment
            System.out.println("Starting elevator movement to " + targetElevatorLevel + "m concurrently");
            elevator.setHeight(targetElevatorLevel);
        } else {
            System.out.println("No valid AprilTag found - moving elevator only");
            targetTagPose = null;
            elevator.setHeight(targetElevatorLevel);
            // Skip directly to scoring active since there's no auto-align
            currentPhase = ScoringPhase.SCORING_ACTIVE;
            phaseTimer.restart();
        }
    }

    @Override
    public void execute() {
        switch (currentPhase) {
            case AUTO_ALIGN_AND_ELEVATOR:
                if (autoAlignCommand != null) {
                    autoAlignCommand.execute();

                    // Check if auto-align is finished
                    if (autoAlignCommand.isFinished()) {
                        autoAlignCommand.end(false);
                        autoAlignCommand = null;

                        System.out.println("Auto-align completed - Phase: SCORING_ACTIVE");
                        System.out.println("Elevator should be at target level (moved concurrently)");
                        System.out.println("Drivetrain now available for manual control during scoring");
                        System.out.println("Drive away from tag (>" + constAutoScore.completionRadiusMeters
                                + "m) to complete scoring");
                        currentPhase = ScoringPhase.SCORING_ACTIVE;
                        phaseTimer.restart();
                    }
                } else {
                    // No auto-align command, just wait for elevator timeout then proceed
                    if (phaseTimer.hasElapsed(constAutoScore.elevatorMovementTimeoutSeconds)) {
                        System.out.println("Elevator timeout reached - Phase: SCORING_ACTIVE");
                        currentPhase = ScoringPhase.SCORING_ACTIVE;
                        phaseTimer.restart();
                    }
                }
                break;

            case SCORING_ACTIVE:
                // Check distance from target tag - robot can be driven manually
                if (targetTagPose != null) {
                    // Get current robot position
                    var currentPose = drivetrain.getPose();
                    var tagPosition = targetTagPose.getTranslation().toTranslation2d();
                    var robotPosition = currentPose.getTranslation();
                    double distanceToTag = robotPosition.getDistance(tagPosition);

                    // Check if robot has moved far enough from the tag
                    if (distanceToTag > constAutoScore.completionRadiusMeters) {
                        System.out.println("Robot moved away from tag (" + String.format("%.2f", distanceToTag) +
                                "m > " + constAutoScore.completionRadiusMeters + "m) - Phase: RETURN_TO_IDLE");
                        currentPhase = ScoringPhase.RETURN_TO_IDLE;
                        phaseTimer.restart();

                        // Return elevator to idle
                        elevator.setHeight(constElevator.idle);
                    }
                } else {
                    // Fallback to time-based completion if no tag pose available
                    if (phaseTimer.hasElapsed(constAutoScore.scoringDelaySeconds)) {
                        System.out.println("Scoring timeout (no tag pose) - Phase: RETURN_TO_IDLE");
                        currentPhase = ScoringPhase.RETURN_TO_IDLE;
                        phaseTimer.restart();

                        // Return elevator to idle
                        elevator.setHeight(constElevator.idle);
                    }
                }
                break;

            case RETURN_TO_IDLE:
                // Wait for elevator to return to idle (robot can still be driven)
                if (phaseTimer.hasElapsed(constAutoScore.elevatorMovementTimeoutSeconds)) {
                    System.out.println("Elevator returned to idle - Phase: FINISHED");
                    currentPhase = ScoringPhase.FINISHED;
                }
                break;

            case FINISHED:
                // Command is complete
                break;
        }

        // Log progress every 0.5 seconds
        if (phaseTimer.get() % 0.5 < 0.02) {
            String driveStatus;
            if (currentPhase == ScoringPhase.AUTO_ALIGN_AND_ELEVATOR) {
                driveStatus = (autoAlignCommand != null) ? " (auto-driving + elevator moving)"
                        : " (elevator moving only)";
            } else {
                driveStatus = " (manual drive available)";
            }

            String distanceInfo = "";
            if (currentPhase == ScoringPhase.SCORING_ACTIVE && targetTagPose != null) {
                var currentPose = drivetrain.getPose();
                var tagPosition = targetTagPose.getTranslation().toTranslation2d();
                double distanceToTag = currentPose.getTranslation().getDistance(tagPosition);
                distanceInfo = " [dist to tag: " + String.format("%.2f", distanceToTag) + "m]";
            }

            System.out.println("Scoring sequence - Phase: " + currentPhase +
                    ", Time: " + String.format("%.1f", phaseTimer.get()) + "s" + driveStatus + distanceInfo);
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == ScoringPhase.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up auto-align command if still running
        if (autoAlignCommand != null) {
            autoAlignCommand.end(true);
        }

        if (interrupted) {
            System.out.println("Auto score command interrupted at phase: " + currentPhase);
            // Return elevator to idle if interrupted
            elevator.setHeight(constElevator.idle);
        } else {
            System.out.println("Auto score command completed successfully");
        }

        System.out.println("Drivetrain fully available for manual control");
    }
}
