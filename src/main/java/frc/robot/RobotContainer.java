// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constVision;
import frc.robot.Constants.constAutoAlign;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.DriveBackwardsCommand;
import frc.robot.commands.SetEndEffectorPositionCommand;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOReal;

public class RobotContainer {

        private final double maxSpeed = constDrivetrain.maxSpeed * constDrivetrain.speedModifier;
        private final double maxAngularRate = RotationsPerSecond.of(constDrivetrain.maxAngularRate)
                        .in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(maxSpeed * constDrivetrain.deadbandPercent)
                        .withRotationalDeadband(maxAngularRate * constDrivetrain.deadbandPercent)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

        private final Telemetry logger = new Telemetry(maxSpeed);
        private final CommandXboxController driveController = new CommandXboxController(0); // Port 0 for driving
        private final CommandXboxController operatorController = new CommandXboxController(1); // Port 1 for
                                                                                               // elevator/end effector

        public final Drive drivetrain = TunerConstants.createDrivetrain();
        private final Elevator elevator;
        private final Vision vision;
        private final EndEffector endEffector;

        // Queued scoring level state
        private double queuedScoringLevel = constElevator.l1; // Default to L1

        public RobotContainer() {
                if (RobotBase.isReal()) {

                        System.out.println("Running Elevator in Real Mode");
                        elevator = new Elevator(new ElevatorIOReal());

                        // Initialize EndEffector with real hardware only
                        endEffector = new EndEffector(new EndEffectorIOReal());

                        vision = new Vision(
                                        drivetrain::addVisionMeasurement,
                                        new VisionIOPhotonVision(constVision.camera0Name,
                                                        constVision.robotToCamera0),
                                        new VisionIOPhotonVision(constVision.camera1Name,
                                                        constVision.robotToCamera1));

                } else {
                        System.out.println("Running Elevator in Sim Mode");
                        elevator = new Elevator(new ElevatorIOSim());

                        // Fixed EndEffector simulation - include ALL required methods
                        endEffector = new EndEffector(new EndEffectorIO() {
                                @Override
                                public void updateInputs(EndEffectorIO.EndEffectorIOInputs inputs) {
                                        // Simulate basic position tracking
                                        inputs.position = inputs.targetPosition; // Always at target in sim
                                        inputs.velocityRotPerSec = 0.0;
                                        inputs.appliedVolts = 0.0;
                                        inputs.currentAmps = 0.0;
                                        inputs.temperatureCelsius = 25.0;

                                        // Simulate outtake motor
                                        inputs.outtakeAppliedVolts = 0.0;
                                        inputs.outtakeCurrentAmps = 0.0;
                                        inputs.outtakeSpeed = 0.0;
                                        inputs.outtakeTemperatureCelsius = 25.0;
                                }

                                @Override
                                public void setPosition(double position) {
                                        System.out.println("EndEffector SIM: Would move to position " + position
                                                        + " (no simulation)");
                                }

                                @Override
                                public void setOuttakeSpeed(double speed) {
                                        System.out.println("EndEffector SIM: Would set outtake speed to " + speed
                                                        + " (no simulation)");
                                }

                                @Override
                                public void clearPositionHold() {
                                        System.out.println(
                                                        "EndEffector SIM: Would clear position hold (no simulation)");
                                }
                        });

                        vision = new Vision(
                                        drivetrain::addVisionMeasurement,
                                        new VisionIOPhotonVisionSim(constVision.camera0Name,
                                                        constVision.robotToCamera0, drivetrain::getPose),
                                        new VisionIOPhotonVisionSim(constVision.camera1Name,
                                                        constVision.robotToCamera1, drivetrain::getPose));

                }

                System.out.println("EndEffector subsystem initialized - Real hardware only, no simulation");
                System.out.println("Drive Controller: Port 0 (driving + scoring)");
                System.out.println("Operator Controller: Port 1 (elevator + end effector)");
                System.out.println("Autonomous: Drive backwards 1 meter");

                configureBindings();
        }

        private void configureBindings() {
                // ===========================================
                // DRIVE CONTROLLER (PORT 0) - FULLY ENABLED
                // ===========================================

                System.out.println("DRIVETRAIN CONTROLS FULLY ENABLED - All functionality restored");

                // Default drive command with field-centric control
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driveController.getLeftY() * maxSpeed)
                                                .withVelocityY(-driveController.getLeftX() * maxSpeed)
                                                .withRotationalRate(-driveController.getRightX() * maxAngularRate)));

                // Brake mode when X is held
                driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));

                // Point wheels in direction of right stick when A is held
                driveController.a().whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driveController.getRightY(),
                                                                -driveController.getRightX()))));

                // Reset gyro on start button
                driveController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Robot-centric drive while right bumper is held
                driveController.rightBumper().whileTrue(
                                drivetrain.applyRequest(() -> forwardStraight
                                                .withVelocityX(-driveController.getLeftY() * maxSpeed)
                                                .withVelocityY(-driveController.getLeftX() * maxSpeed)
                                                .withRotationalRate(-driveController.getRightX() * maxAngularRate)));

                // Slow mode while left bumper is held (50% speed)
                driveController.leftBumper().whileTrue(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driveController.getLeftY() * maxSpeed * 0.5)
                                                .withVelocityY(-driveController.getLeftX() * maxSpeed * 0.5)
                                                .withRotationalRate(
                                                                -driveController.getRightX() * maxAngularRate * 0.5)));

                // Disabled state handling
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // ===========================================
                // OPERATOR CONTROLLER (PORT 1) - ELEVATOR + END EFFECTOR COORDINATED
                // ===========================================

                // IMMEDIATE HOME OVERRIDE (X BUTTON) - Cancel everything and force home
                operatorController.x().onTrue(new InstantCommand(() -> {
                        System.out.println("=== IMMEDIATE HOME OVERRIDE - X BUTTON PRESSED ===");

                        // Cancel ALL commands immediately
                        var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
                        scheduler.cancelAll();
                        System.out.println("ALL COMMANDS CANCELLED");

                        // Immediately command both systems to home positions
                        System.out.println("Commanding EndEffector to stow position: "
                                        + Constants.constEndEffector.stowPosition);
                        endEffector.setPosition(Constants.constEndEffector.stowPosition);

                        System.out.println("Commanding Elevator to idle height: "
                                        + String.format("%.4f", constElevator.idle) + "m");
                        elevator.setHeight(constElevator.idle);

                        // Stop coral motor immediately
                        endEffector.stopOuttake();
                        System.out.println("Coral motor stopped");

                        System.out.println("HOME OVERRIDE COMPLETE - Both systems commanded to home positions");
                        System.out.println("No waiting, no conditions, immediate execution");
                }));

                // D-PAD FOR COORDINATED SCORING LEVELS - VERY ROBUST WITH LOOSE TOLERANCES
                operatorController.pov(180).onTrue(
                                // L1: EndEffector stows first, WAITS until done, then elevator goes down
                                new SetEndEffectorPositionCommand(endEffector, Constants.constEndEffector.stowPosition,
                                                0.05) // Updated tolerance
                                                .andThen(new SetElevatorHeightCommand(elevator, constElevator.l1, 0.05)) // Updated
                                                                                                                         // tolerance
                                                .withTimeout(10.0) // Long timeout
                                                .beforeStarting(() -> {
                                                        queuedScoringLevel = constElevator.l1;
                                                        System.out.println(
                                                                        "OPERATOR - D-pad DOWN: Sequential move to L1 ("
                                                                                        + String.format("%.3f",
                                                                                                        queuedScoringLevel)
                                                                                        + "m)");
                                                        System.out.println(
                                                                        "Step 1: EndEffector stowing to -0.15, then elevator to L1");
                                                })
                                                .finallyDo(() -> {
                                                        System.out.println("L1 sequence complete: Elevator="
                                                                        + String.format("%.3f", elevator.getHeight())
                                                                        + "m, EndEffector=" + String.format("%.3f",
                                                                                        endEffector.getPosition()));
                                                }));

                operatorController.pov(270).onTrue(
                                // L2: Elevator goes up first, WAITS until done, then EndEffector goes to L2
                                // position
                                new SetElevatorHeightCommand(elevator, constElevator.l2, 0.05) // Updated tolerance
                                                .andThen(new SetEndEffectorPositionCommand(endEffector,
                                                                Constants.constEndEffector.l2Position, 0.05)) // Updated
                                                                                                              // tolerance
                                                .withTimeout(10.0) // Long timeout
                                                .beforeStarting(() -> {
                                                        queuedScoringLevel = constElevator.l2;
                                                        System.out.println(
                                                                        "OPERATOR - D-pad LEFT: Sequential move to L2 ("
                                                                                        + String.format("%.3f",
                                                                                                        queuedScoringLevel)
                                                                                        + "m)");
                                                        System.out.println(
                                                                        "Step 1: Elevator moving to L2, then EndEffector to 0.0");
                                                })
                                                .finallyDo(() -> {
                                                        System.out.println("L2 sequence complete: Elevator="
                                                                        + String.format("%.3f", elevator.getHeight())
                                                                        + "m, EndEffector=" + String.format("%.3f",
                                                                                        endEffector.getPosition()));
                                                }));

                operatorController.pov(0).onTrue(
                                // L3: Elevator goes up first, WAITS until done, then EndEffector goes to L3
                                // position
                                new SetElevatorHeightCommand(elevator, constElevator.l3, 0.05) // Updated tolerance
                                                .andThen(new SetEndEffectorPositionCommand(endEffector,
                                                                Constants.constEndEffector.l3Position, 0.05)) // Updated
                                                                                                              // tolerance
                                                .withTimeout(10.0) // Long timeout
                                                .beforeStarting(() -> {
                                                        queuedScoringLevel = constElevator.l3;
                                                        System.out.println(
                                                                        "OPERATOR - D-pad UP: Sequential move to L3 ("
                                                                                        + String.format("%.3f",
                                                                                                        queuedScoringLevel)
                                                                                        + "m)");
                                                        System.out.println(
                                                                        "Step 1: Elevator moving to L3, then EndEffector to 6.8");
                                                })
                                                .finallyDo(() -> {
                                                        System.out.println("L3 sequence complete: Elevator="
                                                                        + String.format("%.3f", elevator.getHeight())
                                                                        + "m, EndEffector=" + String.format("%.3f",
                                                                                        endEffector.getPosition()));
                                                }));

                operatorController.pov(90).onTrue(
                                // L4: Elevator goes up first, WAITS until done, then EndEffector goes to L4
                                // position
                                new SetElevatorHeightCommand(elevator, constElevator.l4, 0.05) // Updated tolerance
                                                .andThen(new SetEndEffectorPositionCommand(endEffector,
                                                                Constants.constEndEffector.l4Position, 0.05)) // Updated
                                                                                                              // tolerance
                                                .withTimeout(10.0) // Long timeout
                                                .beforeStarting(() -> {
                                                        queuedScoringLevel = constElevator.l4;
                                                        System.out.println(
                                                                        "OPERATOR - D-pad RIGHT: Sequential move to L4 ("
                                                                                        + String.format("%.3f",
                                                                                                        queuedScoringLevel)
                                                                                        + "m)");
                                                        System.out.println(
                                                                        "Step 1: Elevator moving to L4, then EndEffector to 8.0");
                                                })
                                                .finallyDo(() -> {
                                                        System.out.println("L4 sequence complete: Elevator="
                                                                        + String.format("%.3f", elevator.getHeight())
                                                                        + "m, EndEffector=" + String.format("%.3f",
                                                                                        endEffector.getPosition()));
                                                }));

                // CORAL INTAKE/OUTTAKE CONTROLS
                // Left Bumper: Coral Intake (hold to run)
                operatorController.leftBumper().onTrue(new InstantCommand(() -> {
                        System.out.println("OPERATOR - Left Bumper pressed - starting coral intake");
                        endEffector.startIntake();
                }));

                operatorController.leftBumper().onFalse(new InstantCommand(() -> {
                        System.out.println("OPERATOR - Left Bumper released - stopping coral intake");
                        endEffector.stopOuttake();
                }));

                // Right Bumper: Coral Outtake/Scoring (hold to run)
                operatorController.rightBumper().onTrue(new InstantCommand(() -> {
                        System.out.println("OPERATOR - Right Bumper pressed - starting coral outtake");
                        endEffector.startOuttake();
                }));

                operatorController.rightBumper().onFalse(new InstantCommand(() -> {
                        System.out.println("OPERATOR - Right Bumper released - stopping coral outtake");
                        endEffector.stopOuttake();
                }));

                // Enhanced Debug and status
                operatorController.start().onTrue(new InstantCommand(() -> {
                        System.out.println("=== ELEVATOR + END EFFECTOR DEBUG STATUS ===");
                        System.out.println("Queued scoring level: " + queuedScoringLevel + "m");
                        System.out.println("");
                        System.out.println("ELEVATOR STATUS:");
                        System.out.println("  Current height: " + String.format("%.4f", elevator.getHeight()) + "m");
                        System.out.println(
                                        "  Target height: " + String.format("%.4f", elevator.getTargetHeight()) + "m");
                        System.out.println(
                                        "  Height difference: "
                                                        + String.format("%.4f",
                                                                        Math.abs(elevator.getHeight()
                                                                                        - elevator.getTargetHeight()))
                                                        + "m");
                        System.out.println("  At target (0.05m): " + elevator.atTarget(0.05));
                        System.out.println("  Idle position: " + String.format("%.4f", constElevator.idle) + "m");
                        System.out.println("");
                        System.out.println("END EFFECTOR STATUS:");
                        System.out.println("  Current position: " + String.format("%.3f", endEffector.getPosition())
                                        + " rotations");
                        System.out.println("  Target position: "
                                        + String.format("%.3f", endEffector.getTargetPosition()) + " rotations");
                        System.out.println(
                                        "  Position difference: "
                                                        + String.format("%.3f",
                                                                        Math.abs(endEffector.getPosition() - endEffector
                                                                                        .getTargetPosition()))
                                                        + " rotations");
                        System.out.println("  At target (0.1): " + endEffector.atTarget(0.1));
                        System.out.println("  Stow position: "
                                        + String.format("%.3f", Constants.constEndEffector.stowPosition)
                                        + " rotations");
                        System.out.println("");
                        System.out.println("CORAL MOTOR STATUS:");
                        System.out.println("  Speed: " + endEffector.getOuttakeSpeed());
                        System.out.println("  Running: " + endEffector.isOuttakeRunning());
                        System.out.println("");
                        System.out.println("COMMAND SCHEDULER STATUS:");
                        var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
                        var elevatorCommand = scheduler.requiring(elevator);
                        var endEffectorCommand = scheduler.requiring(endEffector);
                        System.out.println("  Elevator command: "
                                        + (elevatorCommand != null ? elevatorCommand.getName() : "NONE"));
                        System.out.println("  EndEffector command: "
                                        + (endEffectorCommand != null ? endEffectorCommand.getName() : "NONE"));
                }));

                operatorController.back().onTrue(new InstantCommand(() -> {
                        System.out.println("=== ROBOT CONTROLS HELP ===");
                        System.out.println("");
                        System.out.println("DRIVE CONTROLS (Port 0 - Driver):");
                        System.out.println("  Left stick: Drive translation (X/Y movement)");
                        System.out.println("  Right stick: Drive rotation");
                        System.out.println("  Right bumper: Robot-centric mode (hold)");
                        System.out.println("  Left bumper: Slow mode - 50% speed (hold)");
                        System.out.println("  X: Brake mode (hold)");
                        System.out.println("  A: Point wheels toward right stick (hold)");
                        System.out.println("  Start: Reset field-centric gyro");
                        System.out.println("");
                        System.out.println("ELEVATOR + END EFFECTOR (Port 1 - Operator):");
                        System.out.println(
                                        "  D-pad: Coordinated elevator + EndEffector (DOWN=L1, LEFT=L2, UP=L3, RIGHT=L4)");
                        System.out.println("  X: HOME - Return both to home/idle positions + stop coral motor");
                        System.out.println("  Left Bumper: Coral intake (hold to run)");
                        System.out.println("  Right Bumper: Coral outtake/score (hold to run)");
                        System.out.println("  Start: Status debug");
                        System.out.println("");
                        System.out.println("PIVOT POSITIONING: All positions relative to motor startup position");
                        System.out.println("  Startup position = 0.0 rotations (reference point)");
                        System.out.println("  Negative values = backward from startup");
                        System.out.println("  Positive values = forward from startup");
                }));

                // Re-enable telemetry logging
                drivetrain.registerTelemetry(logger::telemeterize);

                System.out.println("=== DRIVETRAIN FUNCTIONALITY SUMMARY ===");
                System.out.println("✅ Field-centric driving (default)");
                System.out.println("✅ Robot-centric driving (right bumper)");
                System.out.println("✅ Slow mode (left bumper - 50% speed)");
                System.out.println("✅ Brake mode (X button)");
                System.out.println("✅ Wheel pointing (A button)");
                System.out.println("✅ Gyro reset (start button)");
                System.out.println("✅ Telemetry logging");
                System.out.println("✅ Vision integration ready");
                System.out.println("✅ Autonomous support ready");
        }

        /**
         * Gets the autonomous command - simple drive backwards 1 meter
         * 
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new DriveBackwardsCommand(drivetrain);
        }

}
