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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constVision;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
        private final CommandXboxController joystick = new CommandXboxController(constDrivetrain.joystickPort);

        public final Drive drivetrain = TunerConstants.createDrivetrain();
        private final Elevator elevator;
        private final Vision vision;

        // Queued scoring level state
        private double queuedScoringLevel = constElevator.l1; // Default to L1

        public RobotContainer() {
                if (RobotBase.isReal()) {

                        System.out.println("Running Elevator in Real Mode");
                        elevator = new Elevator(new ElevatorIOReal());
                        vision = new Vision(
                                        drivetrain::addVisionMeasurement,
                                        new VisionIOPhotonVision(constVision.camera0Name,
                                                        constVision.robotToCamera0),
                                        new VisionIOPhotonVision(constVision.camera1Name,
                                                        constVision.robotToCamera1));

                } else {
                        System.out.println("Running Elevator in Sim Mode");
                        elevator = new Elevator(new ElevatorIOSim());
                        vision = new Vision(
                                        drivetrain::addVisionMeasurement,
                                        new VisionIOPhotonVisionSim(constVision.camera0Name,
                                                        constVision.robotToCamera0, drivetrain::getPose),
                                        new VisionIOPhotonVisionSim(constVision.camera1Name,
                                                        constVision.robotToCamera1, drivetrain::getPose));

                }
                configureBindings();
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                new DriveCommand(
                                                drivetrain,
                                                () -> -joystick.getLeftY(),
                                                () -> -joystick.getLeftX(),
                                                () -> -joystick.getRightX(),
                                                () -> false, // Remove right bumper from drive command
                                                maxSpeed,
                                                maxAngularRate));

                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Left bumper for LEFT auto-align and score
                joystick.leftBumper().onTrue(new InstantCommand(() -> {
                        System.out.println("Left bumper pressed - starting LEFT auto-align and score sequence");
                        System.out.println("Queued scoring level: " + queuedScoringLevel + "m");

                        // Create and schedule the complete scoring sequence
                        new AutoScoreCommand(drivetrain, vision, elevator,
                                        AutoAlignCommand.AlignmentSide.LEFT, queuedScoringLevel).schedule();
                }));

                // Right bumper for RIGHT auto-align and score
                joystick.rightBumper().onTrue(new InstantCommand(() -> {
                        System.out.println("Right bumper pressed - starting RIGHT auto-align and score sequence");
                        System.out.println("Queued scoring level: " + queuedScoringLevel + "m");

                        // Create and schedule the complete scoring sequence
                        new AutoScoreCommand(drivetrain, vision, elevator,
                                        AutoAlignCommand.AlignmentSide.RIGHT, queuedScoringLevel).schedule();
                }));

                // Y button for field-centric heading reset
                joystick.y().onTrue(new InstantCommand(() -> {
                        System.out.println("Y button pressed - resetting field-centric heading");
                        drivetrain.seedFieldCentric();
                }));

                // X button for manual elevator idle (emergency/manual control)
                joystick.x().onTrue(new InstantCommand(() -> {
                        System.out.println("X button pressed - manual elevator to idle");
                        elevator.setHeight(constElevator.idle);
                }));

                // D-pad for queuing scoring levels
                joystick.pov(180).onTrue(new InstantCommand(() -> {
                        queuedScoringLevel = constElevator.l1;
                        System.out.println(
                                        "D-pad DOWN pressed - queued scoring level: L1 (" + queuedScoringLevel + "m)");
                }));

                joystick.pov(270).onTrue(new InstantCommand(() -> {
                        queuedScoringLevel = constElevator.l2;
                        System.out.println(
                                        "D-pad LEFT pressed - queued scoring level: L2 (" + queuedScoringLevel + "m)");
                }));

                joystick.pov(0).onTrue(new InstantCommand(() -> {
                        queuedScoringLevel = constElevator.l3;
                        System.out.println("D-pad UP pressed - queued scoring level: L3 (" + queuedScoringLevel + "m)");
                }));

                joystick.pov(90).onTrue(new InstantCommand(() -> {
                        queuedScoringLevel = constElevator.l4;
                        System.out.println(
                                        "D-pad RIGHT pressed - queued scoring level: L4 (" + queuedScoringLevel + "m)");
                }));

                // Debug vision system with Back button
                joystick.back().onTrue(new InstantCommand(() -> {
                        System.out.println("=== Vision Debug ===");
                        var latestPose = vision.getLatestVisionPose();
                        var bestTag = vision.getBestTagPose();
                        System.out.println("Latest vision pose: " + latestPose);
                        System.out.println("Best tag pose: " + bestTag);
                        System.out.println("Current drivetrain pose: " + drivetrain.getPose());
                        System.out.println("Queued scoring level: " + queuedScoringLevel + "m");
                }));

                drivetrain.registerTelemetry(logger::telemeterize);
        }
}
