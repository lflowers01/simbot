// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constVision;
import frc.robot.commands.DriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.commands.AutoAlignCommand;

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
                                                joystick.rightBumper(), // Use rightBumper() directly as BooleanSupplier
                                                maxSpeed,
                                                maxAngularRate));

                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // joystick.pov(0).whileTrue(
                // drivetrain.applyRequest(() ->
                // forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                // joystick.pov(180)
                // .whileTrue(drivetrain.applyRequest(
                // () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                joystick.x().onTrue(new InstantCommand(() -> elevator.setHeight(constElevator.idle)));

                joystick.pov(180).onTrue(new InstantCommand(() -> elevator.setHeight(constElevator.l1)));
                joystick.pov(270).onTrue(new InstantCommand(() -> elevator.setHeight(constElevator.l2)));
                joystick.pov(0).onTrue(new InstantCommand(() -> elevator.setHeight(constElevator.l3)));
                joystick.pov(90).onTrue(new InstantCommand(() -> elevator.setHeight(constElevator.l4)));

                // Auto-align to best AprilTag when Y button is pressed
                joystick.y().onTrue(new InstantCommand(() -> {
                        var bestTagPose = vision.getBestTagPose();
                        if (bestTagPose != null) {
                                new AutoAlignCommand(drivetrain, bestTagPose).schedule();
                        } else {
                                System.out.println("No valid AprilTag found for auto-alignment");
                        }
                }));

                // VISION - Remove the manual periodic call
                // vision.periodic(); // Remove this line

                drivetrain.registerTelemetry(logger::telemeterize);
        }

}
