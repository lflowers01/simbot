package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.constDrivetrain;
import frc.robot.subsystems.drive.Drive;

public class DriveCommand extends Command {
    private final Drive m_driveSubsystem;
    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;
    private final BooleanSupplier m_slowDrive;
    private final double m_maxSpeed;
    private final double m_maxAngularRate;

    /**
     * Creates a new DriveCommand.
     * 
     * @param driveSubsystem The drive subsystem
     * @param translation    Raw translation input supplier (forward/backward)
     * @param strafe         Raw strafe input supplier (left/right)
     * @param rotation       Raw rotation input supplier
     * @param slowDrive      Slow drive mode boolean supplier
     * @param maxSpeed       Maximum translation speed
     * @param maxAngularRate Maximum angular rate
     */
    public DriveCommand(
            Drive driveSubsystem,
            DoubleSupplier translation,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier slowDrive,
            double maxSpeed,
            double maxAngularRate) {

        m_driveSubsystem = driveSubsystem;
        m_translation = translation;
        m_strafe = strafe;
        m_rotation = rotation;
        m_slowDrive = slowDrive;
        m_maxSpeed = maxSpeed;
        m_maxAngularRate = maxAngularRate;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get raw joystick inputs
        double rawTranslation = m_translation.getAsDouble();
        double rawStrafe = m_strafe.getAsDouble();
        double rawRotation = m_rotation.getAsDouble();

        // Apply deadband to joystick inputs
        double translationValue = MathUtil.applyDeadband(
                rawTranslation,
                constDrivetrain.DEADBAND,
                1);
        double strafeValue = MathUtil.applyDeadband(
                rawStrafe,
                constDrivetrain.DEADBAND,
                1);
        double rotationValue = MathUtil.applyDeadband(
                rawRotation,
                constDrivetrain.DEADBAND,
                1);

        // Calculate magnitude before applying curve to preserve direction
        double magnitude = Math.sqrt(translationValue * translationValue + strafeValue * strafeValue);

        // Only apply processing if there's actual input
        if (magnitude > 0) {
            // Apply configurable exponent to magnitude while preserving direction
            double curvedMagnitude = Math.pow(magnitude, constDrivetrain.INPUT_CURVE);

            // Scale back to individual components
            double scale = curvedMagnitude / magnitude;
            translationValue *= scale;
            strafeValue *= scale;
        }

        // Apply curve to rotation separately
        rotationValue = Math.copySign(Math.pow(Math.abs(rotationValue), constDrivetrain.INPUT_CURVE), rotationValue);

        // Scale the output if slow drive is enabled
        if (m_slowDrive.getAsBoolean()) {
            translationValue *= constDrivetrain.HALF_SPEED_FACTOR;
            strafeValue *= constDrivetrain.HALF_SPEED_FACTOR;
            rotationValue *= constDrivetrain.HALF_SPEED_FACTOR;
        }

        // Translate input into velocities
        translationValue *= m_maxSpeed;
        strafeValue *= m_maxSpeed;
        rotationValue *= m_maxAngularRate;

        // Use raw rotation input for triggering to avoid false negatives from deadband
        boolean rotationTriggered = Math.abs(rawRotation) > constDrivetrain.DEADBAND;
        boolean rotationActive = MathUtil.isNear(m_driveSubsystem.rotationLastTriggered, Timer.getFPGATimestamp(),
                constDrivetrain.ROTATION_ACTIVE_TIMEOUT) &&
                (Math.abs(m_driveSubsystem.getState().Speeds.omegaRadiansPerSecond) > Math
                        .toRadians(constDrivetrain.ROTATION_ACTIVE_THRESHOLD_DEGREES));

        if (rotationTriggered) {
            m_driveSubsystem.rotationLastTriggered = Timer.getFPGATimestamp();
        }

        if (rotationTriggered || rotationActive) {
            m_driveSubsystem.setControl(
                    m_driveSubsystem.drive
                            .withVelocityX(translationValue)
                            .withVelocityY(strafeValue)
                            .withRotationalRate(rotationValue));
            m_driveSubsystem.currentHeading = Optional.empty();
        } else {
            if (m_driveSubsystem.currentHeading.isEmpty()) {
                m_driveSubsystem.currentHeading = Optional.of(m_driveSubsystem.getState().Pose.getRotation());
            }

            Optional<Alliance> alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                m_driveSubsystem.setControl(
                        m_driveSubsystem.driveMaintainHeading
                                .withVelocityX(translationValue)
                                .withVelocityY(strafeValue)
                                .withTargetDirection(
                                        alliance.get() == Alliance.Blue ? m_driveSubsystem.currentHeading.get()
                                                : m_driveSubsystem.currentHeading.get()
                                                        .rotateBy(Rotation2d.fromDegrees(180))));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        m_driveSubsystem.setControl(m_driveSubsystem.drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }
}
