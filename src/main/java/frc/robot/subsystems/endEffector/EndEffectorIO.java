package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.constEndEffector;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double position = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double targetPosition = 0.0;
        public double temperatureCelsius = 0.0;

        // Outtake motor inputs
        public double outtakeAppliedVolts = 0.0;
        public double outtakeCurrentAmps = 0.0;
        public double outtakeSpeed = 0.0;
        public double outtakeTemperatureCelsius = 0.0;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {
    }

    public default void setPosition(double position) {
    }

    public default void setOuttakeSpeed(double speed) {
    }

    // Provide a no-op default to allow clearing hold from callers if desired
    public default void clearPositionHold() {
    }

    public abstract class TalonFXBase implements EndEffectorIO {
        protected final TalonFX motor;
        protected final TalonFX outtakeMotor; // Add outtake motor
        protected final PositionVoltage control;
        protected final VoltageOut outtakeControl; // Add outtake control
        protected double targetPosition = 0.0;
        protected double currentOuttakeSpeed = 0.0;

        // Sign applied when converting between position commands and motor rotations.
        // Set by subclasses via constructor.
        protected final int positionSign;

        // Tracks whether a position target was explicitly requested (prevents implicit
        // movement on startup)
        protected boolean hasTarget = false;

        // Default constructor keeps legacy behavior (positive sign)
        public TalonFXBase() {
            this(1);
        }

        // New constructor that takes position sign so subclasses can set polarity
        public TalonFXBase(int positionSign) {
            this.positionSign = positionSign;

            motor = new TalonFX(constEndEffector.motorId);
            outtakeMotor = new TalonFX(constEndEffector.outtakeMotorId); // Initialize outtake motor
            control = new PositionVoltage(0);
            outtakeControl = new VoltageOut(0); // Initialize outtake control

            TalonFXConfiguration config = new TalonFXConfiguration();
            TalonFXConfiguration outtakeConfig = new TalonFXConfiguration();

            // PID gains for position control (Slot 1 - different from elevator)
            config.Slot1.kP = constEndEffector.kP;
            config.Slot1.kI = constEndEffector.kI;
            config.Slot1.kD = constEndEffector.kD;

            // Apply speed modifier through motor output limits
            config.MotorOutput.PeakForwardDutyCycle = constEndEffector.speedModifier;
            config.MotorOutput.PeakReverseDutyCycle = -constEndEffector.speedModifier;

            // Set BRAKE MODE for both motors to prevent coasting
            config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
            outtakeConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

            // Software limit switches to prevent over-rotation
            SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
            // Compute signed thresholds then ensure forward > reverse per controller
            // expectations
            double rawForward = this.positionSign * constEndEffector.maxPosition;
            double rawReverse = this.positionSign * constEndEffector.minPosition;
            softLimits.ForwardSoftLimitEnable = true;
            softLimits.ReverseSoftLimitEnable = true;
            softLimits.ForwardSoftLimitThreshold = Math.max(rawForward, rawReverse);
            softLimits.ReverseSoftLimitThreshold = Math.min(rawForward, rawReverse);

            motor.getConfigurator().apply(config);
            outtakeMotor.getConfigurator().apply(outtakeConfig); // Configure outtake motor

            System.out.println("EndEffector PID configured:");
            System.out.println("  Position range: " + constEndEffector.minPosition + " to "
                    + constEndEffector.maxPosition + " rotations");
            System.out.println("  PID gains (Slot 1): P=" + constEndEffector.kP + ", I=" + constEndEffector.kI + ", D="
                    + constEndEffector.kD);
            System.out.println("  Position sign: " + this.positionSign);
            System.out.println("  Speed modifier: " + (constEndEffector.speedModifier * 100) + "% max output");
            System.out.println("  Pivot motor ID " + constEndEffector.motorId + ": BRAKE mode enabled");
            System.out.println("  Outtake motor ID " + constEndEffector.outtakeMotorId + ": BRAKE mode enabled");
        }

        @Override
        public void setPosition(double position) {
            // Clamp commanded position to software limits
            double clamped = Math.max(constEndEffector.minPosition,
                    Math.min(constEndEffector.maxPosition, position));

            if (clamped != position) {
                System.out.println("EndEffector: Position " + position + " clamped to " + clamped);
            }

            targetPosition = clamped;
            hasTarget = true;

            // Simple position command using Slot 1 - speed is limited by motor output
            // configuration
            motor.setControl(
                    control.withPosition(positionSign * clamped)
                            .withSlot(1) // Use Slot 1 instead of Slot 0
                            .withFeedForward(0.0));

            System.out.println("EndEffector: Moving to position " + clamped + " rotations (Slot 1, max output: " +
                    (constEndEffector.speedModifier * 100) + "%)");
        }

        @Override
        public void setOuttakeSpeed(double speed) {
            // Clamp speed to reasonable range
            double clampedSpeed = Math.max(-1.0, Math.min(1.0, speed));
            currentOuttakeSpeed = clampedSpeed;

            // Convert speed to voltage (12V max)
            double voltage = clampedSpeed * 12.0;
            outtakeMotor.setControl(outtakeControl.withOutput(voltage));

            if (Math.abs(clampedSpeed) > 0.01) {
                System.out.println("EndEffector: Outtake motor speed set to " + clampedSpeed + " (" + voltage + "V)");
            }
        }

        @Override
        public void updateInputs(EndEffectorIOInputs inputs) {
            inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
            inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
            inputs.targetPosition = targetPosition;
            inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();

            // Update outtake motor inputs
            inputs.outtakeAppliedVolts = outtakeMotor.getMotorVoltage().getValueAsDouble();
            inputs.outtakeCurrentAmps = outtakeMotor.getSupplyCurrent().getValueAsDouble();
            inputs.outtakeSpeed = currentOuttakeSpeed;
            inputs.outtakeTemperatureCelsius = outtakeMotor.getDeviceTemp().getValueAsDouble();

            updateInterfaceInputs(inputs);

            // Re-assert the current PID position command only if someone requested a hold
            try {
                if (hasTarget && !Double.isNaN(targetPosition) && !Double.isInfinite(targetPosition)) {
                    motor.setControl(control.withPosition(positionSign * targetPosition).withSlot(1));
                }
            } catch (Exception e) {
                System.out.println("EndEffector: failed to reapply control: " + e.getMessage());
            }
        }

        @Override
        public void clearPositionHold() {
            hasTarget = false;
            System.out.println("EndEffector: Position hold cleared");
        }

        protected abstract void updateInterfaceInputs(EndEffectorIOInputs inputs);
    }
}