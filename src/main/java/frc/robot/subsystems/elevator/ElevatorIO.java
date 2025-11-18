package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.constElevator;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double leadAppliedVolts = 0.0;
        public double leadCurrentAmps = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double targetMeters = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setHeight(double meters) {
    }

    // Provide a no-op default to allow clearing hold from callers if desired
    public default void clearHeightHold() {
    }

    public abstract class TalonFXBase implements ElevatorIO {
        protected final TalonFX leadMotor;
        protected final TalonFX followerMotor;
        protected final MotionMagicVoltage control;
        protected double targetMeters = 0.0;

        // Sign applied when converting between meters <-> motor rotations.
        // Set by subclasses via constructor.
        protected final int positionSign;

        // Tracks whether a height target was explicitly requested (prevents implicit
        // movement on startup)
        protected boolean hasTarget = false;

        // Default constructor keeps legacy behavior (positive sign)
        public TalonFXBase() {
            this(1);
        }

        // New constructor that takes position sign so subclasses can set polarity
        public TalonFXBase(int positionSign) {
            this.positionSign = positionSign;

            leadMotor = new TalonFX(constElevator.leadMotorId);
            followerMotor = new TalonFX(constElevator.followMotorId);
            control = new MotionMagicVoltage(0);

            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kG = constElevator.kG;
            config.Slot0.kS = constElevator.kS;
            config.Slot0.kV = constElevator.kV;
            config.Slot0.kA = constElevator.kA;
            config.Slot0.kP = constElevator.kP;
            config.Slot0.kI = constElevator.kI;
            config.Slot0.kD = constElevator.kD;

            MotionMagicConfigs motionMagic = config.MotionMagic;
            motionMagic.MotionMagicCruiseVelocity = constElevator.motionVelocity * constElevator.rotationsPerMeter;
            motionMagic.MotionMagicAcceleration = constElevator.motionAcceleration * constElevator.rotationsPerMeter;
            motionMagic.MotionMagicExpo_kA = constElevator.expoKA;
            motionMagic.MotionMagicExpo_kV = constElevator.expoKV;

            SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
            // Compute signed thresholds then ensure forward > reverse per controller
            // expectations
            double rawForward = this.positionSign * constElevator.maxHeightMeters * constElevator.rotationsPerMeter;
            double rawReverse = this.positionSign * constElevator.minHeightMeters * constElevator.rotationsPerMeter;
            softLimits.ForwardSoftLimitEnable = true;
            softLimits.ReverseSoftLimitEnable = true;
            softLimits.ForwardSoftLimitThreshold = Math.max(rawForward, rawReverse);
            softLimits.ReverseSoftLimitThreshold = Math.min(rawForward, rawReverse);

            leadMotor.getConfigurator().apply(config);
            followerMotor.setControl(new Follower(constElevator.leadMotorId, true));
        }

        @Override
        public void setHeight(double meters) {
            // Clamp commanded height to software limits to avoid commanding outside valid
            // range
            double clamped = Math.max(constElevator.minHeightMeters, Math.min(constElevator.maxHeightMeters, meters));
            targetMeters = clamped;
            // Mark that a target was explicitly requested
            hasTarget = true;
            leadMotor.setControl(control.withPosition(positionSign * clamped * constElevator.rotationsPerMeter));
        }

        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
            inputs.leadAppliedVolts = leadMotor.getMotorVoltage().getValueAsDouble();
            inputs.leadCurrentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();
            inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
            inputs.followerCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
            inputs.targetMeters = targetMeters;

            updateInterfaceInputs(inputs);

            // Re-assert the current MotionMagic position command only if someone requested
            // a hold. This prevents implicit movement at startup when targetMeters defaults
            // to 0.
            try {
                if (hasTarget && !Double.isNaN(targetMeters) && !Double.isInfinite(targetMeters)) {
                    leadMotor.setControl(
                            control.withPosition(positionSign * targetMeters * constElevator.rotationsPerMeter));
                }
            } catch (Exception e) {
                System.out.println("Elevator: failed to reapply control: " + e.getMessage());
            }
        }

        @Override
        public void clearHeightHold() {
            // Stop asserting a closed-loop position target; caller may choose to
            // leave motor idle or apply different control.
            hasTarget = false;
        }

        protected abstract void updateInterfaceInputs(ElevatorIOInputs inputs);
    }
}
