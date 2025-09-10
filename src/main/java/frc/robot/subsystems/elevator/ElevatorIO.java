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

    public abstract class TalonFXBase implements ElevatorIO {
        protected final TalonFX leadMotor;
        protected final TalonFX followerMotor;
        protected final MotionMagicVoltage control;
        protected double targetMeters = 0.0;

        public TalonFXBase() {
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
            softLimits.ForwardSoftLimitEnable = true;
            softLimits.ReverseSoftLimitEnable = true;
            softLimits.ForwardSoftLimitThreshold = constElevator.maxHeightMeters * constElevator.rotationsPerMeter;
            softLimits.ReverseSoftLimitThreshold = constElevator.minHeightMeters * constElevator.rotationsPerMeter;

            leadMotor.getConfigurator().apply(config);
            followerMotor.setControl(new Follower(constElevator.leadMotorId, true));
        }

        @Override
        public void setHeight(double meters) {
            targetMeters = meters;
            leadMotor.setControl(control.withPosition(meters * constElevator.rotationsPerMeter));
        }

        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
            inputs.leadAppliedVolts = leadMotor.getMotorVoltage().getValueAsDouble();
            inputs.leadCurrentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();
            inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
            inputs.followerCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
            inputs.targetMeters = targetMeters;

            updateInterfaceInputs(inputs);
        }

        protected abstract void updateInterfaceInputs(ElevatorIOInputs inputs);
    }
}
