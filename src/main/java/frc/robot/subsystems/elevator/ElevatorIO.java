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
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double targetMeters = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setHeight(double meters) {}

    public abstract class TalonFXBase implements ElevatorIO {
        protected final TalonFX leadMotor;
        protected final TalonFX followerMotor;
        protected final MotionMagicVoltage control;
        protected double targetMeters = 0.0;

        public TalonFXBase() {
            leadMotor = new TalonFX(constElevator.LEAD_MOTOR_ID);
            followerMotor = new TalonFX(constElevator.FOLLOW_MOTOR_ID);
            control = new MotionMagicVoltage(0);

            TalonFXConfiguration config = new TalonFXConfiguration();
            
            config.Slot0.kG = constElevator.KG;
            config.Slot0.kS = constElevator.KS;
            config.Slot0.kV = constElevator.KV;
            config.Slot0.kA = constElevator.KA;
            config.Slot0.kP = constElevator.KP;
            config.Slot0.kI = constElevator.KI;
            config.Slot0.kD = constElevator.KD;

            MotionMagicConfigs motionMagic = config.MotionMagic;
            motionMagic.MotionMagicCruiseVelocity = constElevator.motionVelocity * constElevator.ROTATIONS_PER_METER;
            motionMagic.MotionMagicAcceleration = constElevator.motionAcceleration * constElevator.ROTATIONS_PER_METER;
            motionMagic.MotionMagicExpo_kA = constElevator.EXPO_KA;
            motionMagic.MotionMagicExpo_kV = constElevator.EXPO_KV;
            
            SoftwareLimitSwitchConfigs softLimits = config.SoftwareLimitSwitch;
            softLimits.ForwardSoftLimitEnable = true;
            softLimits.ReverseSoftLimitEnable = true;
            softLimits.ForwardSoftLimitThreshold = constElevator.maxElevatorHeightMeters * constElevator.ROTATIONS_PER_METER;
            softLimits.ReverseSoftLimitThreshold = constElevator.minElevatorHeightMeters * constElevator.ROTATIONS_PER_METER;
            
            leadMotor.getConfigurator().apply(config);
            followerMotor.setControl(new Follower(constElevator.LEAD_MOTOR_ID, true));
        }

        @Override
        public void setHeight(double meters) {
            targetMeters = meters;
            leadMotor.setControl(control.withPosition(meters * constElevator.ROTATIONS_PER_METER));
        }

        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
            inputs.appliedVolts = leadMotor.getMotorVoltage().getValueAsDouble();
            inputs.currentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();
            inputs.targetMeters = targetMeters;
            
            updateInterfaceInputs(inputs);
        }

        protected abstract void updateInterfaceInputs(ElevatorIOInputs inputs);
    }
}
