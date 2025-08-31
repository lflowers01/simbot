package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.constElevator;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX leadMotor;
    private final MotionMagicVoltage control;
    private double targetMeters = 0.0;

    public ElevatorIOReal() {
        leadMotor = new TalonFX(constElevator.LEAD_MOTOR_ID);
        control = new MotionMagicVoltage(0);
        // TODO: Apply configs (kP, kD, MotionMagic, etc.) same as before
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = leadMotor.getPosition().getValueAsDouble() / constElevator.ROTATIONS_PER_METER;
        inputs.velocityMetersPerSec = leadMotor.getVelocity().getValueAsDouble() / constElevator.ROTATIONS_PER_METER;
        inputs.appliedVolts = leadMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();
        inputs.targetMeters = targetMeters;
    }

    @Override
    public void setHeight(double meters) {
        targetMeters = meters;
        leadMotor.setControl(control.withPosition(meters * constElevator.ROTATIONS_PER_METER));
    }

}
