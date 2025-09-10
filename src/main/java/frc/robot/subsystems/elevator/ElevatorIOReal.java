package frc.robot.subsystems.elevator;

import frc.robot.Constants.constElevator;

public class ElevatorIOReal extends ElevatorIO.TalonFXBase {
    @Override
    protected void updateInterfaceInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = leadMotor.getPosition().getValueAsDouble() / constElevator.rotationsPerMeter;
        inputs.velocityMetersPerSec = leadMotor.getVelocity().getValueAsDouble() / constElevator.rotationsPerMeter;
    }
}
